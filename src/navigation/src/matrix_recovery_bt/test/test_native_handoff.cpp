// Software-only ROS integration, isolated domain; installed BT and real action transport.
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "matrix_recovery_bt/srv/recovery_handoff.hpp"
using namespace std::chrono_literals;
using Service=matrix_recovery_bt::srv::RecoveryHandoff;
using Plan=nav2_msgs::action::ComputePathToPose;
using Follow=nav2_msgs::action::FollowPath;
class Native : public testing::Test {
protected:
  static inline pid_t gate=-1;
  static void SetUpTestSuite() {
    ASSERT_NE(std::getenv("ROS_DOMAIN_ID"),nullptr);
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"190");
    gate=fork();ASSERT_GE(gate,0);
    if(gate==0) {execl(GATE_PATH,GATE_PATH,"--ros-args","-p","run_id:=native_test","-p","raw_topic:=/matrix/test/raw",nullptr);_exit(127);}
    int argc=0;rclcpp::init(argc,nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();if(gate>0){kill(gate,SIGINT);waitpid(gate,nullptr,0);}}
  rclcpp::Node::SharedPtr node,bt;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinning;
  rclcpp::Client<Service>::SharedPtr service;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw,safe;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr loc;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output;
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr map;
  rclcpp_action::Server<Plan>::SharedPtr planner;
  rclcpp_action::Server<Follow>::SharedPtr follower;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  rclcpp::TimerBase::SharedPtr inputs;
  std::vector<std::thread> actions;
  std::mutex mutex;
  std::vector<double> commands;
  std::atomic<int> plans{0},follows{0},cancels{0},odom_count{0};
  std::atomic<bool> moving{false},stale{false};
  int planning_ms=450,following_ms=40;
  void SetUp() override {
    node=std::make_shared<rclcpp::Node>("controller_server");bt=std::make_shared<rclcpp::Node>("test_bt_client");
    service=node->create_client<Service>("/matrix/recovery_handoff");
    raw=node->create_publisher<geometry_msgs::msg::Twist>("/matrix/test/raw",10);
    safe=node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe",10);
    odom=node->create_publisher<nav_msgs::msg::Odometry>("/odom/current_pose",rclcpp::SensorDataQoS());
    scan=node->create_publisher<sensor_msgs::msg::LaserScan>("/laser_scan",rclcpp::SensorDataQoS());
    loc=node->create_publisher<std_msgs::msg::UInt8>("/lightning/loc_status",10);
    valid=node->create_publisher<std_msgs::msg::Bool>("/lightning/pose_valid",10);
    tf=std::make_unique<tf2_ros::TransformBroadcaster>(node);
    output=node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav",100,[this](geometry_msgs::msg::Twist::ConstSharedPtr v){std::lock_guard<std::mutex> l(mutex);commands.push_back(v->linear.x);});
    map=node->create_service<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap",[this](const nav2_msgs::srv::GetCostmap::Request::SharedPtr,nav2_msgs::srv::GetCostmap::Response::SharedPtr r){
      auto & m=r->map;m.header.frame_id="odom";m.header.stamp=node->now();m.metadata.size_x=150;m.metadata.size_y=150;
      m.metadata.resolution=.02;m.metadata.origin.position.x=-1.5;m.metadata.origin.position.y=-1.5;m.metadata.origin.orientation.w=1.;m.data.resize(22500,0);
    });
    planner=rclcpp_action::create_server<Plan>(node,"compute_path_to_pose",
      [](const auto &,const auto &){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](const auto &){++cancels;return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const auto & g){actions.emplace_back([this,g]{++plans;std::this_thread::sleep_for(std::chrono::milliseconds(planning_ms));
        auto result=std::make_shared<Plan::Result>();result->path.header.frame_id="map";result->path.header.stamp=node->now();
        geometry_msgs::msg::PoseStamped start;start.header=result->path.header;start.pose.orientation.w=1.;result->path.poses={start,g->get_goal()->goal};
        if(g->is_canceling()){g->canceled(result);}else {g->succeed(result);}});});
    follower=rclcpp_action::create_server<Follow>(node,"follow_path",
      [](const auto &,const auto &){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](const auto &){++cancels;return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const auto & g){actions.emplace_back([this,g]{++follows;std::this_thread::sleep_for(std::chrono::milliseconds(following_ms));
        auto result=std::make_shared<Follow::Result>();if(g->is_canceling()){g->canceled(result);}else {g->succeed(result);}});});
    inputs=node->create_wall_timer(2ms,[this]{
      ++odom_count;auto now=node->now();nav_msgs::msg::Odometry o;o.header.frame_id="odom";o.child_frame_id="base_link";
      o.header.stamp=stale?now-rclcpp::Duration::from_seconds(1):now;
      if(moving){o.twist.twist.linear.x=-.0717;o.twist.twist.linear.y=.1007;o.twist.twist.angular.z=.0125;}odom->publish(o);
      if(odom_count%10) return;
      safe->publish(geometry_msgs::msg::Twist{});std_msgs::msg::UInt8 l;l.data=1;loc->publish(l);
      std_msgs::msg::Bool v;v.data=true;valid->publish(v);
      sensor_msgs::msg::LaserScan s;s.header.frame_id="base_link";s.header.stamp=now;s.range_min=.1;s.range_max=20;s.ranges.resize(360,10);scan->publish(s);
      geometry_msgs::msg::TransformStamped t;t.header.stamp=now;t.header.frame_id="map";t.child_frame_id="odom";t.transform.rotation.w=1.;
      auto u=t;u.header.frame_id="odom";u.child_frame_id="base_link";tf->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{t,u});
    });
    executor.add_node(node);spinning=std::thread([this]{executor.spin();});
    ASSERT_TRUE(service->wait_for_service(5s));std::this_thread::sleep_for(200ms);
  }
  void TearDown() override {
    // All action acceptance callbacks are finished before joining their workers.
    executor.cancel();spinning.join();for(auto & a:actions){if(a.joinable())a.join();}
    executor.remove_node(node);bt.reset();node.reset();
  }
  Service::Response::SharedPtr call(const std::string & op,uint64_t epoch=0) {
    auto req=std::make_shared<Service::Request>();req->run_id="native_test";req->operation=op;req->epoch=epoch;
    static int sequence=0;req->operation_id="native_test:"+std::to_string(++sequence);
    auto future=service->async_send_request(req);if(future.wait_for(3s)!=std::future_status::ready)throw std::runtime_error("service timeout");return future.get();
  }
  BT::Tree tree(BT::BehaviorTreeFactory & factory) {
    factory.registerFromPlugin(PLUGIN_PATH);
    factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_compute_path_to_pose_action_bt_node.so");
    factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_follow_path_action_bt_node.so");
    auto board=BT::Blackboard::create();board->set("node",bt);board->set("server_timeout",std::chrono::milliseconds(1000));
    board->set("bt_loop_duration",std::chrono::milliseconds(10));board->set("wait_for_service_timeout",std::chrono::milliseconds(3000));
    geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.orientation.w=1.;goal.pose.position.x=.5;board->set("goal",goal);
    std::string attrs=" run_id='native_test' footprint='[[-0.3,-0.2],[0.3,-0.2],[0.3,0.2],[-0.3,0.2]]' health_stable='0.1' timeout='5'";
    return factory.createTreeFromText("<root BTCPP_format='4'><BehaviorTree ID='Test'><Sequence><WaitForStoppedAndHealthy"+attrs+"/><ComputePathToPose goal='{goal}' path='{path}' planner_id='GridBased' use_start='false'/><ValidateStoppedPlanStart"+attrs+" path='{path}'/><FollowPath path='{path}' controller_id='FollowPath'/></Sequence></BehaviorTree></root>",board);
  }
  BT::NodeStatus run(BT::Tree & tree,int ms=6000) {
    auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);auto state=BT::NodeStatus::RUNNING;
    while(std::chrono::steady_clock::now()<until && state==BT::NodeStatus::RUNNING){state=tree.tickOnce();std::this_thread::sleep_for(10ms);}return state;
  }
};
TEST_F(Native, ActualRmwForwardsFreshControllerButFencesOldEpochAndLateClose) {
  auto acquired=call("ACQUIRE",call("INSPECT")->epoch);ASSERT_TRUE(acquired->accepted);
  geometry_msgs::msg::Twist c;c.linear.x=.37;for(int i=0;i<10;++i){raw->publish(c);std::this_thread::sleep_for(2ms);}
  auto released=call("RELEASE",acquired->epoch);ASSERT_TRUE(released->accepted);auto zero=released->last_recovery_zero_sequence;
  std::this_thread::sleep_for(150ms);
  {std::lock_guard<std::mutex> l(mutex);for(auto value:commands)EXPECT_NE(value,.37);commands.clear();}
  c.linear.x=.21;for(int i=0;i<20;++i){raw->publish(c);std::this_thread::sleep_for(10ms);}
  EXPECT_FALSE(call("CLOSE",acquired->epoch)->accepted);
  EXPECT_FALSE(call("ACQUIRE",acquired->epoch-1)->accepted);
  std::this_thread::sleep_for(100ms);auto after=call("INSPECT");EXPECT_EQ(after->last_recovery_zero_sequence,zero);
  std::lock_guard<std::mutex> l(mutex);ASSERT_FALSE(commands.empty());for(auto value:commands)EXPECT_DOUBLE_EQ(value,.21);
}
TEST_F(Native, InstalledBtLongPlanWithAdvancing500HzAndLateHalt) {
  BT::BehaviorTreeFactory factory;auto task=tree(factory);auto before=odom_count.load();
  EXPECT_EQ(run(task),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,1);EXPECT_EQ(follows,1);EXPECT_GT(odom_count-before,250);
  auto released=call("INSPECT");EXPECT_EQ(released->mode,"CONTROLLER");task.haltTree();std::this_thread::sleep_for(150ms);
  EXPECT_EQ(call("INSPECT")->last_recovery_zero_sequence,released->last_recovery_zero_sequence);
}
TEST_F(Native, MovingA12NeverPlansUntilMeasuredStopped) {
  moving=true;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  EXPECT_EQ(run(task,1300),BT::NodeStatus::RUNNING);EXPECT_EQ(plans,0);EXPECT_EQ(follows,0);
  moving=false;EXPECT_EQ(run(task),BT::NodeStatus::SUCCESS);EXPECT_EQ(follows,1);
}
TEST_F(Native, HaltDuringRealComputeCancelsOnlyThatActionAndKeepsZero) {
  planning_ms=800;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  for(int i=0;i<400 && !plans;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(plans,1);task.haltTree();std::this_thread::sleep_for(900ms);
  EXPECT_EQ(cancels,1);EXPECT_EQ(follows,0);EXPECT_EQ(call("INSPECT")->mode,"HOLD");
}
TEST_F(Native, StaleOdomDuringPlanPreventsFollowPath) {
  planning_ms=450;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  for(int i=0;i<400 && !plans;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(plans,1);stale=true;EXPECT_EQ(run(task,1000),BT::NodeStatus::RUNNING);EXPECT_EQ(follows,0);
  task.haltTree();EXPECT_EQ(call("INSPECT")->mode,"HOLD");
}
TEST_F(Native, HaltWhileStoppingNeverStartsEitherAction) {
  moving=true;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  EXPECT_EQ(run(task,500),BT::NodeStatus::RUNNING);task.haltTree();
  std::this_thread::sleep_for(150ms);EXPECT_EQ(plans,0);EXPECT_EQ(follows,0);
  EXPECT_EQ(call("INSPECT")->mode,"HOLD");
}
TEST_F(Native, RealFollowPathCancelAndLateCleanupCannotPublishRecoveryZero) {
  following_ms=800;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  for(int i=0;i<400 && !follows;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(follows,1);auto released=call("INSPECT");task.haltTree();
  std::this_thread::sleep_for(900ms);EXPECT_EQ(cancels,1);
  EXPECT_EQ(call("INSPECT")->last_recovery_zero_sequence,released->last_recovery_zero_sequence);
}
