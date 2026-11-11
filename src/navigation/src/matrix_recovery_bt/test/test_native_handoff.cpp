// Software-only ROS integration, isolated domain; installed BT and real action transport.
#include <gtest/gtest.h>
#include <atomic>
#include <array>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>
#include <limits>
#include <functional>
#include <new>
#include <csignal>
#include <cstdarg>
#include <cstdio>
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
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/costmap_2d_publisher.hpp>
#include <rcutils/logging.h>
#include "matrix_recovery_bt/srv/recovery_handoff.hpp"
#include "matrix_recovery_bt/recovery_errors.hpp"
using namespace std::chrono_literals;
using Service=matrix_recovery_bt::srv::RecoveryHandoff;
using Plan=nav2_msgs::action::ComputePathToPose;
using Follow=nav2_msgs::action::FollowPath;
namespace {
// Test executable only. Costmap2D allocates its byte array after the plugin
// unlocks Inputs. A one-shot allocation pause lets real TF DDS callbacks land
// deterministically during that conversion, without a product test hook.
std::atomic<bool> conversion_hook_armed{false};
std::function<void()> conversion_callback;
}
void * operator new[](std::size_t size) {
  void * value=std::malloc(size?size:1);if(!value){throw std::bad_alloc();}
  if(size==22500 && conversion_hook_armed.exchange(false)){conversion_callback();}
  return value;
}
void operator delete[](void * value) noexcept {std::free(value);}
void operator delete[](void * value,std::size_t) noexcept {std::free(value);}
namespace {
std::mutex diagnostic_mutex;
std::vector<std::string> diagnostic_lines;
rcutils_logging_output_handler_t previous_log_handler=nullptr;
void captureDiagnostic(const rcutils_log_location_t * location,int severity,const char * name,
  rcutils_time_point_value_t timestamp,const char * format,va_list * args) {
  va_list copy;va_copy(copy,*args);char text[16384];vsnprintf(text,sizeof(text),format,copy);va_end(copy);
  if(std::string(text).find("matrix_stop_wait_v1 ")==0){std::lock_guard<std::mutex> l(diagnostic_mutex);diagnostic_lines.emplace_back(text+20);}
  if(previous_log_handler){previous_log_handler(location,severity,name,timestamp,format,args);}
}
std::string diagnosticWith(const std::string & wanted) {
  std::lock_guard<std::mutex> l(diagnostic_mutex);
  for(auto it=diagnostic_lines.rbegin();it!=diagnostic_lines.rend();++it){if(it->find(wanted)!=std::string::npos){return *it;}}
  return {};
}
std::uint64_t diagnosticNumber(const std::string & line,const std::string & field) {
  const auto marker="\""+field+"\":";auto start=line.find(marker);
  if(start==std::string::npos){throw std::runtime_error("missing diagnostic field "+field);}
  return std::stoull(line.substr(start+marker.size()));
}
}
class Native : public testing::Test {
protected:
  static inline pid_t gate=-1;
  static void SetUpTestSuite() {
    ASSERT_NE(std::getenv("ROS_DOMAIN_ID"),nullptr);
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"),"190");
    gate=fork();ASSERT_GE(gate,0);
    if(gate==0) {execl(GATE_PATH,GATE_PATH,"--ros-args","-p","run_id:=native_test","-p","raw_topic:=/matrix/test/raw",nullptr);_exit(127);}
    int argc=0;rclcpp::init(argc,nullptr);
    previous_log_handler=rcutils_logging_get_output_handler();rcutils_logging_set_output_handler(captureDiagnostic);
  }
  static void TearDownTestSuite() {rcutils_logging_set_output_handler(previous_log_handler);rclcpp::shutdown();if(gate>0){kill(gate,SIGINT);waitpid(gate,nullptr,0);}}
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
  nav2_util::LifecycleNode::SharedPtr real_map_node;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> real_map;
  std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> real_map_publisher;
  rclcpp::TimerBase::SharedPtr real_map_timer;
  rclcpp::TimerBase::SharedPtr real_map_change_timer;
  rclcpp::TimerBase::SharedPtr stale_raw_timer;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr raw_map;
  rclcpp_action::Server<Plan>::SharedPtr planner;
  rclcpp_action::Server<Follow>::SharedPtr follower;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  rclcpp::TimerBase::SharedPtr inputs;
  std::vector<std::thread> actions;
  std::mutex mutex;
  std::vector<double> commands;
  std::atomic<int> plans{0},follows{0},cancels{0},odom_count{0};
  std::atomic<int> rpc_seen{0},auto_reply_from{1};
  std::atomic<std::int64_t> last_rpc_stamp{0};
  std::mutex rpc_mutex;
  struct DeferredMap {std::shared_ptr<rmw_request_id_t> header;std::int64_t requested;bool replied=false;};
  std::vector<DeferredMap> deferred_maps;
  std::atomic<bool> moving{false},stale{false};
  std::atomic<bool> actual_contract{false},stale_tf{false};
  std::atomic<bool> actual_body_jitter{false};
  std::atomic<int> body_jitter_samples{0},actual_map_changes{0};
  std::atomic<double> map_shift{0.};
  bool drift_after_first_plan=false,invalid_path_start=false,block_body_after_plan=false;
  bool planner_fails=false;
  bool mark_map_after_plan=false;
  double path_start_offset=0.;
  int planning_ms=450,following_ms=40;
  void SetUp() override {
    {std::lock_guard<std::mutex> l(diagnostic_mutex);diagnostic_lines.clear();}
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
      ++rpc_seen;r->map=fixtureMap(block_body_after_plan && plans>0 ? 254 : 0);
    });
    planner=rclcpp_action::create_server<Plan>(node,"compute_path_to_pose",
      [](const auto &,const auto &){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](const auto &){++cancels;return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const auto & g){actions.emplace_back([this,g]{
        const int plan_number=++plans;const double planned_shift=map_shift.load();
        std::this_thread::sleep_for(std::chrono::milliseconds(planning_ms));
        auto result=std::make_shared<Plan::Result>();result->path.header.frame_id="map";result->path.header.stamp=node->now();
        geometry_msgs::msg::PoseStamped start;start.header=result->path.header;start.pose.orientation.w=1.;
        if(actual_contract){start=bodyInMap();start.header=result->path.header;}
        start.pose.position.x+=planned_shift+(invalid_path_start?.1:path_start_offset);
        result->path.poses={start,g->get_goal()->goal};
        if(drift_after_first_plan && plan_number==1){map_shift=.06;std::this_thread::sleep_for(150ms);}
        if(g->is_canceling()){g->canceled(result);}else if(planner_fails){result->error_code=208;g->abort(result);}else {g->succeed(result);}});});
    follower=rclcpp_action::create_server<Follow>(node,"follow_path",
      [](const auto &,const auto &){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](const auto &){++cancels;return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const auto & g){actions.emplace_back([this,g]{++follows;std::this_thread::sleep_for(std::chrono::milliseconds(following_ms));
        auto result=std::make_shared<Follow::Result>();if(g->is_canceling()){g->canceled(result);}else {g->succeed(result);}});});
    inputs=node->create_wall_timer(2ms,[this]{
      ++odom_count;auto now=node->now();nav_msgs::msg::Odometry o;o.header.frame_id="odom";o.child_frame_id="base_link";
      o.header.stamp=stale?now-rclcpp::Duration::from_seconds(1):now;
      if(actual_contract){const auto body=bodyCoordinates();o.pose.pose.position.x=body[0];o.pose.pose.position.y=body[1];
        o.pose.pose.orientation.z=std::sin(body[2]/2);o.pose.pose.orientation.w=std::cos(body[2]/2);
        if(actual_body_jitter){++body_jitter_samples;}}
      if(moving){o.twist.twist.linear.x=-.0717;o.twist.twist.linear.y=.1007;o.twist.twist.angular.z=.0125;}odom->publish(o);
      if(actual_contract){publishTf(now);if(odom_count%25==0){safe->publish(geometry_msgs::msg::Twist{});}}
      if(odom_count%(actual_contract?50:10)) return;
      if(!actual_contract){safe->publish(geometry_msgs::msg::Twist{});}std_msgs::msg::UInt8 l;l.data=1;loc->publish(l);
      std_msgs::msg::Bool v;v.data=true;valid->publish(v);
      sensor_msgs::msg::LaserScan s;s.header.frame_id="base_link";s.header.stamp=now;s.range_min=.1;s.range_max=20;s.ranges.resize(360,10);scan->publish(s);
      if(!actual_contract){publishTf(now);}
    });
    executor.add_node(node);spinning=std::thread([this]{executor.spin();});
    ASSERT_TRUE(service->wait_for_service(5s));std::this_thread::sleep_for(200ms);
  }
  void TearDown() override {
    // All action acceptance callbacks are finished before joining their workers.
    executor.cancel();spinning.join();for(auto & a:actions){if(a.joinable())a.join();}
    if(real_map_node){executor.remove_node(real_map_node->get_node_base_interface());real_map_change_timer.reset();real_map_timer.reset();real_map_publisher.reset();real_map.reset();real_map_node.reset();}
    executor.remove_node(node);bt.reset();node.reset();
  }
  std::array<double,3> bodyCoordinates() const {
    const double tick=odom_count.load();
    return {.16+(actual_body_jitter?.0015*std::sin(tick*.13):0.),
      .01+(actual_body_jitter?.0015*std::cos(tick*.17):0.),
      .07+(actual_body_jitter?.00015*std::sin(tick*.07):0.)};
  }
  geometry_msgs::msg::PoseStamped bodyInMap() {
    const auto body=bodyCoordinates();
    geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";
    p.pose.position.x=-8.218+std::cos(-2.278)*body[0]-std::sin(-2.278)*body[1];
    p.pose.position.y=11.516+std::sin(-2.278)*body[0]+std::cos(-2.278)*body[1];
    p.pose.orientation.z=std::sin((-2.278+body[2])/2);p.pose.orientation.w=std::cos((-2.278+body[2])/2);return p;
  }
  void publishTf(const rclcpp::Time & now) {
    geometry_msgs::msg::TransformStamped t;t.header.stamp=stale_tf?now-rclcpp::Duration::from_seconds(1):now;
    t.header.frame_id="map";t.child_frame_id="odom";t.transform.rotation.w=1.;
    auto u=t;u.header.frame_id="odom";u.child_frame_id="base_link";
    if(actual_contract){t.transform.translation.x=-8.218;t.transform.translation.y=11.516;
      t.transform.rotation.z=std::sin(-2.278/2);t.transform.rotation.w=std::cos(-2.278/2);
      const auto body=bodyCoordinates();u.transform.translation.x=body[0];u.transform.translation.y=body[1];
      u.transform.rotation.z=std::sin(body[2]/2);u.transform.rotation.w=std::cos(body[2]/2);}
    t.transform.translation.x+=map_shift.load();
    tf->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{t,u});
  }
  void actualCostmap(unsigned char initial=255) {
    // The service implementation is the installed Jazzy Costmap2DPublisher,
    // not a hand-built response. This still does not simulate the layer pipeline.
    executor.cancel();spinning.join();map.reset();actual_contract=true;
    real_map_node=std::make_shared<nav2_util::LifecycleNode>("costmap","/local_costmap");
    real_map=std::make_unique<nav2_costmap_2d::Costmap2D>(600,600,.01,-2.84,-2.99,initial);
    real_map_publisher=std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(real_map_node,real_map.get(),"odom","costmap",true);
    real_map_publisher->on_activate();real_map_timer=real_map_node->create_wall_timer(1s,[this]{real_map_publisher->publishCostmap();});
    executor.add_node(real_map_node->get_node_base_interface());spinning=std::thread([this]{executor.spin();});
    auto client=node->create_client<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap");
    ASSERT_TRUE(client->wait_for_service(3s));auto f=client->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>());
    ASSERT_EQ(f.wait_for(3s),std::future_status::ready);auto response=f.get();
    ASSERT_EQ(response->map.header.frame_id,"odom");ASSERT_GT(rclcpp::Time(response->map.header.stamp).nanoseconds(),0);
    ASSERT_DOUBLE_EQ(response->map.metadata.origin.orientation.w,1.);ASSERT_EQ(response->map.data.size(),360000U);
  }
  void fillActualMap(unsigned char value) {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> l(*real_map->getMutex());
    std::fill(real_map->getCharMap(),real_map->getCharMap()+360000,value);
  }
  void changeActualMapAtTenHz() {
    real_map_change_timer=real_map_node->create_wall_timer(100ms,[this]{
      {std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> l(*real_map->getMutex());
        // Real data generation changes, outside both padded body and connector.
        real_map->setCost(0,0,(++actual_map_changes%2)?254:255);}
      real_map_publisher->publishCostmap();
    });
  }
  void paddedNearCell(unsigned char value) {
    const auto body=bodyCoordinates();unsigned int x=0,y=0;
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> l(*real_map->getMutex());
    // 0.75 m is outside the unpadded 0.647 m nose, inside its 0.797 m padded nose.
    ASSERT_TRUE(real_map->worldToMap(body[0]+.75*std::cos(body[2]),body[1]+.75*std::sin(body[2]),x,y));
    real_map->setCost(x,y,value);
  }
  nav2_msgs::msg::Costmap fixtureMap(unsigned char value=0,std::int64_t stamp=0) {
    nav2_msgs::msg::Costmap m;m.header.frame_id="odom";m.header.stamp=stamp?rclcpp::Time(stamp):node->now();
    last_rpc_stamp=rclcpp::Time(m.header.stamp).nanoseconds();m.metadata.size_x=m.metadata.size_y=150;
    m.metadata.resolution=.02;m.metadata.origin.position.x=m.metadata.origin.position.y=-1.5;
    m.metadata.origin.orientation.w=1.;m.data.resize(22500,value);
    if(mark_map_after_plan && plans>0 && value==0){m.data.front()=1;}
    return m;
  }
  void deferredCostmap(int first_automatic=std::numeric_limits<int>::max()) {
    executor.cancel();spinning.join();map.reset();rpc_seen=0;auto_reply_from=first_automatic;
    map=node->create_service<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap",
      [this](std::shared_ptr<rmw_request_id_t> header,nav2_msgs::srv::GetCostmap::Request::SharedPtr) {
        const int number=++rpc_seen;
        {std::lock_guard<std::mutex> lock(rpc_mutex);deferred_maps.push_back({header,
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(),false});}
        if(number>=auto_reply_from.load()){replyMap(number);}
      });
    spinning=std::thread([this]{executor.spin();});
  }
  void replyMap(std::size_t number,unsigned char value=0,std::int64_t stamp=0) {
    std::shared_ptr<rmw_request_id_t> header;
    {std::lock_guard<std::mutex> lock(rpc_mutex);
      if(!number || number>deferred_maps.size() || deferred_maps[number-1].replied){throw std::runtime_error("invalid deferred request");}
      auto & pending=deferred_maps[number-1];pending.replied=true;header=pending.header;}
    nav2_msgs::srv::GetCostmap::Response response;response.map=fixtureMap(value,stamp);map->send_response(*header,response);
  }
  void publishOldRawAgainstRpc() {
    raw_map=node->create_publisher<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw",rclcpp::QoS(1).reliable().transient_local());
    stale_raw_timer=node->create_wall_timer(10ms,[this]{
      const auto stamp=last_rpc_stamp.load();if(!stamp){return;}
      // Alternate strictly older raw and equal-header raw with blocked data.
      static bool equal=false;equal=!equal;auto m=fixtureMap(254,stamp-(equal?0:50000000));
      last_rpc_stamp=stamp;raw_map->publish(m);
    });
  }
  Service::Response::SharedPtr call(const std::string & op,uint64_t epoch=0) {
    auto req=std::make_shared<Service::Request>();req->run_id="native_test";req->operation=op;req->epoch=epoch;
    static int sequence=0;req->operation_id="native_test:"+std::to_string(++sequence);
    auto future=service->async_send_request(req);if(future.wait_for(3s)!=std::future_status::ready)throw std::runtime_error("service timeout");return future.get();
  }
  BT::Tree tree(BT::BehaviorTreeFactory & factory,bool recovery=false,double timeout=5.,bool scope_exception=false) {
    factory.registerFromPlugin(PLUGIN_PATH);
    factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_compute_path_to_pose_action_bt_node.so");
    factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_follow_path_action_bt_node.so");
    if(recovery){factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_recovery_node_bt_node.so");}
    if(scope_exception){factory.registerSimpleAction("FixtureScopeException",[](BT::TreeNode &)->BT::NodeStatus {
      throw std::runtime_error("fixture exception after real measured-stop Wait");});}
    auto board=BT::Blackboard::create();board->set("node",bt);board->set("server_timeout",std::chrono::milliseconds(1000));
    board->set("bt_loop_duration",std::chrono::milliseconds(10));board->set("wait_for_service_timeout",std::chrono::milliseconds(3000));
    geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.orientation.w=1.;goal.pose.position.x=.5;
    if(actual_contract){goal=bodyInMap();goal.pose.position.x+=.5;}board->set("goal",goal);
    std::string attrs=" run_id='native_test' footprint='[[-0.3,-0.2],[0.3,-0.2],[0.3,0.2],[-0.3,0.2]]' health_stable='0.1' timeout='"+std::to_string(timeout)+"'";
    // Omit both timing ports: exercise the production defaults of 1s and 30s.
    if(actual_contract){attrs=" run_id='native_test' footprint='[[-0.647,-0.53],[0.647,-0.53],[0.647,0.53],[-0.647,0.53]]' padding='0.15'";}
    attrs+=" error_code_id='{recovery_error_code}'";
    auto sequence="<Sequence><StoppedReplanScope><Sequence><BeginStoppedReplanAttempt controller_error_code='{follow_path_error_code}' planner_error_code='{compute_path_error_code}' recovery_error_code='{recovery_error_code}'/>"
      "<WaitForStoppedAndHealthy"+attrs+"/>"+(scope_exception?std::string("<FixtureScopeException/>"):
      std::string("<ComputePathToPose goal='{goal}' path='{path}' planner_id='GridBased' use_start='false' error_code_id='{compute_path_error_code}'/>"))+
      "<ValidateStoppedPlanStart"+attrs+" path='{path}'/></Sequence></StoppedReplanScope><FollowPath path='{path}' controller_id='FollowPath' error_code_id='{follow_path_error_code}'/></Sequence>";
    if(recovery){sequence="<RecoveryNode number_of_retries='1'>"+sequence+
      "<WouldAStoppedPlanRecoveryHelp error_code='{recovery_error_code}'/></RecoveryNode>";}
    return factory.createTreeFromText("<root BTCPP_format='4'><BehaviorTree ID='Test'>"+sequence+"</BehaviorTree></root>",board);
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
TEST_F(Native, FreshPlanStartDriftFailsTypedBeforeReleaseWithoutThirtySecondWait) {
  drift_after_first_plan=true;planning_ms=100;
  BT::BehaviorTreeFactory factory;auto task=tree(factory);
  const auto started=std::chrono::steady_clock::now();
  EXPECT_EQ(run(task),BT::NodeStatus::FAILURE);
  EXPECT_LT(std::chrono::steady_clock::now()-started,3s);
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::PlanStartInvalidated);
  EXPECT_EQ(plans,1);EXPECT_EQ(follows,0);EXPECT_EQ(call("INSPECT")->mode,"HOLD");
  const auto diagnostic=diagnosticWith("plan_start_invalidated");
  EXPECT_NE(diagnostic.find("\"planned_start\":{\"frame\":\"map\""),std::string::npos);
  // The current certificate retains the measured body in the Costmap frame.
  // A map/odom pair must not be presented as a same-frame numeric delta.
  EXPECT_NE(diagnostic.find("\"last_valid_body\":{\"frame\":\"odom\""),std::string::npos);
  EXPECT_NE(diagnostic.find("\"plan_invalidated\":true"),std::string::npos);
  RecordProperty("frame_aware_drift_diagnostic",diagnostic);
}
TEST_F(Native, OneNativeRetryRecapturesActualStartAndClearsCurrentErrorOnly) {
  drift_after_first_plan=true;planning_ms=100;
  BT::BehaviorTreeFactory factory;auto task=tree(factory,true);
  EXPECT_EQ(run(task),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(plans,2);EXPECT_EQ(follows,1);
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),0);
  const auto history=task.rootBlackboard()->get<std::vector<std::array<int,3>>>("matrix_recovery_cause_history");
  ASSERT_EQ(history.size(),1U);EXPECT_EQ(history[0][2],matrix_recovery_bt::PlanStartInvalidated);
}
TEST_F(Native, FreshPathStartMismatchInvalidatesEvenWhenNewCostmapBlocksBody) {
  invalid_path_start=true;block_body_after_plan=true;planning_ms=450;
  BT::BehaviorTreeFactory factory;auto task=tree(factory);
  const auto started=std::chrono::steady_clock::now();
  EXPECT_EQ(run(task),BT::NodeStatus::FAILURE);
  EXPECT_LT(std::chrono::steady_clock::now()-started,3s);
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::PlanStartInvalidated);
  EXPECT_EQ(plans,1);EXPECT_EQ(follows,0);EXPECT_EQ(call("INSPECT")->mode,"HOLD");
}
TEST_F(Native, RepeatedInvalidPathStartExhaustsExactlyOneNativeRetry) {
  invalid_path_start=true;planning_ms=50;
  BT::BehaviorTreeFactory factory;auto task=tree(factory,true);
  EXPECT_EQ(run(task),BT::NodeStatus::FAILURE);
  EXPECT_EQ(plans,2);EXPECT_EQ(follows,0);EXPECT_EQ(call("INSPECT")->mode,"HOLD");
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::PlanStartInvalidated);
}
TEST_F(Native, StaleInputsHaveFiniteTypedHealthWaitWithoutGeometricRetry) {
  stale=true;BT::BehaviorTreeFactory factory;auto task=tree(factory,true,.8);
  EXPECT_EQ(run(task,2000),BT::NodeStatus::FAILURE);
  EXPECT_EQ(plans,0);EXPECT_EQ(follows,0);
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::InputHealthTimeout);
  EXPECT_EQ(call("INSPECT")->mode,"HOLD");
}
TEST_F(Native, HaltWithUnacknowledgedReleaseRequiresReconcileEvenAfterLateReply) {
  planning_ms=800;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  for(int i=0;i<400 && !plans;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(plans,1);
  struct ResumeOwnedGate {pid_t pid;~ResumeOwnedGate(){kill(pid,SIGCONT);}} resume{gate};
  ASSERT_EQ(kill(gate,SIGSTOP),0);
  EXPECT_EQ(run(task,1200),BT::NodeStatus::RUNNING);
  task.haltTree();
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::OwnershipUnknown);
  kill(gate,SIGCONT);std::this_thread::sleep_for(150ms);
  EXPECT_EQ(run(task,500),BT::NodeStatus::FAILURE);
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::OwnershipUnknown);
  EXPECT_EQ(follows,0);EXPECT_EQ(plans,1);
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
TEST_F(Native, ActualCostmapUnknownBodyWaitsThenFreshMapAndDefaultStablePermitNormalEntry) {
  actualCostmap();BT::BehaviorTreeFactory factory;auto task=tree(factory);
  EXPECT_EQ(run(task,1500),BT::NodeStatus::RUNNING);EXPECT_EQ(plans,0);EXPECT_EQ(follows,0);
  auto unknown=diagnosticWith("\"body_collision\":");ASSERT_FALSE(unknown.empty());
  EXPECT_NE(unknown.find("\"map_unknown_cells\":360000"),std::string::npos);
  EXPECT_NE(unknown.find("\"full_body_free\":false"),std::string::npos);RecordProperty("unknown_wait_diagnostic",unknown);
  const auto freed=std::chrono::steady_clock::now();fillActualMap(0);
  EXPECT_EQ(run(task,7000),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,1);EXPECT_EQ(follows,1);
  EXPECT_GE(std::chrono::steady_clock::now()-freed,1s);EXPECT_EQ(call("INSPECT")->mode,"CONTROLLER");
}
TEST_F(Native, ActualCostmapUnknownBodyTimesOutInStopPhaseAtDefaultThirtySeconds) {
  actualCostmap();BT::BehaviorTreeFactory factory;auto task=tree(factory);const auto started=std::chrono::steady_clock::now();
  EXPECT_EQ(run(task,32000),BT::NodeStatus::FAILURE);
  EXPECT_GE(std::chrono::steady_clock::now()-started,30s);EXPECT_EQ(plans,0);EXPECT_EQ(follows,0);
  EXPECT_EQ(call("INSPECT")->mode,"HOLD");auto timeout=diagnosticWith("\"event\":\"timeout\"");ASSERT_FALSE(timeout.empty());
  for(auto field:{"\"phase\":\"WAIT_STOP_AND_HEALTH\"","\"timeout_sec\":30","\"health_stable_required_sec\":1",
      "\"map_unknown_cells\":360000","\"body_aabb_unknown_cells\":","\"advancing_and_bounded\":true",
      "\"follow_active\":false","\"full_body_free\":false","\"rpc_responses\":","\"safe_command\":"}) {
    EXPECT_NE(timeout.find(field),std::string::npos)<<field;
  }
  RecordProperty("default_timeout_diagnostic",timeout);
  std::lock_guard<std::mutex> l(diagnostic_mutex);EXPECT_LE(diagnostic_lines.size(),32U);
}
TEST_F(Native, ActualCostmapLethalBodyRemainsBlockedUntilMapChanges) {
  actualCostmap(254);BT::BehaviorTreeFactory factory;auto task=tree(factory);
  EXPECT_EQ(run(task,1500),BT::NodeStatus::RUNNING);EXPECT_EQ(plans,0);
  auto blocked=diagnosticWith("\"map_lethal_cells\":360000");ASSERT_FALSE(blocked.empty());RecordProperty("lethal_wait_diagnostic",blocked);
  fillActualMap(0);EXPECT_EQ(run(task,7000),BT::NodeStatus::SUCCESS);EXPECT_EQ(follows,1);
}
TEST_F(Native, ActualCostmapStaleTfReportsExceptionAndFreshTfRestoresNormalEntry) {
  actualCostmap(0);stale_tf=true;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  EXPECT_EQ(run(task,1500),BT::NodeStatus::RUNNING);EXPECT_EQ(plans,0);EXPECT_EQ(follows,0);
  auto stale_snapshot=diagnosticWith("stale body TF");ASSERT_FALSE(stale_snapshot.empty());RecordProperty("stale_tf_diagnostic",stale_snapshot);
  stale_tf=false;EXPECT_EQ(run(task,7000),BT::NodeStatus::SUCCESS);EXPECT_EQ(follows,1);
}
TEST_F(Native, MapRpcMissingFirstResponseHasFiniteRequestAndRecoversInSameTree) {
  deferredCostmap(2);BT::BehaviorTreeFactory factory;auto task=tree(factory);
  EXPECT_EQ(run(task),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,1);EXPECT_EQ(follows,1);
  ASSERT_GE(rpc_seen.load(),2);
  {std::lock_guard<std::mutex> lock(rpc_mutex);ASSERT_GE(deferred_maps.size(),2U);
    const auto elapsed=deferred_maps[1].requested-deferred_maps[0].requested;
    // These are server receipt times. The difference contains two independent
    // DDS transport delays; the product's exact 500ms deadline is separately
    // established by its timeout counter and the new request generation.
    EXPECT_GE(elapsed,480000000LL);EXPECT_LT(elapsed,950000000LL);RecordProperty("server_receipt_retry_interval_ns",elapsed);}
  const auto report=diagnosticWith("WAIT_RELEASE_RESPONSE");ASSERT_FALSE(report.empty());
  EXPECT_GE(diagnosticNumber(report,"rpc_timeouts"),1U);RecordProperty("timeout_recovery_diagnostic",report);
}
TEST_F(Native, MapRpcLateOldResponseCannotClearNewPendingOrInstallOldGeometry) {
  deferredCostmap();BT::BehaviorTreeFactory factory;auto task=tree(factory);
  for(int i=0;i<150 && rpc_seen<2;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(rpc_seen,2);replyMap(1,254);const auto second_seen=rpc_seen.load();
  EXPECT_EQ(run(task,150),BT::NodeStatus::RUNNING);EXPECT_EQ(rpc_seen,second_seen);EXPECT_EQ(follows,0);
  auto_reply_from=3;
  EXPECT_EQ(run(task),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,1);EXPECT_EQ(follows,1);
  const auto report=diagnosticWith("WAIT_RELEASE_RESPONSE");ASSERT_FALSE(report.empty());
  EXPECT_GE(diagnosticNumber(report,"rpc_timeouts"),2U);RecordProperty("late_reply_diagnostic",report);
}
TEST_F(Native, MapRpcNewerAndEqualRpcRejectOutOfOrderBlockedRaw) {
  publishOldRawAgainstRpc();BT::BehaviorTreeFactory factory;auto task=tree(factory);
  EXPECT_EQ(run(task),BT::NodeStatus::SUCCESS);EXPECT_EQ(follows,1);
  const auto report=diagnosticWith("WAIT_RELEASE_RESPONSE");ASSERT_FALSE(report.empty());
  EXPECT_NE(report.find("\"source\":\"GetCostmap\""),std::string::npos);
  EXPECT_GT(diagnosticNumber(report,"rejected_samples"),0U);RecordProperty("map_selection_diagnostic",report);
}
TEST_F(Native, MapRpcPollsThroughPlanningThenStopsThroughoutFollowPathAndAfterHalt) {
  planning_ms=1000;following_ms=1000;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  for(int i=0;i<400 && !plans;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(plans,1);const auto during_plan=rpc_seen.load();
  EXPECT_EQ(run(task,300),BT::NodeStatus::RUNNING);EXPECT_GE(rpc_seen-during_plan,2);
  for(int i=0;i<300 && !follows;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(follows,1);std::this_thread::sleep_for(120ms);const auto released=rpc_seen.load();
  EXPECT_EQ(run(task,300),BT::NodeStatus::RUNNING);EXPECT_EQ(rpc_seen,released);
  task.haltTree();std::this_thread::sleep_for(250ms);EXPECT_EQ(rpc_seen,released);
}
TEST_F(Native, MapRpcScopeClosesOnRealPlannerFailureAndOnPlannerHalt) {
  planner_fails=true;planning_ms=100;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  EXPECT_EQ(run(task),BT::NodeStatus::FAILURE);EXPECT_EQ(follows,0);
  std::this_thread::sleep_for(120ms);auto stopped=rpc_seen.load();std::this_thread::sleep_for(250ms);EXPECT_EQ(rpc_seen,stopped);
  planner_fails=false;planning_ms=1000;task.haltTree();
  for(int i=0;i<400 && plans<2;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(plans,2);task.haltTree();std::this_thread::sleep_for(120ms);stopped=rpc_seen.load();
  std::this_thread::sleep_for(250ms);EXPECT_EQ(rpc_seen,stopped);EXPECT_EQ(follows,0);
}
TEST_F(Native, MapRpcHaltRetiresPendingAndReentryHasFreshRequestPhase) {
  deferredCostmap();BT::BehaviorTreeFactory factory;auto task=tree(factory);
  for(int i=0;i<80 && !rpc_seen;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(rpc_seen,1);task.haltTree();std::this_thread::sleep_for(650ms);EXPECT_EQ(rpc_seen,1);
  auto_reply_from=2;
  EXPECT_EQ(run(task),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,1);EXPECT_EQ(follows,1);
  replyMap(1,254);std::this_thread::sleep_for(150ms);EXPECT_EQ(call("INSPECT")->mode,"CONTROLLER");
}
TEST_F(Native, MapRpcBlockedValidateUsesSharedPhaseDeadlineAndTypedCollisionCause) {
  block_body_after_plan=true;planning_ms=200;BT::BehaviorTreeFactory factory;auto task=tree(factory,false,1.8);
  const auto started=std::chrono::steady_clock::now();
  EXPECT_EQ(run(task,3500),BT::NodeStatus::FAILURE);EXPECT_EQ(plans,1);EXPECT_EQ(follows,0);
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::CollisionWaitTimeout);
  EXPECT_GE(std::chrono::steady_clock::now()-started,1700ms);EXPECT_LT(std::chrono::steady_clock::now()-started,3200ms);
  const auto report=diagnosticWith("\"phase\":\"WAIT_PLAN_START_CERTIFICATE\"");ASSERT_FALSE(report.empty());
  RecordProperty("validate_collision_diagnostic",report);
}
TEST_F(Native, MapRpcHighFrequencyConstantPoseSucceedsAndMicroJitterTerminatesWithinBudget) {
  actualCostmap(0);planning_ms=150;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  const auto before=odom_count.load();EXPECT_EQ(run(task,7000),BT::NodeStatus::SUCCESS);EXPECT_GT(odom_count-before,250);EXPECT_EQ(follows,1);
  task.haltTree();std::atomic<bool> done{false};
  std::thread jitter([this,&done]{int i=0;while(!done){map_shift=(++i%2)?.0001:0.;std::this_thread::sleep_for(2ms);}});
  const auto begin=std::chrono::steady_clock::now();auto state=run(task,32000);done=true;jitter.join();
  EXPECT_NE(state,BT::NodeStatus::RUNNING);EXPECT_LT(std::chrono::steady_clock::now()-begin,32s);
  RecordProperty("micro_jitter_terminal",state==BT::NodeStatus::SUCCESS?"SUCCESS":"FINITE_FAILURE");
}
TEST_F(Native, ActualBodyMicroJitterAndTenHzMapChurnCompletePlanningAndRelease) {
  actualCostmap(0);actual_body_jitter=true;changeActualMapAtTenHz();planning_ms=200;
  BT::BehaviorTreeFactory factory;auto task=tree(factory);
  const auto begin=std::chrono::steady_clock::now();
  EXPECT_EQ(run(task,10000),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(plans,1);EXPECT_EQ(follows,1);EXPECT_GT(body_jitter_samples,500);EXPECT_GT(actual_map_changes,5);
  EXPECT_LT(std::chrono::steady_clock::now()-begin,10s);
  const auto released=call("INSPECT");EXPECT_EQ(released->mode,"CONTROLLER");
  const auto report=diagnosticWith("\"certificate_checks\":");ASSERT_FALSE(report.empty());
  EXPECT_GT(diagnosticNumber(report,"certificate_checks"),0U);
  EXPECT_LT(diagnosticNumber(report,"certificate_commit_max_ns"),300000000U);
  RecordProperty("actual_body_micro_jitter_and_map_churn_diagnostic",report);
  RecordProperty("actual_body_jitter_sample_count",body_jitter_samples.load());
  RecordProperty("actual_ten_hz_map_geometry_updates",actual_map_changes.load());
}
TEST_F(Native, ActualPaddedNearLethalAndUnknownCannotPlanOrReleaseUntilCleared) {
  actualCostmap(0);actual_body_jitter=true;
  for(const unsigned char value:{static_cast<unsigned char>(254),static_cast<unsigned char>(255)}) {
    fillActualMap(0);paddedNearCell(value);BT::BehaviorTreeFactory factory;auto task=tree(factory);
    EXPECT_EQ(run(task,1800),BT::NodeStatus::RUNNING);EXPECT_EQ(plans,0);EXPECT_EQ(follows,0);
    EXPECT_EQ(call("INSPECT")->mode,"HOLD");task.haltTree();
  }
  fillActualMap(0);BT::BehaviorTreeFactory factory;auto task=tree(factory);
  EXPECT_EQ(run(task,10000),BT::NodeStatus::SUCCESS);EXPECT_EQ(plans,1);EXPECT_EQ(follows,1);
  EXPECT_EQ(call("INSPECT")->mode,"CONTROLLER");
}
TEST_F(Native, PendingReleaseMapRetryCannotHideDeliveredNonzeroOdomAndRecoveredStopSegment) {
  raw_map=node->create_publisher<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw",rclcpp::QoS(1).reliable().transient_local());
  planning_ms=200;BT::BehaviorTreeFactory factory;auto task=tree(factory);
  for(int i=0;i<400 && !plans;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(plans,1);
  struct ResumeOwnedGate {pid_t pid;~ResumeOwnedGate(){kill(pid,SIGCONT);}} resume{gate};
  ASSERT_EQ(kill(gate,SIGSTOP),0);
  for(int i=0;i<200 && diagnosticWith("\"phase\":\"WAIT_RELEASE_RESPONSE\"").empty();++i){
    task.tickOnce();std::this_thread::sleep_for(10ms);
  }
  const auto before=diagnosticWith("\"phase\":\"WAIT_RELEASE_RESPONSE\"");ASSERT_FALSE(before.empty());
  std::atomic<int> triggered{0};
  struct ResetHook {~ResetHook(){conversion_hook_armed=false;conversion_callback={};}} reset;
  conversion_callback=[this,&triggered]{
    ++triggered;auto changed=fixtureMap();changed.data.back()=252;raw_map->publish(changed);
    moving=true;std::this_thread::sleep_for(40ms);moving=false;
    // Let the real subscription consume the bad sample and re-qualify a new
    // segment before the plugin can see the queued old RELEASE response.
    std::this_thread::sleep_for(400ms);
  };
  conversion_hook_armed=true;auto changed=fixtureMap();changed.data.back()=253;raw_map->publish(changed);
  EXPECT_EQ(run(task,2000),BT::NodeStatus::FAILURE);EXPECT_EQ(triggered,1);
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::OwnershipUnknown);
  EXPECT_EQ(follows,0);EXPECT_EQ(plans,1);
  const auto after=diagnosticWith("stop_continuity_lost_during_certificate");ASSERT_FALSE(after.empty());
  EXPECT_GT(diagnosticNumber(after,"first_steady_ns"),diagnosticNumber(before,"first_steady_ns"));
  EXPECT_NE(after.find("\"qualified\":true"),std::string::npos);
  RecordProperty("pending_release_recovered_segment_diagnostic",after);
  kill(gate,SIGCONT);std::this_thread::sleep_for(150ms);
  EXPECT_EQ(run(task,500),BT::NodeStatus::FAILURE);EXPECT_EQ(follows,0);
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::OwnershipUnknown);
}
TEST_F(Native, MapRpcConnectorTransformChangesDuringUnlockedConversionCannotRelease) {
  mark_map_after_plan=true;path_start_offset=.04;planning_ms=500;
  BT::BehaviorTreeFactory factory;auto task=tree(factory);std::atomic<int> triggered{0};
  struct ResetHook {~ResetHook(){conversion_hook_armed=false;conversion_callback={};}} reset;
  for(int i=0;i<400 && !plans;++i){task.tickOnce();std::this_thread::sleep_for(10ms);}
  ASSERT_EQ(plans,1);
  conversion_callback=[this,&triggered]{
    ++triggered;map_shift=-.02;publishTf(node->now());std::this_thread::sleep_for(60ms);
  };
  conversion_hook_armed=true;
  EXPECT_EQ(run(task),BT::NodeStatus::FAILURE);EXPECT_EQ(triggered,1);
  EXPECT_EQ(task.rootBlackboard()->get<int>("recovery_error_code"),matrix_recovery_bt::PlanStartInvalidated);
  EXPECT_EQ(plans,1);EXPECT_EQ(follows,0);EXPECT_EQ(call("INSPECT")->mode,"HOLD");
  const auto report=diagnosticWith("plan_start_invalidated");ASSERT_FALSE(report.empty());
  RecordProperty("connector_race_diagnostic",report);
}
TEST_F(Native, MapRpcScopeExceptionClosesPollingAndPreservesConfirmedGateHold) {
  BT::BehaviorTreeFactory factory;auto task=tree(factory,false,5.,true);
  EXPECT_THROW(run(task),BT::NodeExecutionError);EXPECT_EQ(plans,0);EXPECT_EQ(follows,0);
  std::this_thread::sleep_for(120ms);const auto stopped=rpc_seen.load();
  std::this_thread::sleep_for(250ms);EXPECT_EQ(rpc_seen,stopped);EXPECT_EQ(call("INSPECT")->mode,"HOLD");
}
