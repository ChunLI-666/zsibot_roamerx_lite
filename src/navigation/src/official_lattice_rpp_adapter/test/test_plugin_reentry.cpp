// SPDX-License-Identifier: Apache-2.0
// Exercise the real plugin DSO through its public Jazzy interfaces. The input
// poses are synthetic, not a replay of B30's unrecorded controller requests.
#include <gtest/gtest.h>
#include <class_loader/class_loader.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <thread>

using namespace std::chrono_literals;
namespace {
geometry_msgs::msg::PoseStamped pose(double x, double heading=M_PI) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id="map";p.pose.position.x=x;
  p.pose.orientation.z=std::sin(heading/2.);p.pose.orientation.w=std::cos(heading/2.);
  return p;
}
class PluginReentry : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const char * domain=std::getenv("ROS_DOMAIN_ID");
    if (!domain||std::string(domain)!="190") {
      throw std::runtime_error("Synthetic plugin tests require isolated ROS_DOMAIN_ID=190");
    }
    rclcpp::init(0,nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    const char * override_library=std::getenv("RFN_REENTRY_TEST_LIBRARY");
    loader_=std::make_unique<class_loader::ClassLoader>(
      override_library?override_library:RFN_REENTRY_ADAPTER_LIBRARY);
    node_=std::make_shared<rclcpp_lifecycle::LifecycleNode>("reentry_plugin_test");
    node_->declare_parameter("odom_topic",std::string("reentry_test_odom"));
    node_->declare_parameter("goal_checker.xy_goal_tolerance",.15);
    node_->declare_parameter("goal_checker.yaw_goal_tolerance",.08);
    node_->declare_parameter("FollowPath.handler.inversion_xy_tolerance",.15);
    node_->declare_parameter("FollowPath.completion_path_length",.15);
    node_->declare_parameter("FollowPath.terminal_goal_binding",true);
    node_->declare_parameter("FollowPath.terminal_track_xy_drift_limit",.05);
    node_->declare_parameter("FollowPath.initial_goal_region_shortcut",true);
    node_->declare_parameter("FollowPath.initial_goal_region_max_xy_error",.15);
    node_->declare_parameter("FollowPath.rpp.desired_linear_vel",.2);
    node_->declare_parameter("FollowPath.rpp.lookahead_dist",.75);
    map_=std::make_shared<nav2_costmap_2d::Costmap2DROS>("reentry_test_costmap");
    const auto changed=map_->set_parameters({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame","map"),
      rclcpp::Parameter("robot_base_frame","base_link"),
      rclcpp::Parameter("width",6),rclcpp::Parameter("height",6),
      rclcpp::Parameter("origin_x",-3.),rclcpp::Parameter("origin_y",-3.),
      rclcpp::Parameter("resolution",.01),rclcpp::Parameter("footprint_padding",.15),
      rclcpp::Parameter("footprint",std::string(
        "[[-0.647,-0.530],[0.647,-0.530],[0.647,0.530],[-0.647,0.530]]"))});
    for (const auto & result:changed) {ASSERT_TRUE(result.successful)<<result.reason;}
    ASSERT_EQ(map_->on_configure(rclcpp_lifecycle::State()),nav2_util::CallbackReturn::SUCCESS);
    map_->getCostmap()->resetMap(0,0,map_->getCostmap()->getSizeInCellsX(),
      map_->getCostmap()->getSizeInCellsY());
    std::fill_n(map_->getCostmap()->getCharMap(),
      map_->getCostmap()->getSizeInCellsX()*map_->getCostmap()->getSizeInCellsY(),0);
    checker_=loader_->createInstance<nav2_core::GoalChecker>(
      "official_lattice_rpp_adapter::FeasibleGoalChecker");
    controller_=loader_->createInstance<nav2_core::Controller>(
      "official_lattice_rpp_adapter::FeasibleRPP");
    checker_->initialize(node_,"goal_checker",map_);
    controller_->configure(node_,"FollowPath",map_->getTfBuffer(),map_);
    controller_->activate();
    publisher_=node_->create_publisher<nav_msgs::msg::Odometry>("reentry_test_odom",10);
    executor_.add_node(node_->get_node_base_interface());
    const auto until=std::chrono::steady_clock::now()+1s;
    while (publisher_->get_subscription_count()==0&&std::chrono::steady_clock::now()<until) {
      executor_.spin_some();std::this_thread::sleep_for(5ms);
    }
    ASSERT_GT(publisher_->get_subscription_count(),0u);
  }
  void TearDown() override {
    if (controller_) {controller_->deactivate();controller_->cleanup();}
    executor_.remove_node(node_->get_node_base_interface());
    controller_.reset();checker_.reset();publisher_.reset();node_.reset();
    if (map_) {map_->on_cleanup(rclcpp_lifecycle::State());map_.reset();}
    loader_.reset();
  }
  nav_msgs::msg::Path plan(double start) {
    nav_msgs::msg::Path p;p.header.frame_id="map";p.header.stamp=node_->now();
    p.poses={pose(start),pose(0.),pose(0.)};
    for (auto & x:p.poses) {x.header.stamp=p.header.stamp;}
    return p;
  }
  geometry_msgs::msg::PoseStamped input(double x,double speed=0.) {
    auto p=pose(x);p.header.stamp=node_->now();
    geometry_msgs::msg::TransformStamped t;t.header=p.header;t.child_frame_id="base_link";
    t.transform.translation.x=x;t.transform.rotation=p.pose.orientation;
    EXPECT_TRUE(map_->getTfBuffer()->setTransform(t,"synthetic_test_input",false));
    nav_msgs::msg::Odometry o;o.header=p.header;o.child_frame_id="base_link";
    o.pose.pose=p.pose;o.twist.twist.linear.x=speed;
    for (int i=0;i<5;++i) {publisher_->publish(o);executor_.spin_some();std::this_thread::sleep_for(2ms);}
    return p;
  }
  geometry_msgs::msg::TwistStamped command(double x,double speed=0.) {
    auto p=input(x,speed);geometry_msgs::msg::Twist v;v.linear.x=speed;
    return controller_->computeVelocityCommands(p,v,checker_.get());
  }
  void escapedTerminalAndFreshPlan() {
    controller_->setPlan(plan(.14));checker_->reset();
    EXPECT_NO_THROW(command(.14)); // Stopped initial original-region admission.
    EXPECT_THROW(command(.167),nav2_core::NoValidControl); // Active escape still fails.
    controller_->reset();
    controller_->setPlan(plan(.167));checker_->reset();
  }
  bool reached(double x) {
    auto p=input(x);return checker_->isGoalReached(p.pose,pose(0.).pose,geometry_msgs::msg::Twist{});
  }
  std::unique_ptr<class_loader::ClassLoader> loader_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::shared_ptr<nav2_core::Controller> controller_;
  std::shared_ptr<nav2_core::GoalChecker> checker_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(PluginReentry, FreshOrdinaryPathCanRepairOutsideRegionWithoutGrantingCompletion) {
  publisher_->on_activate();escapedTerminalAndFreshPlan();
  geometry_msgs::msg::TwistStamped cmd;
  ASSERT_NO_THROW(cmd=command(.167));
  EXPECT_GT(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.linear.y,0.);
  EXPECT_FALSE(reached(.167));
  cmd=command(.149,.1);EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0.);EXPECT_FALSE(reached(.149));
  for (int i=0;i<3;++i) {
    cmd=command(.149);EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);
    EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0.);EXPECT_FALSE(reached(.149));
    std::this_thread::sleep_for(90ms);
  }
  // Reentry stop permission is not final success. Final original-goal stop
  // dwell must still independently mature through the real controller/checker.
  bool final=false;
  for (int i=0;i<15&&!final;++i) {command(.149);final=reached(.149);std::this_thread::sleep_for(50ms);}
  EXPECT_TRUE(final);
}

TEST_F(PluginReentry, OrdinaryRepairRetainsFilledCommandCollisionChecking) {
  publisher_->on_activate();escapedTerminalAndFreshPlan();
  geometry_msgs::msg::TwistStamped clear_command;
  ASSERT_NO_THROW(clear_command=command(.167));ASSERT_GT(clear_command.twist.linear.x,0.);
  unsigned int mx,my;ASSERT_TRUE(map_->getCostmap()->worldToMap(-.68,0.,mx,my));
  map_->getCostmap()->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
  try {
    command(.167);FAIL()<<"A repair command crossed a lethal filled-body sweep";
  } catch (const nav2_core::NoValidControl & error) {
    const std::string message=error.what();
    EXPECT_TRUE(message.find("collision")!=std::string::npos||
      message.find("Collision")!=std::string::npos||
      message.find("Filled")!=std::string::npos)<<message;
  }
  EXPECT_FALSE(reached(.167));
}
}  // namespace
