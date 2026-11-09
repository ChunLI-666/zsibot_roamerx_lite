// SPDX-License-Identifier: Apache-2.0
// Exercise the real plugin DSO through its public Jazzy interfaces. The input
// poses are synthetic except for the explicitly cached B34 geometry below.
// All use current fixture clocks; none is raw DDS replay or freshness proof.
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
#include "fixtures/b34_terminal_approaches.hpp"

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
  virtual double entryRadius() const {return 0.;}
  virtual double handoffRadius() const {return .15;}
  virtual bool initialShortcut() const {return true;}
  virtual bool stoppedEntry() const {return false;}
  virtual double driftLimit() const {return .05;}
  virtual double mapOrigin() const {return -3.;}
  virtual void configureExtra() {}
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
    node_->declare_parameter("FollowPath.handler.inversion_xy_tolerance",handoffRadius());
    node_->declare_parameter("FollowPath.completion_path_length",handoffRadius());
    node_->declare_parameter("FollowPath.terminal_rotation_entry_xy_tolerance",entryRadius());
    node_->declare_parameter("FollowPath.terminal_goal_binding",true);
    node_->declare_parameter("FollowPath.terminal_track_xy_drift_limit",driftLimit());
    node_->declare_parameter("FollowPath.terminal_stop_before_rotation",stoppedEntry());
    node_->declare_parameter("FollowPath.initial_goal_region_shortcut",initialShortcut());
    node_->declare_parameter("FollowPath.initial_goal_region_max_xy_error",.15);
    node_->declare_parameter("FollowPath.rpp.desired_linear_vel",.2);
    node_->declare_parameter("FollowPath.rpp.lookahead_dist",.75);
    configureExtra();
    map_=std::make_shared<nav2_costmap_2d::Costmap2DROS>("reentry_test_costmap");
    const auto changed=map_->set_parameters({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame","map"),
      rclcpp::Parameter("robot_base_frame","base_link"),
      rclcpp::Parameter("width",6),rclcpp::Parameter("height",6),
      rclcpp::Parameter("origin_x",mapOrigin()),rclcpp::Parameter("origin_y",mapOrigin()),
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
  nav_msgs::msg::Path turnPlan(double start) {
    auto p=plan(start);p.poses.push_back(pose(0.,0.));p.poses.push_back(pose(0.,0.));
    return p;
  }
  geometry_msgs::msg::PoseStamped input(double x,double speed=0.) {
    return inputAt(x,0.,M_PI,speed);
  }
  geometry_msgs::msg::PoseStamped inputAt(double x,double y,double heading,
    double speed=0.,double lateral=0.,double angular=0.) {
    auto p=pose(x,heading);p.pose.position.y=y;p.header.stamp=node_->now();
    geometry_msgs::msg::TransformStamped t;t.header=p.header;t.child_frame_id="base_link";
    t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation=p.pose.orientation;
    EXPECT_TRUE(map_->getTfBuffer()->setTransform(t,"synthetic_test_input",false));
    nav_msgs::msg::Odometry o;o.header=p.header;o.child_frame_id="base_link";
    o.pose.pose=p.pose;o.twist.twist.linear.x=speed;o.twist.twist.linear.y=lateral;
    o.twist.twist.angular.z=angular;
    for (int i=0;i<5;++i) {publisher_->publish(o);executor_.spin_some();std::this_thread::sleep_for(2ms);}
    return p;
  }
  geometry_msgs::msg::TwistStamped command(double x,double speed=0.) {
    auto p=input(x,speed);geometry_msgs::msg::Twist v;v.linear.x=speed;
    return controller_->computeVelocityCommands(p,v,checker_.get());
  }
  geometry_msgs::msg::TwistStamped commandAt(double x,double y,double heading,
    double speed=0.,double lateral=0.,double angular=0.) {
    auto p=inputAt(x,y,heading,speed,lateral,angular);geometry_msgs::msg::Twist v;
    v.linear.x=speed;v.linear.y=lateral;v.angular.z=angular;
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

class CenteredEntry : public PluginReentry {
protected:
  double entryRadius() const override {return .05;}
};
// Keep the old .15 handler here to make the old-DSO negative control prove
// premature admission, rather than merely failing its old parameter coupling.
class CenteredLegacyHandoff : public PluginReentry {
protected:
  double entryRadius() const override {return .05;}
};
class CenteredNoShortcut : public CenteredLegacyHandoff {
protected:
  bool initialShortcut() const override {return false;}
};
class LegacyNoShortcut : public PluginReentry {
protected:
  bool initialShortcut() const override {return false;}
};

TEST_F(LegacyNoShortcut, ZeroEntryOptionRetainsQuantizedOriginalHandoff) {
  publisher_->on_activate();auto p=plan(.3);
  p.poses[1].pose.position.x=-.008;p.poses.back().pose.position.x=-.004;
  controller_->setPlan(p);checker_->reset();
  auto cmd=command(.140); // Original error .144; endpoint distance .148.
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0.);
}

TEST_F(CenteredLegacyHandoff, EdgeOfOriginalGoalFollowsPathBeforeInitialTurn) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.149));checker_->reset();
  auto cmd=command(.149);
  EXPECT_GT(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.linear.y,0.);
  EXPECT_NEAR(cmd.twist.angular.z,0.,1e-8);
}

TEST_F(CenteredNoShortcut, OrdinaryTerminalHandoffDoesNotStopAtOriginalDiscEdge) {
  publisher_->on_activate();controller_->setPlan(plan(.3));checker_->reset();
  auto cmd=command(.128);
  EXPECT_GT(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.linear.y,0.);
  EXPECT_NEAR(cmd.twist.angular.z,0.,1e-8);EXPECT_FALSE(reached(.128));
}

TEST_F(CenteredEntry, FinalRunApproachesCenterWithOriginalHandlerRadius) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.128));checker_->reset();
  auto cmd=command(.128);
  EXPECT_GT(cmd.twist.linear.x,0.);EXPECT_NEAR(cmd.twist.angular.z,0.,1e-8);
}

TEST_F(CenteredEntry, SamePlanApproachCanHandoffToCheckedTerminalTurn) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.128));checker_->reset();
  auto approach=command(.128);ASSERT_GT(approach.twist.linear.x,0.);
  // The initial shortcut opportunity was consumed above. No setPlan/reset is
  // allowed between the approach and this ordinary ordered terminal handoff.
  auto turn=command(.02);
  EXPECT_DOUBLE_EQ(turn.twist.linear.x,0.);EXPECT_GT(std::abs(turn.twist.angular.z),0.);
}

TEST_F(CenteredEntry, QuantizedTerminalConstraintApproachesOriginalCenter) {
  publisher_->on_activate();auto p=turnPlan(.128);
  for (std::size_t i=1;i+1<p.poses.size();++i) {p.poses[i].pose.position.x=.004;}
  controller_->setPlan(p);checker_->reset();
  auto cmd=command(.052); // .048 from the cell endpoint, .052 from the original goal.
  RecordProperty("command_vx",std::to_string(cmd.twist.linear.x));
  RecordProperty("command_wz",std::to_string(cmd.twist.angular.z));
  EXPECT_GT(cmd.twist.linear.x,0.);EXPECT_NEAR(cmd.twist.angular.z,0.,1e-8);
}

TEST_F(CenteredEntry, InsideCenterRetainsCheckedPureTurnAndOriginalOuterGuard) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.02));checker_->reset();
  auto cmd=command(.02);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_GT(std::abs(cmd.twist.angular.z),0.);
  // Entry radius does not shrink the original execution/acceptance region.
  EXPECT_NO_THROW(command(.12));
  EXPECT_THROW(command(.23),nav2_core::NoValidControl);
}

TEST_F(CenteredEntry, FinalStoppedAcceptanceKeepsOriginalGoalTolerance) {
  publisher_->on_activate();controller_->setPlan(plan(.02));checker_->reset();
  auto cmd=command(.02);EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);
  EXPECT_FALSE(reached(.12));
  bool final=false;
  for (int i=0;i<12&&!final;++i) {command(.12);final=reached(.12);std::this_thread::sleep_for(50ms);}
  EXPECT_TRUE(final);
}

TEST_F(CenteredEntry, CenteringStillRejectsFilledCommandCollision) {
  publisher_->on_activate();controller_->setPlan(plan(.128));checker_->reset();
  auto clear=command(.128);ASSERT_GT(clear.twist.linear.x,0.);
  unsigned int mx,my;ASSERT_TRUE(map_->getCostmap()->worldToMap(-.69,0.,mx,my));
  map_->getCostmap()->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(command(.128),nav2_core::NoValidControl);EXPECT_FALSE(reached(.128));
}

class StoppedTerminal : public CenteredEntry {
protected:
  bool stoppedEntry() const override {return true;}
  bool initialShortcut() const override {return false;}
  void configureExtra() override {
    node_->set_parameter(rclcpp::Parameter("FollowPath.rpp.desired_linear_vel",.4));
    node_->declare_parameter("FollowPath.rpp.min_approach_linear_velocity",.4);
    node_->declare_parameter("FollowPath.rpp.regulated_linear_scaling_min_speed",.4);
    node_->declare_parameter("FollowPath.rpp.rotate_to_heading_angular_vel",.3);
    node_->declare_parameter("FollowPath.rpp.max_angular_accel",6.);
  }
};
class StoppedInitial : public StoppedTerminal {
protected:
  bool initialShortcut() const override {return true;}
  double driftLimit() const override {return 0.;}
};
class StoppedB34Geometry : public StoppedTerminal {
protected:
  double mapOrigin() const override {return 1.;}
};

TEST_F(CenteredNoShortcut, DefaultOffRetainsB19MovingTerminalTurn) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.3));checker_->reset();
  auto cmd=command(.04166,.6508);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_GT(std::abs(cmd.twist.angular.z),0.);
}

TEST_F(StoppedB34Geometry, RecordedLastTracksReachEarlyBrakeAndMovingTurnGate) {
  publisher_->on_activate();
  for (const auto & recorded:b34TerminalApproaches()) {
    SCOPED_TRACE(recorded.plan_header_ns);
    nav_msgs::msg::Path p;p.header.frame_id="map";p.header.stamp=node_->now();
    for (const auto & row:recorded.wire_poses) {
      auto x=pose(row[0],row[2]);x.pose.position.y=row[1];x.header=p.header;p.poses.push_back(x);
    }
    // Exact recorded final TRACK + terminal ROTATE + immutable wire goal.
    // The clock is current fixture time; body velocity below is a controlled
    // combination, not a claim that the planned anchor was the actual body.
    for (const auto & actual:{recorded.planned_track_start,recorded.native_pose}) {
      controller_->reset();controller_->setPlan(p);checker_->reset();
      auto cmd=commandAt(actual[0],actual[1],actual[2],recorded.body_twist[0],
        recorded.body_twist[1],recorded.body_twist[2]);
      EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0.);
    }
  }
}

TEST_F(StoppedTerminal, BrakeInOuterWindowThenRequireFullStopAtCenter) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.3));checker_->reset();
  auto brake=command(.14,.4);
  EXPECT_DOUBLE_EQ(brake.twist.linear.x,0.);EXPECT_DOUBLE_EQ(brake.twist.angular.z,0.);
  // Small linear velocity alone is insufficient while angular motion persists.
  auto still=commandAt(.04,0.,M_PI,.02,0.,.1);
  EXPECT_DOUBLE_EQ(still.twist.linear.x,0.);EXPECT_DOUBLE_EQ(still.twist.angular.z,0.);
  auto turn=command(.04);
  EXPECT_DOUBLE_EQ(turn.twist.linear.x,0.);EXPECT_GT(std::abs(turn.twist.angular.z),0.);
  // Once the checked run starts, retain the original .15 execution region.
  auto active=command(.10);
  EXPECT_DOUBLE_EQ(active.twist.linear.x,0.);EXPECT_GT(std::abs(active.twist.angular.z),0.);
}

TEST_F(StoppedTerminal, StopShortOfCenterRequestsFiniteFreshPath) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.3));checker_->reset();
  command(.14,.4);
  try {command(.10);FAIL()<<"An off-center braking stop needs a fresh path";}
  catch (const nav2_core::NoValidControl & error) {
    EXPECT_NE(std::string(error.what()).find("fresh original-goal path"),std::string::npos);
  }
  controller_->setPlan(turnPlan(.10));checker_->reset();
  auto repair=command(.10);EXPECT_GT(repair.twist.linear.x,0.);
}

TEST_F(StoppedTerminal, CompatibleBrakingPlanCannotResetMandatoryPrefix) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.3));checker_->reset();
  command(.14,.4);
  // A compatible replacement contains an unrelated loop. While braking it must
  // validate completely, but cannot replace the current mandatory cursor.
  auto replacement=turnPlan(.3);
  replacement.poses={pose(.3),pose(.3,M_PI_2),pose(.3,M_PI_2),
    pose(.3,M_PI),pose(0.),pose(0.,0.),pose(0.,0.)};
  for (auto & p:replacement.poses) {p.header=replacement.header;}
  controller_->setPlan(replacement);
  auto waiting=command(.12,.2);
  EXPECT_DOUBLE_EQ(waiting.twist.linear.x,0.);EXPECT_DOUBLE_EQ(waiting.twist.angular.z,0.);
  EXPECT_FALSE(reached(.12));
}

TEST_F(StoppedInitial, InitialTurnNeedsStopEvenWithoutDriftExtension) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.3));checker_->reset();
  auto moving=command(.03367,.5577);
  EXPECT_DOUBLE_EQ(moving.twist.linear.x,0.);EXPECT_DOUBLE_EQ(moving.twist.angular.z,0.);
  auto stopped=command(.03367);
  EXPECT_DOUBLE_EQ(stopped.twist.linear.x,0.);EXPECT_GT(std::abs(stopped.twist.angular.z),0.);
}

TEST_F(StoppedTerminal, FirstCommandRechecksFreshCenterWithoutFreezingTFHeaders) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.3));checker_->reset();
  auto old=input(.04);auto fresh=input(.06);
  EXPECT_GT(rclcpp::Time(fresh.header.stamp).nanoseconds(),rclcpp::Time(old.header.stamp).nanoseconds());
  EXPECT_THROW(controller_->computeVelocityCommands(old,geometry_msgs::msg::Twist{},checker_.get()),
    nav2_core::NoValidControl);
  controller_->reset();controller_->setPlan(turnPlan(.3));checker_->reset();
  old=input(.04);fresh=input(.0401);
  geometry_msgs::msg::TwistStamped cmd;
  ASSERT_NO_THROW(cmd=controller_->computeVelocityCommands(old,geometry_msgs::msg::Twist{},checker_.get()));
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_GT(std::abs(cmd.twist.angular.z),0.);
  EXPECT_EQ(cmd.header.stamp,fresh.header.stamp);
}

TEST_F(StoppedTerminal, FailedFreshCheckedZeroNeverConsumesRotationEntry) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.3));checker_->reset();
  auto old=input(.04);
  // This corner is outside the old origin's entire rotation radius, but inside
  // the newly sampled stationary footprint. A fresh yaw sign change must take
  // the checked-zero path; failure cannot authorize a later off-center turn.
  inputAt(.049,0.,std::atan2(.68,.797));
  unsigned int mx,my;ASSERT_TRUE(map_->getCostmap()->worldToMap(1.105,0.,mx,my));
  map_->getCostmap()->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
  try {
    controller_->computeVelocityCommands(old,geometry_msgs::msg::Twist{},checker_.get());
    FAIL()<<"A blocked fresh checked zero must fail";
  } catch (const nav2_core::NoValidControl & error) {
    EXPECT_NE(std::string(error.what()).find("Filled terminal stop sweep blocked"),std::string::npos)
      <<error.what();
  }
  map_->getCostmap()->setCost(mx,my,nav2_costmap_2d::FREE_SPACE);
  EXPECT_THROW(command(.10),nav2_core::NoValidControl);
}

TEST_F(StoppedTerminal, FarFinalTrackPathAlignmentIsNotTerminalBraking) {
  publisher_->on_activate();controller_->setPlan(plan(2.));checker_->reset();
  // finalPrefixActive also includes ordinary final TRACK. Goal is still1m
  // away: this RPP pure-yaw command aligns to the path, not to final heading.
  auto cmd=commandAt(1.,0.,M_PI_2,0.,0.,.1);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_GT(std::abs(cmd.twist.angular.z),0.);
}
}  // namespace
