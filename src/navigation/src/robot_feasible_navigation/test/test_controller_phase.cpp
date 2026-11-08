// Regression fixture for the actual controller server and real PoseProgressChecker.
// Real SegmentControllerServer bodies and real Jazzy PoseProgressChecker are used.
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <thread>
#include "gtest/gtest.h"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "robot_feasible_navigation/segment_controller_server.hpp"
using robot_feasible_navigation::SegmentControllerServer;
using robot_feasible_navigation::SegmentCoordinator;
using Pose=geometry_msgs::msg::PoseStamped;
constexpr double pi=3.14159265358979323846;
namespace {
Pose pose(double x,double yaw,std::int64_t stamp=1000000000000LL) {
  Pose p;p.header.frame_id="map";p.header.stamp=rclcpp::Time(stamp,RCL_ROS_TIME);
  p.pose.position.x=x;p.pose.orientation.z=std::sin(yaw/2);p.pose.orientation.w=std::cos(yaw/2);return p;
}
nav_msgs::msg::Path turn(std::int64_t stamp,double goal_yaw=pi) {
  nav_msgs::msg::Path p;p.header=pose(0,0,stamp).header;
  for(int i=0;i<=4;++i)p.poses.push_back(pose(0,goal_yaw*i/4,stamp));return p;
}
nav_msgs::msg::Path approach(std::int64_t stamp) {
  nav_msgs::msg::Path p;p.header=pose(.18,pi,stamp).header;
  for(int i=18;i>=0;--i)p.poses.push_back(pose(i*.01,pi,stamp));
  p.poses.push_back(pose(0,pi/2,stamp));p.poses.push_back(pose(0,0,stamp));return p;
}
nav_msgs::msg::Path microTrack(std::int64_t stamp) {
  nav_msgs::msg::Path p;p.header=pose(0,0,stamp).header;
  p.poses={pose(0,0,stamp),pose(.01,0,stamp),pose(.02,0,stamp),
    pose(.02,pi/4,stamp),pose(.02,pi/2,stamp)};
  for(int i=1;i<=100;++i) {
    auto end=pose(.02,pi/2,stamp);end.pose.position.y=i*.01;p.poses.push_back(end);
  }
  return p;
}
class PlanSink : public nav2_core::Controller {
public:
  int resets=0,plans=0,unexpected_compute=0;
  double received_xy=-1.;
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,std::string,
    std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)override{}
  void cleanup()override{} void activate()override{} void deactivate()override{}
  void setPlan(const nav_msgs::msg::Path&)override{++plans;}
  void reset()override{++resets;}
  void setSpeedLimit(const double&,const bool&)override{}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const Pose&,
    const geometry_msgs::msg::Twist&,nav2_core::GoalChecker* checker)override {
    geometry_msgs::msg::Pose tolerance;geometry_msgs::msg::Twist velocity;
    if(checker&&checker->getTolerances(tolerance,velocity))received_xy=tolerance.position.x;
    ++unexpected_compute;throw std::runtime_error("Motion compute escaped expected zero branch");
  }
};
class GoalProbe : public nav2_core::GoalChecker {
public:
  bool accepts=true;
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,const std::string&,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)override{}
  void reset()override{}
  bool isGoalReached(const geometry_msgs::msg::Pose&,const geometry_msgs::msg::Pose&,
    const geometry_msgs::msg::Twist&)override{return accepts;}
  bool getTolerances(geometry_msgs::msg::Pose&p,geometry_msgs::msg::Twist&v)override {
    p.position.x=p.position.y=.15;p.orientation.z=std::sin(.08/2);p.orientation.w=std::cos(.08/2);
    v.linear.x=v.linear.y=.03;v.angular.z=.05;return true;
  }
};
class CountedPoseProgress : public nav2_controller::PoseProgressChecker {
public:
  int checks=0,resets=0;
  void bind(rclcpp::Clock::SharedPtr c) {
    clock_=c;radius_=.05;required_movement_angle_=.1;
    time_allowance_=rclcpp::Duration::from_seconds(10);baseline_pose_set_=false;
  }
  bool check(Pose&p)override{++checks;return nav2_controller::PoseProgressChecker::check(p);}
  void reset()override{++resets;nav2_controller::PoseProgressChecker::reset();}
};
class Peer : public SegmentControllerServer {
public:
  Peer():SegmentControllerServer(rclcpp::NodeOptions().enable_rosout(false)
    .start_parameter_services(false).start_parameter_event_publisher(false)) {
    SegmentCoordinator::Config cfg;cfg.xy_tolerance=.15;cfg.yaw_tolerance=.08;
    cfg.stopped_linear_velocity=.03;cfg.stopped_angular_velocity=.05;cfg.odom_timeout=.3;
    coordinator_=SegmentCoordinator(cfg);
    current_controller_="FollowPath";current_goal_checker_="Checker";current_progress_checker_="Progress";
    sink=std::make_shared<PlanSink>();goal=std::make_shared<GoalProbe>();progress=std::make_shared<CountedPoseProgress>();
    controllers_[current_controller_]=sink;goal_checkers_[current_goal_checker_]=goal;
    configureIntermediateGoalChecker(cfg);progress_checkers_[current_progress_checker_]=progress;
    if(rcl_enable_ros_time_override(get_clock()->get_clock_handle())!=RCL_RET_OK)throw std::runtime_error("test ROS clock enable failed");
    progress->bind(get_clock());time(0);
  }
  std::shared_ptr<PlanSink>sink;std::shared_ptr<GoalProbe>goal;std::shared_ptr<CountedPoseProgress>progress;
  Pose actual;geometry_msgs::msg::Twist measured;
  bool odom_valid=true,sweep_free=true,zero_ok=true;int zeros=0,final_zeros=0,activations=0;
  std::int64_t ns=1000000000000LL;
  void time(double s) {
    ns=1000000000000LL+static_cast<std::int64_t>(s*1e9);
    if(rcl_set_ros_time_override(get_clock()->get_clock_handle(),ns)!=RCL_RET_OK)throw std::runtime_error("test clock set failed");
    actual.header.stamp=rclcpp::Time(ns,RCL_ROS_TIME);
  }
  void load(const nav_msgs::msg::Path&p){setPlannerPath(p);}
  void external(const nav_msgs::msg::Path&p) {
    nav2_msgs::action::FollowPath::Goal g;g.path=p;g.controller_id="FollowPath";
    g.goal_checker_id="Checker";g.progress_checker_id="Progress";
    installPendingPlannerGoal(g); // Actual extracted updateGlobalPath body, not a test replica.
  }
  void queue(const nav_msgs::msg::Path&p){queued_path_=p;}
  bool tick() {
    const bool before=terminal_rotation_active_;const bool done=isGoalReached();
    if(!before&&terminal_rotation_active_)++activations;return done;
  }
  bool finalActive()const{return terminal_rotation_active_;}
  void emitSameCycle(){computeAndPublishVelocity();}
  void zeroCycle(){if(!tick())computeAndPublishVelocity();}
  void ageExistingWait(double elapsed) {
    if(rotation_anchor_wait_since_)rotation_anchor_wait_since_=std::chrono::steady_clock::now()-
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(elapsed));
  }
  void expireAnchor(){rotation_anchor_wait_since_=std::chrono::steady_clock::now()-std::chrono::seconds(16);}
  void guardNextTurn(bool enabled){require_next_rotation_clearance_=enabled;}
  std::size_t segmentIndex()const{return coordinator_.index();}
  bool repairPending()const{return track_approach_pending_;}
  double controlXY(){return intermediateXYTolerance();}
  void configurePhaseXY(double xy,bool enabled) {
    require_next_rotation_clearance_=enabled;
    auto cfg=coordinator_.config();cfg.xy_tolerance=xy;coordinator_=SegmentCoordinator(cfg);
    configureIntermediateGoalChecker(cfg);
  }
protected:
  bool getRobotPose(Pose&out)override{out=actual;return true;}
  Pose toPathPose(const Pose&p)override{return p;}
  bool measuredVelocity(geometry_msgs::msg::Twist&v,std::int64_t*stamp=nullptr)override {
    v=measured;if(stamp)*stamp=ns-5000000;return odom_valid;
  }
  bool rotationSweepFree(const Pose&,const Pose&)override{return sweep_free;}
  void publishSegmentStop()override{++zeros;}
  std::int64_t publishFinalGoalStop(const Pose&)override {
    if(!zero_ok)throw nav2_core::NoValidControl("Injected checked-zero sweep failure");
    ++final_zeros;final_stop_zero_this_cycle_=true;last_valid_cmd_time_=now();return ns;
  }
};
class ControllerPhase : public ::testing::Test {
protected:
  static void SetUpTestSuite(){
    const char * domain=std::getenv("ROS_DOMAIN_ID");
    if(!domain||std::string(domain)!="190")throw std::runtime_error("Controller tests require ROS_DOMAIN_ID=190");
    if(!rclcpp::ok()){int argc=0;rclcpp::init(argc,nullptr);}
  }
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp()override{s=std::make_shared<Peer>();s->actual=pose(.149,.2);s->load(turn(s->ns));}
  std::shared_ptr<Peer>s;
};
TEST_F(ControllerPhase, QueuedOldRestoreMustNotResetSameGoalChecker) {
  ASSERT_FALSE(s->tick());ASSERT_TRUE(s->finalActive());
  const int resets=s->progress->resets;s->queue(turn(s->ns-1));s->actual=pose(.1519187745,.2,s->ns);
  ASSERT_FALSE(s->tick());EXPECT_FALSE(s->finalActive());EXPECT_EQ(s->progress->resets,resets);
}
TEST_F(ControllerPhase, FreshValidExternalPlansEveryCycleCannotRearmFailedFinal500Times) {
  ASSERT_FALSE(s->tick());ASSERT_TRUE(s->finalActive());s->actual=pose(.1519187745,.2,s->ns);
  ASSERT_FALSE(s->tick());const int resets=s->progress->resets;bool timed_out=false;
  for(int i=1;i<=500;++i) {
    s->time(i);s->external(turn(s->ns)); // New validated compatible header, non-queue installation.
    s->actual=pose(.149,i%2?.2:-.2,s->ns);s->ageExistingWait(i);
    try {s->tick();}catch(const nav2_core::NoValidControl&){timed_out=true;break;}
    // No angular source progress is fabricated by the plan; measured stopped is explicit.
    if(s->finalActive()){s->actual=pose(.1519187745,i%2?.2:-.2,s->ns);s->tick();}
  }
  EXPECT_LE(s->activations,1);EXPECT_EQ(s->progress->resets,resets);EXPECT_TRUE(timed_out);
}
TEST_F(ControllerPhase, HealthyZeroHoldCannotBypassTenSecondProgressDeadline) {
  s->actual=pose(.20,.2,s->ns);s->zeroCycle();const int checks=s->progress->checks;
  s->time(10.1);EXPECT_THROW(s->zeroCycle(),nav2_core::FailedToMakeProgress);
  EXPECT_GT(s->progress->checks,checks);EXPECT_EQ(s->sink->unexpected_compute,0);
}
TEST_F(ControllerPhase, ExistingFifteenSecondAnchorBoundaryIsActuallyRead) {
  s->actual=pose(.20,.2,s->ns);s->expireAnchor();
  EXPECT_THROW(s->tick(),nav2_core::NoValidControl);
}
TEST_F(ControllerPhase, GenuineAngularProgressBeyondThirtySecondsDoesNotTimeOut) {
  s->actual=pose(0,0,s->ns);ASSERT_FALSE(s->tick());ASSERT_TRUE(s->finalActive());
  const int resets=s->progress->resets;s->expireAnchor();s->measured.angular.z=.08;
  for(int i=1;i<=700;++i){s->time(i*.05);s->actual=pose(0,.08*i*.05,s->ns);ASSERT_NO_THROW(s->tick());}
  EXPECT_TRUE(s->finalActive());EXPECT_EQ(s->progress->resets,resets);
}
TEST_F(ControllerPhase, ValidCheckedStoppedDwellMayFinishAcrossTenSecondBoundary) {
  s->time(0);s->actual=pose(.18,0,s->ns);s->load(approach(s->ns));
  ASSERT_TRUE(s->progress->check(s->actual));s->time(9.85);s->actual=pose(.14,0,s->ns);
  bool done=false;
  for(int i=0;i<10&&!done;++i) {
    if(i){std::this_thread::sleep_for(std::chrono::milliseconds(50));s->time(9.85+i*.05);s->actual=pose(.14,0,s->ns);}
    ASSERT_NO_THROW(done=s->tick());
  }
  EXPECT_TRUE(done);EXPECT_GE(s->final_zeros,7);
}
TEST_F(ControllerPhase, InsideGeometryMovingOrCheckerFalseDoesNotGetDwellExemption) {
  s->time(0);s->actual=pose(.18,0,s->ns);s->load(approach(s->ns));
  ASSERT_TRUE(s->progress->check(s->actual));s->time(10.1);s->actual=pose(.14,0,s->ns);
  s->measured.linear.x=.04;EXPECT_THROW(s->tick(),nav2_core::FailedToMakeProgress);
  s->measured.linear.x=0;s->goal->accepts=false;EXPECT_THROW(s->tick(),nav2_core::FailedToMakeProgress);
}
TEST_F(ControllerPhase, FailedCheckedZeroNeverAcceptsOrExemptsProgress) {
  s->actual=pose(0,pi,s->ns);s->zero_ok=false;
  EXPECT_THROW(s->tick(),nav2_core::NoValidControl);
}

TEST_F(ControllerPhase, RestoredUnvalidatedAnchorCannotEnterMotionComputeInSameCycle) {
  s->actual=pose(.149,.2,s->ns);s->load(turn(s->ns));
  ASSERT_FALSE(s->tick());ASSERT_TRUE(s->finalActive());
  s->actual=pose(.1519187745,.2,s->ns);ASSERT_FALSE(s->tick());ASSERT_FALSE(s->finalActive());
  // Actual worker order: compute immediately after restore, before another phase check.
  EXPECT_NO_THROW(s->emitSameCycle());EXPECT_EQ(s->sink->unexpected_compute,0);
}
TEST_F(ControllerPhase, SameGoalFreshPlanEveryTickCannotRegainFirstZeroExemption500Times) {
  auto p=s;p->time(0);p->actual=pose(.18,0,p->ns);p->load(approach(p->ns));
  ASSERT_TRUE(p->progress->check(p->actual));const int resets=p->progress->resets;
  bool bounded_failure=false,done=false;int sent=0;
  for(int i=0;i<500&&!bounded_failure&&!done;++i) {
    // First twenty periods advance real steady time, too: no synthetic shortcut to the .5s budget.
    if(i&&i<=20)std::this_thread::sleep_for(std::chrono::milliseconds(50));
    p->time(9.85+i*.05);p->actual=pose(.14,0,p->ns);p->external(approach(p->ns));++sent;
    try{done=p->tick();}catch(const nav2_core::FailedToMakeProgress&){bounded_failure=true;}
  }
  EXPECT_FALSE(done);EXPECT_TRUE(bounded_failure);EXPECT_LT(sent,500);
  EXPECT_EQ(p->progress->resets,resets);
  ::testing::Test::RecordProperty("compatible_plans_sent",sent);
  ::testing::Test::RecordProperty("bounded_native_progress_failure",bounded_failure?1:0);
}
TEST_F(ControllerPhase, LegacyMicroTrackConsumesBeforeTranslation) {
  s->actual=pose(0,0,s->ns);s->load(microTrack(s->ns));s->sweep_free=false;
  ASSERT_FALSE(s->tick());EXPECT_EQ(s->segmentIndex(),1U);
  EXPECT_FALSE(s->repairPending()); // Preserved opt-out behavior reproduces the original handoff.
}
TEST_F(ControllerPhase, UnsafeNextTurnKeepsTrackAndSuppliesApproachTriggerToRPP) {
  s->guardNextTurn(true);s->actual=pose(0,0,s->ns);s->load(microTrack(s->ns));s->sweep_free=false;
  ASSERT_FALSE(s->tick());EXPECT_EQ(s->segmentIndex(),0U);EXPECT_TRUE(s->repairPending());
  const int zeros=s->zeros;
  EXPECT_THROW(s->emitSameCycle(),std::runtime_error);
  EXPECT_EQ(s->sink->unexpected_compute,1);EXPECT_DOUBLE_EQ(s->sink->received_xy,.005);
  EXPECT_EQ(s->zeros,zeros);
  geometry_msgs::msg::Pose tolerance;geometry_msgs::msg::Twist velocity;
  ASSERT_TRUE(s->goal->getTolerances(tolerance,velocity));EXPECT_DOUBLE_EQ(tolerance.position.x,.15);
}
TEST_F(ControllerPhase, SafeNextTurnStillRequiresMeasuredStopBeforeAdvance) {
  s->guardNextTurn(true);s->actual=pose(0,0,s->ns);s->load(microTrack(s->ns));s->sweep_free=false;s->tick();
  s->sweep_free=true;s->actual=pose(.02,0,s->ns);s->measured.linear.x=.1;
  ASSERT_FALSE(s->tick());EXPECT_EQ(s->segmentIndex(),0U);EXPECT_TRUE(s->repairPending());
  const int zeros=s->zeros;s->emitSameCycle();EXPECT_EQ(s->zeros,zeros+1);
  s->measured.linear.x=0.;ASSERT_FALSE(s->tick());EXPECT_EQ(s->segmentIndex(),1U);
  EXPECT_FALSE(s->repairPending());EXPECT_DOUBLE_EQ(s->controlXY(),.15);
}
TEST_F(ControllerPhase, SameCycleFreshPoseCannotBypassUnsafeNextTurnGate) {
  s->guardNextTurn(true);s->actual=pose(-.2,0,s->ns);s->load(microTrack(s->ns));s->sweep_free=false;
  ASSERT_FALSE(s->tick());EXPECT_FALSE(s->repairPending());
  s->actual=pose(0,0,s->ns);EXPECT_THROW(s->emitSameCycle(),std::runtime_error);
  EXPECT_TRUE(s->repairPending());EXPECT_EQ(s->segmentIndex(),0U);
  EXPECT_EQ(s->sink->unexpected_compute,1);EXPECT_DOUBLE_EQ(s->sink->received_xy,.005);
}
TEST_F(ControllerPhase, RepairCannotContinueOutsideOriginalHandoffRegion) {
  s->guardNextTurn(true);s->actual=pose(0,0,s->ns);s->load(microTrack(s->ns));s->sweep_free=false;s->tick();
  const int resets=s->progress->resets;s->actual=pose(.18,0,s->ns);
  EXPECT_THROW(s->tick(),nav2_core::NoValidControl);EXPECT_EQ(s->segmentIndex(),0U);
  EXPECT_EQ(s->progress->resets,resets);EXPECT_TRUE(s->repairPending());
}
TEST_F(ControllerPhase, TemporaryClearanceDuringBrakingDoesNotEraseRepairBound) {
  s->guardNextTurn(true);s->actual=pose(0,0,s->ns);s->load(microTrack(s->ns));s->sweep_free=false;s->tick();
  s->sweep_free=true;s->measured.linear.x=.1;s->tick();EXPECT_TRUE(s->repairPending());
  s->actual=pose(.18,0,s->ns);s->measured.linear.x=0.;
  EXPECT_THROW(s->tick(),nav2_core::NoValidControl);EXPECT_EQ(s->segmentIndex(),0U);
}
TEST_F(ControllerPhase, FreshPoseOutsideRepairBoundCannotHideBehindSameCycleBrakingZero) {
  s->guardNextTurn(true);s->actual=pose(0,0,s->ns);s->load(microTrack(s->ns));s->sweep_free=false;s->tick();
  s->sweep_free=true;s->measured.linear.x=.1;s->tick();
  const int zeros=s->zeros;s->actual=pose(.18,0,s->ns);
  EXPECT_THROW(s->emitSameCycle(),nav2_core::NoValidControl);
  EXPECT_EQ(s->zeros,zeros);EXPECT_EQ(s->segmentIndex(),0U);EXPECT_TRUE(s->repairPending());
}
TEST_F(ControllerPhase, DisabledGuardKeepsExistingSubFiveMillimeterConfiguration) {
  ASSERT_NO_THROW(s->configurePhaseXY(.001,false));
  s->actual=pose(0,0,s->ns);s->load(microTrack(s->ns));s->sweep_free=false;
  ASSERT_FALSE(s->tick());EXPECT_FALSE(s->repairPending());
  EXPECT_THROW(s->emitSameCycle(),std::runtime_error);
  EXPECT_DOUBLE_EQ(s->sink->received_xy,.001);EXPECT_EQ(s->segmentIndex(),0U);
}
TEST_F(ControllerPhase, EnabledGuardValidatesItsApproachTriggerAtConfiguration) {
  EXPECT_THROW(s->configurePhaseXY(.001,true),std::invalid_argument);
}
} // namespace
