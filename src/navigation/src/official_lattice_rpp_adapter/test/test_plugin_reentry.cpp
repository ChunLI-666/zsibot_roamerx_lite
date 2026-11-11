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
#include <atomic>
#include <future>
#include <limits>
#include <fstream>
#include <new>
#include <rcl/time.h>
#include "fixtures/b34_terminal_approaches.hpp"

using namespace std::chrono_literals;
namespace p1_allocation_fixture {
// Test executable only: a thread-directed one-shot pause at a real DSO
// allocation. No hook or altered code is linked into the product library.
std::atomic<std::size_t> size{0};
std::atomic<bool> armed{false},entered{false},released{false},timed_out{false};
thread_local bool target_thread=false;
class Barrier {
public:
  explicit Barrier(std::size_t bytes) {
    size=bytes;entered=false;released=false;timed_out=false;armed=true;
  }
  ~Barrier() {release();armed=false;}
  void release() {released=true;}
  bool wait() const {
    const auto end=std::chrono::steady_clock::now()+100ms;
    while(!entered&&std::chrono::steady_clock::now()<end)std::this_thread::sleep_for(1ms);
    return entered;
  }
};
struct TargetThread {
  TargetThread() {target_thread=true;}
  ~TargetThread() {target_thread=false;}
};
}
void * operator new(std::size_t bytes) {
  void * value=std::malloc(bytes?bytes:1);if(!value)throw std::bad_alloc();
  using namespace p1_allocation_fixture;
  if(target_thread&&bytes==size.load()&&armed.exchange(false)) {
    entered=true;const auto end=std::chrono::steady_clock::now()+80ms;
    while(!released&&std::chrono::steady_clock::now()<end)std::this_thread::sleep_for(1ms);
    if(!released)timed_out=true;
  }
  return value;
}
void operator delete(void * value) noexcept {std::free(value);}
void operator delete(void * value,std::size_t) noexcept {std::free(value);}
namespace {
class CallbackExecutor : public rclcpp::executors::SingleThreadedExecutor {
public:
  bool reserve(rclcpp::AnyExecutable & executable,std::chrono::nanoseconds wait) {
    return get_next_executable(executable,wait);
  }
  void execute(rclcpp::AnyExecutable & executable) {execute_any_executable(executable);}
  nav_msgs::msg::Odometry deliver(rclcpp::AnyExecutable & executable) {
    if (!executable.subscription) {throw std::runtime_error("No reserved Odom subscription");}
    auto sub=executable.subscription;auto message=sub->create_message();rclcpp::MessageInfo info;
    struct ReturnGroup {
      rclcpp::AnyExecutable & e;
      ~ReturnGroup() {if(e.callback_group)e.callback_group->can_be_taken_from().store(true);}
    } group{executable};
    if (!sub->take_type_erased(message.get(),info)) {throw std::runtime_error("Reserved Odom was not taken");}
    const auto received=*std::static_pointer_cast<nav_msgs::msg::Odometry>(message);
    // This is the registered subscription's real callback, with a message taken
    // from DDS. No observer subscription or copied callback stands in for it.
    sub->handle_message(message,info);sub->return_message(message);
    return received;
  }
};
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
  virtual std::string globalFrame() const {return "map";}
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
      rclcpp::Parameter("global_frame",globalFrame()),
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
    nav_msgs::msg::Path p;p.header.frame_id=globalFrame();p.header.stamp=node_->now();
    p.poses={pose(start),pose(0.),pose(0.)};
    for (auto & x:p.poses) {x.header=p.header;}
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
    auto p=pose(x,heading);p.header.frame_id=globalFrame();p.pose.position.y=y;
    geometry_msgs::msg::TransformStamped t;t.child_frame_id="base_link";
    t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation=p.pose.orientation;
    nav_msgs::msg::Odometry o;o.child_frame_id="base_link";
    o.pose.pose=p.pose;o.twist.twist.linear.x=speed;o.twist.twist.linear.y=lateral;
    o.twist.twist.angular.z=angular;
    // Each real send has its own original stamp. The old five identical
    // headers now deliberately mean a delivered duplicate/continuity break.
    for (int i=0;i<5;++i) {
      p.header.stamp=node_->now();t.header=p.header;o.header=p.header;
      EXPECT_TRUE(map_->getTfBuffer()->setTransform(t,"synthetic_test_input",false));
      publisher_->publish(o);executor_.spin_some();std::this_thread::sleep_for(2ms);
    }
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
  CallbackExecutor executor_;
};

class P1DeliveredStop : public PluginReentry {
protected:
  void SetUp() override {
    PluginReentry::SetUp();publisher_->on_activate();
    controller_->setPlan(plan(.02));checker_->reset();
  }
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::Odometry last_delivered_;
  std::size_t callbacks_=0;
  virtual void updateSourceClock() {}
  virtual double odomAge() const {return 0.;}
  void freshTransform() {
    updateSourceClock();
    current_pose_=pose(.02);current_pose_.header.frame_id=globalFrame();current_pose_.header.stamp=node_->now();
    geometry_msgs::msg::TransformStamped t;t.header=current_pose_.header;t.child_frame_id="base_link";
    t.transform.translation.x=.02;t.transform.rotation=current_pose_.pose.orientation;
    ASSERT_TRUE(map_->getTfBuffer()->setTransform(t,"p1_registered_callback_fixture",false));
  }
  nav_msgs::msg::Odometry message(double linear=0.,double angular=0.) {
    freshTransform();nav_msgs::msg::Odometry o;o.header=current_pose_.header;
    o.header.stamp=node_->now()-rclcpp::Duration::from_seconds(odomAge());
    o.child_frame_id="base_link";o.pose.pose=current_pose_.pose;
    o.twist.twist.linear.x=linear;o.twist.twist.angular.z=angular;return o;
  }
  rclcpp::AnyExecutable reserveOdometry(std::chrono::milliseconds wait=100ms) {
    const auto end=std::chrono::steady_clock::now()+wait;
    while (std::chrono::steady_clock::now()<end) {
      rclcpp::AnyExecutable executable;
      if (!executor_.reserve(executable,2ms)) {continue;}
      if (executable.subscription&&
        std::string(executable.subscription->get_topic_name())=="/reentry_test_odom") {return executable;}
      executor_.execute(executable);
    }
    throw std::runtime_error("Registered Odom callback was not ready within100ms");
  }
  void deliver(const nav_msgs::msg::Odometry & value) {
    publisher_->publish(value);auto executable=reserveOdometry();
    last_delivered_=executor_.deliver(executable);++callbacks_;
    ASSERT_EQ(last_delivered_.header.stamp,value.header.stamp);
    ASSERT_EQ(last_delivered_.child_frame_id,value.child_frame_id);
    // Copying NaN is intentional in its rejection case.
    if (std::isfinite(value.twist.twist.angular.z)) {
      ASSERT_DOUBLE_EQ(last_delivered_.twist.twist.angular.z,value.twist.twist.angular.z);
    }
  }
  geometry_msgs::msg::TwistStamped tick() {
    freshTransform();return controller_->computeVelocityCommands(current_pose_,geometry_msgs::msg::Twist{},checker_.get());
  }
  bool check() {
    freshTransform();return checker_->isGoalReached(current_pose_.pose,pose(0.).pose,geometry_msgs::msg::Twist{});
  }
  bool healthyFor(std::chrono::milliseconds duration,bool forbid_ready=false) {
    const auto start=std::chrono::steady_clock::now();const auto end=start+duration;
    auto next_tick=start;auto next_sample=start;bool ready=false;
    while (std::chrono::steady_clock::now()<end) {
      deliver(message());
      const auto now=std::chrono::steady_clock::now();
      if (now>=next_tick) {
        const auto cmd=tick();EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0.);
        ready=check();if(forbid_ready)EXPECT_FALSE(ready);
        next_tick=now+50ms;
      }
      next_sample+=2ms;std::this_thread::sleep_until(next_sample);
    }
    return ready;
  }
  void establishReady() {
    ASSERT_TRUE(healthyFor(450ms));ASSERT_TRUE(check());
  }
  void fullDwellAfterRevocation() {
    const auto start=std::chrono::steady_clock::now();
    ASSERT_FALSE(check());const auto cmd=tick();
    EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0.);
    EXPECT_FALSE(check());healthyFor(275ms,true);
    EXPECT_LT(std::chrono::steady_clock::now()-start,300ms);
    EXPECT_TRUE(healthyFor(150ms));
  }
};

TEST_F(P1DeliveredStop, Healthy500HzOriginalSamplesCompleteWith20HzControl) {
  const auto start=std::chrono::steady_clock::now();
  ASSERT_TRUE(healthyFor(450ms));EXPECT_TRUE(check());
  EXPECT_GE(callbacks_,150u);EXPECT_GE(std::chrono::steady_clock::now()-start,300ms);
  RecordProperty("registered_dds_callbacks",static_cast<int>(callbacks_));
}

TEST_F(P1DeliveredStop, FourteenMillisecondAngularPulseRevokesReadyBetweenControllerTicks) {
  establishReady();const auto start=std::chrono::steady_clock::now();std::size_t pulse=0;
  const auto end=start+14ms;
  do {deliver(message(0.,.08));++pulse;std::this_thread::sleep_for(2ms);} while(std::chrono::steady_clock::now()<end);
  ASSERT_FALSE(check());deliver(message());ASSERT_FALSE(check());
  ASSERT_LT(std::chrono::steady_clock::now()-start,50ms);EXPECT_GE(pulse,5u);
  RecordProperty("delivered_nonstop_samples",static_cast<int>(pulse));fullDwellAfterRevocation();
}

TEST_F(P1DeliveredStop, FourteenMillisecondLinearPulseRevokesReadyBetweenControllerTicks) {
  establishReady();const auto start=std::chrono::steady_clock::now();const auto end=start+14ms;
  do {deliver(message(.0314));std::this_thread::sleep_for(2ms);} while(std::chrono::steady_clock::now()<end);
  ASSERT_FALSE(check());deliver(message());ASSERT_FALSE(check());
  ASSERT_LT(std::chrono::steady_clock::now()-start,50ms);fullDwellAfterRevocation();
}

TEST_F(P1DeliveredStop, DuplicateAndRegressingDeliveredHeadersCannotBeHiddenByALaterZero) {
  for (bool regression:{false,true}) {
    establishReady();auto bad=last_delivered_;
    if(regression)bad.header.stamp=rclcpp::Time(rclcpp::Time(bad.header.stamp).nanoseconds()-1000000);
    deliver(bad);ASSERT_FALSE(check());deliver(message());ASSERT_FALSE(check());fullDwellAfterRevocation();
  }
}

TEST_F(P1DeliveredStop, NonfiniteDeliveredTwistRevokesReadyBeforeTheNextControlRead) {
  establishReady();auto bad=message();bad.twist.twist.angular.z=std::numeric_limits<double>::quiet_NaN();
  deliver(bad);ASSERT_FALSE(check());deliver(message());fullDwellAfterRevocation();
}

class P1ShortProof : public P1DeliveredStop {
protected:
  void configureExtra() override {node_->declare_parameter("FollowPath.odom_timeout",.1);}
  double odomAge() const override {return .06;}
};

TEST_F(P1ShortProof, FreshHealthyCallbackAndRepeatedControlReadsCannotExtendTheOriginalProof) {
  healthyFor(450ms);
  auto original=message();
  deliver(original);tick();ASSERT_TRUE(check());
  const auto start=std::chrono::steady_clock::now();
  std::this_thread::sleep_until(start+15ms);tick();ASSERT_TRUE(check());
  std::this_thread::sleep_until(start+55ms);EXPECT_FALSE(check());
  EXPECT_THROW(tick(),nav2_core::NoValidControl);
  // A new healthy callback can replace the input, but never grants completion
  // until the controller certifies that new input through its ordinary cycle.
  deliver(message());EXPECT_FALSE(check());tick();EXPECT_TRUE(check());
  EXPECT_LT(std::chrono::steady_clock::now()-start,250ms);
}

class P1TwoClocks : public P1DeliveredStop {
protected:
  void configureExtra() override {node_->declare_parameter("FollowPath.odom_timeout",.5);}
  void SetUp() override {
    P1DeliveredStop::SetUp();source_base_=node_->now().nanoseconds();steady_base_=std::chrono::steady_clock::now();
    ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()),RCL_RET_OK);
    manual_clock_=true;updateSourceClock();
  }
  void TearDown() override {
    if(manual_clock_) {EXPECT_EQ(rcl_disable_ros_time_override(node_->get_clock()->get_clock_handle()),RCL_RET_OK);manual_clock_=false;}
    P1DeliveredStop::TearDown();
  }
  bool manual_clock_=false;double rate_=1.;std::int64_t source_base_=0;
  std::chrono::steady_clock::time_point steady_base_;
  void updateSourceClock() override {
    if(!manual_clock_)return;
    const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now()-steady_base_).count();
    ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(),
      source_base_+static_cast<std::int64_t>(elapsed*rate_)),RCL_RET_OK);
  }
  void rate(double value) {
    updateSourceClock();source_base_=node_->now().nanoseconds();steady_base_=std::chrono::steady_clock::now();rate_=value;
  }
  void jump(std::chrono::milliseconds value) {source_base_+=std::chrono::duration_cast<std::chrono::nanoseconds>(value).count();updateSourceClock();}
};

TEST_F(P1TwoClocks, SourceGapRevokesContinuityWithoutAReceiveGap) {
  establishReady();const auto previous=last_delivered_;const auto start=std::chrono::steady_clock::now();
  jump(310ms);deliver(message());ASSERT_FALSE(check());
  EXPECT_GT(rclcpp::Time(last_delivered_.header.stamp).nanoseconds()-
    rclcpp::Time(previous.header.stamp).nanoseconds(),300000000);
  ASSERT_LT(std::chrono::steady_clock::now()-start,50ms);fullDwellAfterRevocation();
}

TEST_F(P1TwoClocks, ReceiveGapRevokesContinuityWhileSourceGapAndOriginalFreshnessRemainBounded) {
  establishReady();deliver(message());tick();ASSERT_TRUE(check());
  const auto previous=last_delivered_;rate(.2);const auto start=std::chrono::steady_clock::now();
  for (int i=1;i<=6;++i) {std::this_thread::sleep_until(start+i*50ms);tick();ASSERT_TRUE(check());}
  std::this_thread::sleep_until(start+315ms);deliver(message());ASSERT_FALSE(check());
  const auto span=rclcpp::Time(last_delivered_.header.stamp).nanoseconds()-rclcpp::Time(previous.header.stamp).nanoseconds();
  EXPECT_GT(span,0);EXPECT_LT(span,300000000);rate(1.);fullDwellAfterRevocation();
}

TEST_F(P1DeliveredStop, CancelResetDeactivateAndNewPlanCannotReuseReady) {
  establishReady();controller_->cancel();EXPECT_FALSE(check());
  deliver(message());tick();EXPECT_FALSE(check());
  controller_->reset();controller_->setPlan(plan(.02));checker_->reset();EXPECT_FALSE(check());
  // Same-goal reset retains its independent Track reentry .3s debt, before
  // the separate final .3s stop hold. A450ms whole-fixture budget cannot cover
  // both unchanged gates; allow a bounded850ms without relaxing either one.
  ASSERT_TRUE(healthyFor(850ms));ASSERT_TRUE(check());
  controller_->deactivate();EXPECT_FALSE(check());
}

TEST_F(P1DeliveredStop, AReservedRealSubscriptionCallbackRetainsItsGateAcrossCleanup) {
  establishReady();auto bad=message(0.,.08);publisher_->publish(bad);
  auto queued=reserveOdometry();auto original=std::move(controller_);
  original->deactivate();original->cleanup();
  // The executable retains the subscription and its captured gate, while the
  // controller object (and its Odom mutex) deliberately remains alive.
  const auto delivered=executor_.deliver(queued);
  EXPECT_EQ(delivered.header.stamp,bad.header.stamp);EXPECT_DOUBLE_EQ(delivered.twist.twist.angular.z,.08);
  EXPECT_FALSE(check());original.reset();
}

TEST_F(P1DeliveredStop, ReadyCommitCannotRegrantTheCycleRevokedAfterItsOriginalOdomRead) {
  establishReady();ASSERT_EQ(map_->getCostmap()->getSizeInCellsX(),600u);
  ASSERT_EQ(map_->getCostmap()->getSizeInCellsY(),600u);
  // ScopedObstacleIndex allocates its 76x76 uint32 prefix after measuredOdometry.
  // Pausing here preserves the real public compute path and its old Odom copy.
  p1_allocation_fixture::Barrier barrier(76u*76u*sizeof(std::uint32_t));
  const auto input=current_pose_;
  auto computed=std::async(std::launch::async,[&]{
    p1_allocation_fixture::TargetThread target;
    return controller_->computeVelocityCommands(input,geometry_msgs::msg::Twist{},checker_.get());
  });
  const bool injected=barrier.wait();
  if(injected) {
    auto pulse=last_delivered_;pulse.header.stamp=node_->now();pulse.twist.twist.angular.z=.08;deliver(pulse);
    auto stopped=pulse;stopped.header.stamp=node_->now();stopped.twist.twist.angular.z=0.;deliver(stopped);
  }
  barrier.release();geometry_msgs::msg::TwistStamped result;ASSERT_NO_THROW(result=computed.get());
  ASSERT_TRUE(injected);ASSERT_FALSE(p1_allocation_fixture::timed_out);
  EXPECT_DOUBLE_EQ(result.twist.linear.x,0.);EXPECT_DOUBLE_EQ(result.twist.angular.z,0.);
  EXPECT_FALSE(check());RecordProperty("real_allocation_pause_bytes",23104);
  fullDwellAfterRevocation();
}

class P1CheckerConsume : public P1DeliveredStop {
protected:
  std::string globalFrame() const override {return "p1_checker_frame_"+std::string(111,'x');}
};

TEST_F(P1CheckerConsume, CallbackAfterTokenSnapshotDefeatsTheFinalGoalCheckerConsume) {
  establishReady();freshTransform();const auto input=current_pose_;const auto goal=pose(0.);
  // Snapshot copies no string; the first frame-sized allocation is inside the
  // real latestBody/TF lookup, after the immutable completion token was taken.
  const auto bytes=globalFrame().size()+1;p1_allocation_fixture::Barrier barrier(bytes);
  auto checked=std::async(std::launch::async,[&]{
    p1_allocation_fixture::TargetThread target;
    return checker_->isGoalReached(input.pose,goal.pose,geometry_msgs::msg::Twist{});
  });
  const bool injected=barrier.wait();
  if(injected) {
    // Do not write TF while its real lookup is paused. Registered Odom callback
    // execution does not require that TF mutex, and all geometry stays fixed.
    auto pulse=last_delivered_;pulse.header.stamp=node_->now();pulse.twist.twist.angular.z=.08;deliver(pulse);
    auto stopped=pulse;stopped.header.stamp=node_->now();stopped.twist.twist.angular.z=0.;deliver(stopped);
  }
  barrier.release();const bool reached=checked.get();
  ASSERT_TRUE(injected);ASSERT_FALSE(p1_allocation_fixture::timed_out);EXPECT_FALSE(reached);
  EXPECT_FALSE(check());RecordProperty("real_frame_allocation_pause_bytes",static_cast<int>(bytes));
  fullDwellAfterRevocation();
}

TEST_F(P1DeliveredStop, InvalidOriginalStampCannotBeHiddenByAFreshStoppedCallback) {
  for (int invalid=0;invalid<3;++invalid) {
    establishReady();auto bad=message();
    if(invalid==0) {bad.header.stamp.sec=0;bad.header.stamp.nanosec=0;}
    if(invalid==1)bad.header.stamp=node_->now()+rclcpp::Duration::from_seconds(.06);
    if(invalid==2)bad.header.stamp=node_->now()-rclcpp::Duration::from_seconds(.31);
    deliver(bad);ASSERT_FALSE(check());deliver(message());fullDwellAfterRevocation();
  }
}

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
  testing::internal::CaptureStderr();
  try {command(.10);FAIL()<<"An off-center braking stop needs a fresh path";}
  catch (const nav2_core::NoValidControl & error) {
    EXPECT_NE(std::string(error.what()).find("fresh original-goal path"),std::string::npos);
  }
  const auto context=testing::internal::GetCapturedStderr();
  RecordProperty("braking_failure_context",context);
  for (const char * field:{"terminal_failure_context_v1 reason=braking_stopped_outside_center",
    "original_goal_xyz=", "path_endpoint_xyz=", "body_frame=", "path_body_frame=",
    "measured_original_stamp_ns=", "measured_linear=", "last_plugin_return_vx_vy_wz=",
    "handler_prefix_offset=", "handler_heading_next=", "terminal_braking=", "entry_xy_limit=",
    "original_xy_limit=", "in_entry_region=0", "in_original_region=1", "measured_stopped=1"}) {
    EXPECT_NE(context.find(field),std::string::npos)<<field<<'\n'<<context;
  }
  controller_->setPlan(turnPlan(.10));checker_->reset();
  auto repair=command(.10);EXPECT_GT(repair.twist.linear.x,0.);
}

TEST_F(StoppedTerminal, RotationExitRecordsCursorWithoutGrantingCompletion) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.3));checker_->reset();
  command(.14,.4);command(.04);
  testing::internal::CaptureStderr();
  EXPECT_THROW(command(.16),nav2_core::NoValidControl);
  const auto context=testing::internal::GetCapturedStderr();
  RecordProperty("rotation_failure_context",context);
  for (const char * field:{"terminal_failure_context_v1 reason=rotation_left_original_region",
    "path_geometry_digest=", "binding_generation=", "body_xyz=", "original_goal_xyz=",
    "handler_constraint_locale=", "handler_rotation_active=", "terminal_heading_next=",
    "terminal_heading_count=", "terminal_armed=1", "in_original_region=0",
    "command_boundary=plugin_return_not_downstream_measured_command"}) {
    EXPECT_NE(context.find(field),std::string::npos)<<field<<'\n'<<context;
  }
  EXPECT_FALSE(reached(.16));
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

class FastOnlyBraking : public StoppedTerminal {
protected:
  void configureExtra() override {
    StoppedTerminal::configureExtra();
    node_->declare_parameter("FollowPath.terminal_early_braking_min_linear_velocity",.30);
    node_->declare_parameter("FollowPath.rpp.approach_velocity_scaling_dist",.3);
  }
};
class FastOnlyB34Geometry : public FastOnlyBraking {
protected:
  double mapOrigin() const override {return 1.;}
};

TEST_F(FastOnlyBraking, OuterWindowLowMeasuredSpeedRetainsOriginalForwardFloor) {
  publisher_->on_activate();
  // B35 recorded low drift speeds must retain the unchanged .4 m/s floor;
  // they are above final-stop acceptance but below the separate early brake.
  for (double speed:{.0318,.055,.10,.20,.30}) {
    SCOPED_TRACE(speed);
    controller_->reset();controller_->setPlan(turnPlan(.3));checker_->reset();
    auto cmd=command(.14,speed);
    // Original .4 desired/regulated/approach floors are retained. This is a
    // native command assertion, not evidence that the simulator has moved.
    EXPECT_DOUBLE_EQ(cmd.twist.linear.x,.4);EXPECT_NEAR(cmd.twist.angular.z,0.,1e-8);
  }
  controller_->reset();controller_->setPlan(turnPlan(.3));checker_->reset();
  auto fast=command(.14,.31);
  EXPECT_DOUBLE_EQ(fast.twist.linear.x,0.);EXPECT_DOUBLE_EQ(fast.twist.angular.z,0.);
  controller_->reset();controller_->setPlan(turnPlan(.3));checker_->reset();
  // The gate uses the measured planar norm, not its forward component alone.
  auto lateral=commandAt(.14,0.,M_PI,.20,.24);
  EXPECT_DOUBLE_EQ(lateral.twist.linear.x,0.);EXPECT_DOUBLE_EQ(lateral.twist.angular.z,0.);
}

TEST_F(FastOnlyB34Geometry, RecordedFastApproachesRetainCheckedEarlyZero) {
  publisher_->on_activate();
  for (const auto & recorded:b34TerminalApproaches()) {
    SCOPED_TRACE(recorded.plan_header_ns);
    nav_msgs::msg::Path p;p.header.frame_id="map";p.header.stamp=node_->now();
    for (const auto & row:recorded.wire_poses) {
      auto x=pose(row[0],row[2]);x.pose.position.y=row[1];x.header=p.header;p.poses.push_back(x);
    }
    // Same cached geometry/Native velocity evidence and separate fixture clock
    // as B20. Planned-anchor + recorded twist remains a controlled combination.
    for (const auto & actual:{recorded.planned_track_start,recorded.native_pose}) {
      controller_->reset();controller_->setPlan(p);checker_->reset();
      auto cmd=commandAt(actual[0],actual[1],actual[2],recorded.body_twist[0],
        recorded.body_twist[1],recorded.body_twist[2]);
      EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0.);
    }
  }
}

TEST_F(FastOnlyBraking, CenteredFirstTurnStillNeedsOriginalFullMeasuredStop) {
  publisher_->on_activate();controller_->setPlan(turnPlan(.3));checker_->reset();
  auto moving=command(.04,.10);
  EXPECT_DOUBLE_EQ(moving.twist.linear.x,0.);EXPECT_DOUBLE_EQ(moving.twist.angular.z,0.);
  auto rotating=commandAt(.04,0.,M_PI,.02,0.,.10);
  EXPECT_DOUBLE_EQ(rotating.twist.linear.x,0.);EXPECT_DOUBLE_EQ(rotating.twist.angular.z,0.);
  auto stopped=command(.04);
  EXPECT_DOUBLE_EQ(stopped.twist.linear.x,0.);EXPECT_GT(std::abs(stopped.twist.angular.z),0.);
  auto active=command(.10);
  EXPECT_DOUBLE_EQ(active.twist.linear.x,0.);EXPECT_GT(std::abs(active.twist.angular.z),0.);
}


// P2 software protocol cases use the real plugin and its real delivered DDS
// callback. They do not replay Source17 or infer a physical stopping envelope.
class P2DeliveredReentry : public P1DeliveredStop {};
TEST_F(P2DeliveredReentry, ContinuousFreshReentrySamplesRemainUsableUntilReady) {
  establishReady();controller_->reset();controller_->setPlan(plan(.02));checker_->reset();
  ASSERT_TRUE(healthyFor(950ms));ASSERT_TRUE(check());
  std::ifstream maps("/proc/self/maps");std::string line,mapping;
  while(std::getline(maps,line))if(line.find("libnav2_regulated_pure_pursuit_controller.so")!=std::string::npos)mapping=line;
  ASSERT_FALSE(mapping.empty());RecordProperty("actual_rpp_mapping",mapping);
}
TEST_F(P2DeliveredReentry, SameTickPulseCannotConsumeReentryAndHealthyOriginalDwellRecovers) {
  establishReady();controller_->reset();controller_->setPlan(plan(.02));checker_->reset();
  deliver(message());tick();ASSERT_FALSE(check());
  healthyFor(275ms,true);
  const auto until=std::chrono::steady_clock::now()+80ms;
  while(std::chrono::steady_clock::now()<until){deliver(message());std::this_thread::sleep_for(2ms);}
  p1_allocation_fixture::Barrier barrier(76u*76u*sizeof(std::uint32_t));const auto input=current_pose_;
  auto computed=std::async(std::launch::async,[&]{p1_allocation_fixture::TargetThread target;
    return controller_->computeVelocityCommands(input,geometry_msgs::msg::Twist{},checker_.get());});
  const bool injected=barrier.wait();const auto before=callbacks_;
  if(injected){deliver(message(0.,.08));deliver(message());}
  barrier.release();geometry_msgs::msg::TwistStamped value;ASSERT_NO_THROW(value=computed.get());
  ASSERT_TRUE(injected);ASSERT_FALSE(p1_allocation_fixture::timed_out);ASSERT_EQ(callbacks_-before,2u);
  EXPECT_DOUBLE_EQ(value.twist.linear.x,0.);EXPECT_DOUBLE_EQ(value.twist.angular.z,0.);EXPECT_FALSE(check());
  healthyFor(275ms,true);ASSERT_TRUE(healthyFor(750ms));ASSERT_TRUE(check());
  RecordProperty("delivered_pulse_and_stop_callbacks",2);
}
class P2OriginalPivot : public StoppedTerminal {
protected:
  bool initialShortcut() const override {return true;}
  void configureExtra() override {StoppedTerminal::configureExtra();
    node_->declare_parameter("FollowPath.terminal_original_region_pivot",true);}
  void completed() {publisher_->on_activate();controller_->setPlan(plan(.02));checker_->reset();
    auto value=command(.02);ASSERT_DOUBLE_EQ(value.twist.linear.x,0.);}
};
TEST_F(P2OriginalPivot, CompletedOriginalCursorCanStopAtElevenCentimetresAndSixHundredthsYaw) {
  completed();bool ready=false;
  for(int i=0;i<12&&!ready;++i){auto value=commandAt(.11,0.,M_PI-.06);
    EXPECT_DOUBLE_EQ(value.twist.linear.x,0.);EXPECT_DOUBLE_EQ(value.twist.angular.z,0.);
    auto current=inputAt(.11,0.,M_PI-.06);ready=checker_->isGoalReached(current.pose,pose(0.).pose,geometry_msgs::msg::Twist{});
    std::this_thread::sleep_for(50ms);}
  ASSERT_TRUE(ready);RecordProperty("original_xy",.11);RecordProperty("original_yaw_error",.06);
  auto fresh=plan(.3); // Same original goal, valid new forward-path generation.
  controller_->setPlan(fresh);checker_->reset();
  auto current=inputAt(.11,0.,M_PI-.06);EXPECT_FALSE(checker_->isGoalReached(current.pose,pose(0.).pose,geometry_msgs::msg::Twist{}));
}
TEST_F(P2OriginalPivot, ZeroAimStillSweepsTheWholeFreshCompleteCursorRotation) {
  completed();ASSERT_NO_THROW(commandAt(.11,0.,M_PI-.06));
  unsigned int mx,my;ASSERT_TRUE(map_->getCostmap()->worldToMap(.90,.67,mx,my));
  map_->getCostmap()->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
  try{commandAt(.11,0.,M_PI-.06);FAIL()<<"Fresh original-goal sweep crossed a lethal cell";}
  catch(const nav2_core::NoValidControl & error){EXPECT_NE(std::string(error.what()).find("complete-cursor goal rotation"),std::string::npos)<<error.what();}
  EXPECT_FALSE(reached(.11));
}
class P2UnfinishedPivot : public P2OriginalPivot {
protected:bool initialShortcut() const override {return false;}
};
TEST_F(P2UnfinishedPivot, NearOriginalGoalCannotSkipUnfinishedMandatoryHeadings) {
  publisher_->on_activate();auto p=plan(.3);p.poses.insert(p.poses.end()-1,pose(0.,M_PI/2.));
  p.poses.insert(p.poses.end()-1,pose(0.,M_PI)); // Retain original yaw in the executable prefix.
  controller_->setPlan(p);checker_->reset();auto value=commandAt(.11,0.,M_PI-.06);
  EXPECT_GT(value.twist.linear.x,0.);auto current=inputAt(.11,0.,M_PI-.06);
  EXPECT_FALSE(checker_->isGoalReached(current.pose,pose(0.).pose,geometry_msgs::msg::Twist{}));
}
class P2InnerAim : public P2OriginalPivot {
protected:void configureExtra() override {P2OriginalPivot::configureExtra();
  node_->declare_parameter("FollowPath.terminal_rotation_aim_yaw_tolerance",.03);}
};
TEST_F(P2InnerAim, CrossingAimRetainsBrakingDebtUntilAValidStoppedNewPlan) {
  publisher_->on_activate();const auto original=plan(.11);controller_->setPlan(original);checker_->reset();
  auto before=commandAt(.11,0.,M_PI-.06);ASSERT_GT(before.twist.angular.z,0.);
  auto crossed=commandAt(.11,0.,M_PI+.04,0.,0.,.08);
  ASSERT_DOUBLE_EQ(crossed.twist.linear.x,0.);ASSERT_DOUBLE_EQ(crossed.twist.angular.z,0.);
  EXPECT_THROW(controller_->setPlan(original),nav2_core::InvalidPath);
  for(int i=0;i<2;++i){EXPECT_THROW(commandAt(.11,0.,M_PI+.04),nav2_core::NoValidControl);}
  nav_msgs::msg::Path invalid;EXPECT_ANY_THROW(controller_->setPlan(invalid));
  EXPECT_ANY_THROW(commandAt(.11,0.,M_PI+.04));
  ASSERT_NO_THROW(controller_->setPlan(original));checker_->reset();bool rotated=false;
  for(int i=0;i<20&&!rotated;++i){geometry_msgs::msg::TwistStamped value;
    ASSERT_NO_THROW(value=commandAt(.11,0.,M_PI+.04));EXPECT_DOUBLE_EQ(value.twist.linear.x,0.);
    if(value.twist.angular.z!=0.){EXPECT_LT(value.twist.angular.z,0.);rotated=true;}
    std::this_thread::sleep_for(50ms);}
  ASSERT_TRUE(rotated);EXPECT_FALSE(reached(.11));
}


class P5StableTerminal : public P1DeliveredStop {
protected:
  bool stoppedEntry() const override {return true;}
  virtual double phaseSeconds() const {return 10.;}
  void configureExtra() override {
    node_->declare_parameter("FollowPath.terminal_yaw_stability_window_sec",1.);
    node_->declare_parameter("FollowPath.terminal_yaw_stability_max_span_rad",.005);
    node_->declare_parameter("FollowPath.terminal_settle_timeout_sec",phaseSeconds());
  }
  double heading_=M_PI;
  nav_msgs::msg::Odometry p5Message(double angular=0.) {
    current_pose_=pose(.02,heading_);current_pose_.header.frame_id=globalFrame();current_pose_.header.stamp=node_->now();
    geometry_msgs::msg::TransformStamped t;t.header=current_pose_.header;t.child_frame_id="base_link";
    t.transform.translation.x=.02;t.transform.rotation=current_pose_.pose.orientation;
    EXPECT_TRUE(map_->getTfBuffer()->setTransform(t,"p5_original_pose_fixture",false));
    nav_msgs::msg::Odometry o;o.header=current_pose_.header;o.child_frame_id="base_link";o.pose.pose=current_pose_.pose;o.twist.twist.angular.z=angular;
    return o;
  }
  geometry_msgs::msg::TwistStamped p5Tick() {return controller_->computeVelocityCommands(current_pose_,geometry_msgs::msg::Twist{},checker_.get());}
  bool p5Check() {return checker_->isGoalReached(current_pose_.pose,pose(0.).pose,geometry_msgs::msg::Twist{});}
  bool p5Run(std::chrono::milliseconds duration,bool forbid_ready=false,double slope=0.,double angular=0.,bool control=true) {
    const auto start=std::chrono::steady_clock::now(),end=start+duration;auto next_tick=start,next_sample=start;
    const auto initial=heading_;bool ready=false;
    while(std::chrono::steady_clock::now()<end){
      heading_=initial+slope*std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      deliver(p5Message(angular));const auto now=std::chrono::steady_clock::now();
      if(control&&now>=next_tick){const auto cmd=p5Tick();EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0.);
        ready=p5Check();if(forbid_ready)EXPECT_FALSE(ready);next_tick=now+50ms;}
      next_sample+=2ms;std::this_thread::sleep_until(next_sample);
    }
    return ready;
  }
};
TEST_F(P5StableTerminal, LowTwistSlowOriginalPoseDriftCannotReachReady) {
  ASSERT_FALSE(p5Run(1400ms,true,.014,.014));EXPECT_FALSE(p5Check());
  RecordProperty("original_pose_yaw_rate_fixture",std::to_string(.014));RecordProperty("original_twist_wz",std::to_string(.014));
}
TEST_F(P5StableTerminal, ZeroPublishedTwistDoesNotHideSlowOriginalPoseDrift) {
  ASSERT_FALSE(p5Run(1400ms,true,.014,0.));EXPECT_FALSE(p5Check());
  RecordProperty("original_pose_yaw_rate_fixture",std::to_string(.014));RecordProperty("original_twist_wz",std::to_string(0.));
}
TEST_F(P5StableTerminal, HealthyFullPostZeroWindowCompletesAndBadPoseOrFrameRevokesIt) {
  // A full pre-zero history cannot substitute for the selected zero's window.
  p5Run(1100ms,false,0.,0.,false);EXPECT_FALSE(p5Check());
  const auto first_zero=std::chrono::steady_clock::now();p5Run(950ms,true);ASSERT_TRUE(p5Run(450ms));
  EXPECT_GE(std::chrono::steady_clock::now()-first_zero,1s);
  for(int fault=0;fault<2;++fault){auto bad=last_delivered_;bad.header.stamp=node_->now();
    if(fault==0)bad.pose.pose.position.z=std::numeric_limits<double>::quiet_NaN();else bad.child_frame_id="changed_original_body_frame";
    deliver(bad);EXPECT_FALSE(p5Check());deliver(p5Message());EXPECT_FALSE(p5Check());
    p5Run(950ms,true);ASSERT_TRUE(p5Run(450ms));}
  RecordProperty("registered_dds_callbacks",static_cast<int>(callbacks_));
}
class P5StableConsume : public P5StableTerminal {
protected:std::string globalFrame() const override {return "p5_stable_consume_"+std::string(111,'x');}
};
TEST_F(P5StableConsume, NewPoseBeyondCertificateSpanDefeatsAnAlreadySnapshottedGoalCheckerToken) {
  ASSERT_TRUE(p5Run(1400ms));ASSERT_TRUE(p5Check());const auto input=current_pose_;const auto goal=pose(0.);
  const auto bytes=globalFrame().size()+1;p1_allocation_fixture::Barrier barrier(bytes);
  auto checked=std::async(std::launch::async,[&]{p1_allocation_fixture::TargetThread target;
    return checker_->isGoalReached(input.pose,goal.pose,geometry_msgs::msg::Twist{});});
  const bool injected=barrier.wait();const auto before=callbacks_;
  if(injected){auto changed=last_delivered_;changed.header.stamp=node_->now();
    changed.pose.pose.orientation.z=std::sin((M_PI+.006)/2.);changed.pose.pose.orientation.w=std::cos((M_PI+.006)/2.);deliver(changed);}
  barrier.release();const bool value=checked.get();ASSERT_TRUE(injected);ASSERT_FALSE(p1_allocation_fixture::timed_out);
  ASSERT_EQ(callbacks_-before,1u);EXPECT_FALSE(value);EXPECT_FALSE(p5Check());
  heading_=M_PI+.006;p5Run(950ms,true);ASSERT_TRUE(p5Run(450ms));
  RecordProperty("pose_span_crossing_delivered_callbacks",1);RecordProperty("real_frame_pause_bytes",static_cast<int>(bytes));
}
TEST_F(P5StableTerminal, ASelectedNonzeroCorrectionRequiresANewFullWindow) {
  p5Run(650ms,true);heading_=M_PI+.09;deliver(p5Message());
  // CorrectYaw selects one checked zero while installing the correction cursor.
  const auto prepared=p5Tick();ASSERT_DOUBLE_EQ(prepared.twist.linear.x,0.);ASSERT_DOUBLE_EQ(prepared.twist.angular.z,0.);
  EXPECT_FALSE(p5Check());const auto next_tick=std::chrono::steady_clock::now()+50ms;
  while(std::chrono::steady_clock::now()<next_tick){deliver(p5Message());std::this_thread::sleep_for(2ms);}
  const auto correction=p5Tick();ASSERT_NE(correction.twist.angular.z,0.);EXPECT_DOUBLE_EQ(correction.twist.linear.x,0.);EXPECT_FALSE(p5Check());
  heading_=M_PI+.02;const auto restarted=std::chrono::steady_clock::now();p5Run(950ms,true);ASSERT_TRUE(p5Run(450ms));
  EXPECT_GE(std::chrono::steady_clock::now()-restarted,1s);
}
class P5ShortPhase : public P5StableTerminal {
protected:double phaseSeconds() const override {return 1.6;}
};
TEST_F(P5ShortPhase, ValidAndInvalidSameGoalPlansCannotRenewTheOriginalSettlingPhase) {
  const auto first=std::chrono::steady_clock::now();p5Run(500ms,true);
  ASSERT_NO_THROW(controller_->setPlan(plan(.02)));checker_->reset();
  nav_msgs::msg::Path invalid;EXPECT_ANY_THROW(controller_->setPlan(invalid));
  ASSERT_NO_THROW(controller_->setPlan(plan(.02)));checker_->reset();
  const auto end=first+1900ms;auto next_tick=std::chrono::steady_clock::now();bool expired=false;
  while(std::chrono::steady_clock::now()<end&&!expired){deliver(p5Message());
    if(std::chrono::steady_clock::now()>=next_tick){
      try{const auto cmd=p5Tick();EXPECT_DOUBLE_EQ(cmd.twist.linear.x,0.);EXPECT_DOUBLE_EQ(cmd.twist.angular.z,0.);EXPECT_FALSE(p5Check());}
      catch(const nav2_core::NoValidControl & error){EXPECT_NE(std::string(error.what()).find("fixed phase deadline"),std::string::npos)<<error.what();expired=true;}
      next_tick=std::chrono::steady_clock::now()+50ms;}
    std::this_thread::sleep_for(2ms);}
  ASSERT_TRUE(expired);EXPECT_FALSE(p5Check());RecordProperty("phase_elapsed_ms",static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-first).count()));
}

class P7InnerAimStableTerminal : public P5StableTerminal {
protected:
  void configureExtra() override {
    P5StableTerminal::configureExtra();
    node_->declare_parameter("FollowPath.terminal_original_region_pivot",true);
    node_->declare_parameter("FollowPath.terminal_rotation_aim_yaw_tolerance",.03);
    node_->declare_parameter("FollowPath.rpp.rotate_to_heading_angular_vel",.3);
    node_->declare_parameter("FollowPath.rpp.max_angular_accel",6.);
  }
  bool originalReached() {
    return checker_->isGoalReached(current_pose_.pose,pose(0.,0.).pose,geometry_msgs::msg::Twist{});
  }
};
TEST_F(P7InnerAimStableTerminal, RecordedOuterResidualRequiresInnerAimAndANewFullStableWindow) {
  // Same public plugin/callback path and installed DSO as P5. This is a finite
  // protocol fixture, not a claim that a plant follows this supplied pose tail.
  controller_->reset();auto original=plan(-.02);
  for(auto & point:original.poses)point.pose.orientation=pose(0.,0.).pose.orientation;
  controller_->setPlan(original);checker_->reset();heading_=.077;
  deliver(p5Message());const auto correction=p5Tick();
  ASSERT_DOUBLE_EQ(correction.twist.linear.x,0.);
  ASSERT_LT(correction.twist.angular.z,0.);ASSERT_LE(std::abs(correction.twist.angular.z),.3);
  ASSERT_FALSE(originalReached());
  RecordProperty("outer_residual_rad",.077);RecordProperty("outer_correction_wz",correction.twist.angular.z);
  heading_=.02;deliver(p5Message());const auto first_zero=p5Tick();
  ASSERT_DOUBLE_EQ(first_zero.twist.linear.x,0.);ASSERT_DOUBLE_EQ(first_zero.twist.angular.z,0.);
  ASSERT_FALSE(originalReached());const auto zero_time=std::chrono::steady_clock::now();
  auto next_tick=zero_time,next_sample=zero_time;bool ready=false;
  const auto end=zero_time+1400ms;
  while(std::chrono::steady_clock::now()<end){
    deliver(p5Message());const auto now=std::chrono::steady_clock::now();
    if(now>=next_tick){const auto value=p5Tick();
      ASSERT_DOUBLE_EQ(value.twist.linear.x,0.);ASSERT_DOUBLE_EQ(value.twist.angular.z,0.);
      ready=originalReached();if(now-zero_time<1s)ASSERT_FALSE(ready);
      if(ready)break;next_tick=now+50ms;
    }
    next_sample+=2ms;std::this_thread::sleep_until(next_sample);
  }
  ASSERT_TRUE(ready);ASSERT_GE(std::chrono::steady_clock::now()-zero_time,1s);
  ASSERT_DOUBLE_EQ(node_->get_parameter("goal_checker.yaw_goal_tolerance").as_double(),.08);
  ASSERT_DOUBLE_EQ(node_->get_parameter("FollowPath.terminal_settle_timeout_sec").as_double(),10.);
  std::ifstream maps("/proc/self/maps");std::string line,adapter,rpp;
  while(std::getline(maps,line)){
    if(line.find("libofficial_lattice_rpp_adapter.so")!=std::string::npos)adapter=line;
    if(line.find("libnav2_regulated_pure_pursuit_controller.so")!=std::string::npos)rpp=line;
  }
  ASSERT_FALSE(adapter.empty());ASSERT_FALSE(rpp.empty());
  RecordProperty("actual_adapter_mapping",adapter);RecordProperty("actual_rpp_mapping",rpp);
  RecordProperty("registered_dds_callbacks",static_cast<int>(callbacks_));
  RecordProperty("post_zero_ready_ms",static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-zero_time).count()));
}
}  // namespace
