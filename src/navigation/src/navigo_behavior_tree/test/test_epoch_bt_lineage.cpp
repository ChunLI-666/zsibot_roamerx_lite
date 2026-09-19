// Integration contract tests for the actual dynamically loaded BT plugin.
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_epoch_msgs/msg/epoch_path.hpp"
#include "navigo_epoch_msgs/action/follow_path_epoch.hpp"
#include "navigo_epoch_msgs/action/back_up_epoch.hpp"
#include "navigo_epoch_msgs/msg/localization_epoch.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "navigo_util/simple_action_server.hpp"

using namespace std::chrono_literals;
using Action = nav2_msgs::action::ComputePathToPose;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using State = navigo_epoch_msgs::msg::LocalizationEpoch;
using Path = navigo_epoch_msgs::msg::EpochPath;

class EpochBT : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    static int serial = 0;
    mock_ = std::make_shared<rclcpp::Node>("epoch_bt_test_server_" + std::to_string(++serial));
    client_ = std::make_shared<rclcpp::Node>("epoch_bt_test_client_" + std::to_string(serial));
    state_pub_ = mock_->create_publisher<State>("/lightning/localization_epoch", 1);
    gate_pub_ = mock_->create_publisher<std_msgs::msg::String>("/nav_epoch/gate_session", 1);
    server_ = rclcpp_action::create_server<Action>(mock_, "compute_path_to_pose",
      [this](const auto &, const auto) {
        goal_seen_ = true;
        while (delay_ack_ && !stopping_) {std::this_thread::sleep_for(1ms);}
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto) {++cancels_; return rclcpp_action::CancelResponse::REJECT;},
      [this](std::shared_ptr<Handle> handle) {std::lock_guard<std::mutex> lock(mutex_); goals_.push_back(handle);});
    executor_.add_node(mock_);
    server_thread_ = std::thread([this] {executor_.spin();});
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", client_);
    blackboard_->set("epoch_goal_revision", uint64_t(1));
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map"; goal.pose.orientation.w = 1; goal.pose.position.x = 2;
    blackboard_->set("goal", goal);
    factory_.registerFromPlugin(EPOCH_PLUGIN_PATH);
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
      "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
      "<EpochGuard><EpochComputePathToPose goal='{goal}' epoch_path='{epoch_raw_path}'/>"
      "</EpochGuard></BehaviorTree></root>", blackboard_));
    state_.schema_version = 1; state_.identity.process_session_id = "loc-test-session";
    state_.identity.map_loaded_instance = "fixed-test-map";
    state_.identity.epoch = 1; state_.identity.commits = 1;
    state_.lifecycle_enabled = true; state_.health = State::NORMAL;
    state_.fusion_ready = state_.output_ready = true;
    state_.phase = "TRACKING"; state_.base_frame_id = "base_link";
    state_.output_pose.header.frame_id = "map"; state_.output_pose.pose.orientation.w = 1;
    state_.output_pose.pose.position.x = 0.25;
    gate_ = "gate-a";
  }
  void TearDown() override
  {
    stopping_ = true; delay_ack_ = false;
    if (tree_) {tree_->haltTree();}
    executor_.cancel();
    if (server_thread_.joinable()) {server_thread_.join();}
    tree_.reset(); blackboard_.reset();
    executor_.remove_node(mock_);
    server_.reset(); state_pub_.reset(); gate_pub_.reset(); client_.reset(); mock_.reset();
  }
  BT::NodeStatus tick(bool send = true, bool advance_stamp = true)
  {
    if (send) {
      ++state_.heartbeat_sequence;
      if (advance_stamp) {state_.output_pose.header.stamp = mock_->now();}
      state_pub_->publish(state_);
      std_msgs::msg::String gate; gate.data = gate_; gate_pub_->publish(gate);
    }
    const auto tick_started=std::chrono::steady_clock::now();
    const auto result = tree_->tickRoot();
    maximum_tick_us_=std::max(maximum_tick_us_,std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-tick_started).count());
    std::this_thread::sleep_for(10ms);
    return result;
  }
  template<class Predicate> bool until(Predicate predicate, double seconds = 4)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds) {
      tick(); if (predicate()) {return true;}
    }
    return false;
  }
  size_t count() {std::lock_guard<std::mutex> lock(mutex_); return goals_.size();}
  std::shared_ptr<Handle> goal(size_t i) {std::lock_guard<std::mutex> lock(mutex_); return goals_.at(i);}
  void succeed(size_t i)
  {
    auto handle = goal(i); auto result = std::make_shared<Action::Result>();
    result->path.header = handle->get_goal()->start.header;
    result->path.poses = {handle->get_goal()->start, handle->get_goal()->goal};
    handle->succeed(result);
  }
  Path output()
  {
    Path result; blackboard_->get("epoch_raw_path", result); return result;
  }
  rclcpp::Node::SharedPtr mock_, client_;
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gate_pub_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread server_thread_;
  std::mutex mutex_;
  std::vector<std::shared_ptr<Handle>> goals_;
  std::atomic<bool> delay_ack_{false}, goal_seen_{false}, stopping_{false};
  std::atomic<int> cancels_{0};
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> tree_;
  State state_;
  std::string gate_;
  double maximum_tick_us_{0.};
};

TEST_F(EpochBT, PlannerUsesTaggedStartAndResultRetainsCapturedIdentity)
{
  ASSERT_TRUE(until([&] {return count() == 1;}));
  EXPECT_TRUE(goal(0)->get_goal()->use_start);
  EXPECT_DOUBLE_EQ(goal(0)->get_goal()->start.pose.position.x, 0.25);
  const auto captured = goal(0)->get_goal()->start.header.stamp;
  succeed(0);
  ASSERT_TRUE(until([&] {return !output().path.poses.empty();}));
  EXPECT_EQ(output().token.localization.epoch, 1u);
  EXPECT_EQ(output().planning_start.header.stamp, captured);
  EXPECT_EQ(output().token.gate_session_id, "gate-a");
}

TEST_F(EpochBT, DelayedOldResultCannotBecomeNewEpochPath)
{
  ASSERT_TRUE(until([&] {return count() == 1;}));
  state_.health = State::LOST; state_.output_ready = false;
  for (int i = 0; i < 8; ++i) {tick();}
  state_.identity.epoch = 2; state_.identity.commits = 2;
  state_.health = State::NORMAL; state_.output_ready = true;
  ASSERT_TRUE(until([&] {return count() >= 2;}));
  succeed(0);
  for (int i = 0; i < 8; ++i) {tick();}
  EXPECT_TRUE(output().path.poses.empty());
  succeed(1);
  ASSERT_TRUE(until([&] {return !output().path.poses.empty();}));
  EXPECT_EQ(output().token.localization.epoch, 2u);
  EXPECT_GE(cancels_.load(), 1);
}

TEST_F(EpochBT, SameGeometryPreemptRequiresNewTaskRevision)
{
  ASSERT_TRUE(until([&] {return count() == 1;}));
  blackboard_->set("epoch_goal_revision", uint64_t(2));
  ASSERT_TRUE(until([&] {return count() >= 2;}));
  succeed(0);
  for (int i = 0; i < 5; ++i) {tick();}
  EXPECT_TRUE(output().path.poses.empty());
  succeed(1);
  ASSERT_TRUE(until([&] {return !output().path.poses.empty();}));
  EXPECT_EQ(output().token.task_sequence, 2u);
}

TEST_F(EpochBT, GateRestartInvalidatesPendingPlanAndLateAcknowledgementIsCancelled)
{
  delay_ack_ = true;
  ASSERT_TRUE(until([&] {return goal_seen_.load();}));
  gate_ = "gate-b";
  for (int i = 0; i < 10; ++i) {tick();}
  delay_ack_ = false;
  ASSERT_TRUE(until([&] {return count() >= 2 && cancels_.load() >= 1;}));
  succeed(0);
  for (int i = 0; i < 5; ++i) {tick();}
  EXPECT_TRUE(output().path.poses.empty());
  succeed(1);
  ASSERT_TRUE(until([&] {return !output().path.poses.empty();}));
  EXPECT_EQ(output().token.gate_session_id, "gate-b");
}

TEST_F(EpochBT, RepeatedOutputStampCannotStayReadyThroughFreshHeartbeats)
{
  ASSERT_TRUE(until([&] {return count() == 1;}));
  for (int i = 0; i < 60; ++i) {tick(true, false);}
  succeed(0);
  for (int i = 0; i < 10; ++i) {tick(true, false);}
  EXPECT_TRUE(output().path.poses.empty());
  EXPECT_EQ(count(), 1u);
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::RUNNING);
}

TEST_F(EpochBT, EmptyNotReadyPoseCannotWashOldSourceFresh)
{
  ASSERT_TRUE(until([&] {return count() == 1;}));
  const auto original = state_.output_pose;
  state_.output_ready = false;
  state_.output_pose = geometry_msgs::msg::PoseStamped();
  for (int i = 0; i < 50; ++i) {tick(true, false);}
  state_.output_ready = true; state_.output_pose = original;
  for (int i = 0; i < 10; ++i) {tick(true, false);}
  EXPECT_EQ(count(), 1u);
  // A genuinely advancing output may recover in the same localization epoch.
  ASSERT_TRUE(until([&] {return count() >= 2;}));
}

TEST_F(EpochBT, LateAcknowledgementIsCancelledWhileStillLost)
{
  delay_ack_ = true;
  ASSERT_TRUE(until([&] {return goal_seen_.load();}));
  state_.health = State::LOST; state_.output_ready = false;
  for (int i = 0; i < 10; ++i) {tick();}
  delay_ack_ = false;
  ASSERT_TRUE(until([&] {return cancels_.load() >= 1;}));
  EXPECT_EQ(count(), 1u);
  EXPECT_TRUE(output().path.poses.empty());
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::RUNNING);
}

TEST_F(EpochBT, NewerEmptyReloadRevocationIsImmediateWithoutLoweringWatermark)
{
  ASSERT_TRUE(until([&] {return count() == 1;}));
  state_.health = State::LOST; state_.output_ready = false;
  state_.identity.epoch = state_.identity.commits = 0;
  state_.identity.map_loaded_instance.clear();
  for (int i = 0; i < 8; ++i) {tick();}
  EXPECT_GE(cancels_.load(), 1);
  succeed(0);
  for (int i = 0; i < 5; ++i) {tick();}
  EXPECT_TRUE(output().path.poses.empty());
}

TEST_F(EpochBT, ThroughPosesPrunesOnlyReachedPrefixUsingTaggedPoseAndKeepsFinalGoal)
{
  using Through = nav2_msgs::action::ComputePathThroughPoses;
  auto received = std::make_shared<std::vector<Through::Goal>>();
  auto received_mutex = std::make_shared<std::mutex>();
  auto server = rclcpp_action::create_server<Through>(mock_, "compute_path_through_poses",
    [](const auto &, const auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](const auto) {return rclcpp_action::CancelResponse::ACCEPT;},
    [received, received_mutex](auto handle) {
      {
        std::lock_guard<std::mutex> lock(*received_mutex);
        received->push_back(*handle->get_goal());
      }
      auto result = std::make_shared<Through::Result>();
      result->path.header = handle->get_goal()->start.header;
      result->path.poses = handle->get_goal()->goals;
      handle->succeed(result);
    });
  tree_->haltTree();
  std::vector<geometry_msgs::msg::PoseStamped> goals(3);
  for (auto & pose : goals) {pose.header.frame_id = "map"; pose.pose.orientation.w = 1;}
  goals[0].pose.position.x = .25; goals[1].pose.position.x = 1.25; goals[2].pose.position.x = 3.;
  blackboard_->set("goals", goals);
  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<EpochGuard><EpochComputePathThroughPoses goals='{goals}' epoch_path='{epoch_raw_path}'/>"
    "</EpochGuard></BehaviorTree></root>", blackboard_));
  auto has_last = [&](double x) {
      std::lock_guard<std::mutex> lock(*received_mutex);
      return !received->empty() && received->back().start.pose.position.x == x;
    };
  ASSERT_TRUE(until([&] {return has_last(.25);}));
  {
    std::lock_guard<std::mutex> lock(*received_mutex);
    ASSERT_EQ(received->front().goals.size(), 2u);
    EXPECT_DOUBLE_EQ(received->front().goals.front().pose.position.x, 1.25);
  }
  state_.output_pose.pose.position.x = 1.25;
  ASSERT_TRUE(until([&] {return has_last(1.25);}));
  state_.output_pose.pose.position.x = 3.;
  ASSERT_TRUE(until([&] {return has_last(3.);}));
  {
    std::lock_guard<std::mutex> lock(*received_mutex);
    ASSERT_EQ(received->back().goals.size(), 1u);
    EXPECT_DOUBLE_EQ(received->back().goals.front().pose.position.x, 3.);
  }
  EXPECT_EQ(blackboard_->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals").size(), 1u);
}

TEST_F(EpochBT, TerminalFollowFailureWinsOverSimultaneousReplacement)
{
  using FollowAction = navigo_epoch_msgs::action::FollowPathEpoch;
  using FollowHandle = rclcpp_action::ServerGoalHandle<FollowAction>;
  std::shared_ptr<FollowHandle> follow_handle;
  std::atomic<int> follow_requests{0};
  auto follow_server = rclcpp_action::create_server<FollowAction>(mock_, "follow_path_epoch",
    [](const auto &, const auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](const auto) {return rclcpp_action::CancelResponse::ACCEPT;},
    [&](std::shared_ptr<FollowHandle> handle) {
      std::lock_guard<std::mutex> lock(mutex_);follow_handle=handle;++follow_requests;
    });
  tree_->haltTree();tree_.reset();
  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><EpochGuard><Sequence>"
    "<EpochComputePathToPose goal='{goal}' epoch_path='{epoch_raw_path}'/>"
    "<EpochFollowPath epoch_path='{epoch_raw_path}'/>"
    "</Sequence></EpochGuard></BehaviorTree></root>",blackboard_));
  ASSERT_TRUE(until([&] {return count()==1;}));succeed(0);
  ASSERT_TRUE(until([&] {return follow_requests.load()==1;}));
  // Pump the accepted-goal response so the client has issued GetResult before
  // making both a terminal result and a replacement available.
  for (int i=0;i<3;++i) {tick();}
  // This failure and a newer path are both available on the next BT tick.
  auto result=std::make_shared<FollowAction::Result>();
  result->error_code=FollowAction::Result::INVALID_EPOCH;result->error_msg="Failed to make progress";
  {std::lock_guard<std::mutex> lock(mutex_);follow_handle->abort(result);}
  auto newer=output();++newer.token.plan_sequence;blackboard_->set("epoch_raw_path",newer);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(tick(),BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(follow_requests.load(),1);
  tree_->haltTree();tree_.reset();
}

TEST_F(EpochBT, BackupAcknowledgmentSurvivesExecutionBeyondAckDeadline)
{
  using BackupAction = navigo_epoch_msgs::action::BackUpEpoch;
  using BackupHandle = rclcpp_action::ServerGoalHandle<BackupAction>;
  std::shared_ptr<BackupHandle> backup_handle;
  std::atomic<bool> accepted{false};
  auto backup_server = rclcpp_action::create_server<BackupAction>(mock_, "back_up_epoch",
    [](const auto &, const auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](const auto) {return rclcpp_action::CancelResponse::ACCEPT;},
    [&](std::shared_ptr<BackupHandle> handle) {
      std::lock_guard<std::mutex> lock(mutex_);backup_handle=handle;accepted=true;
    });
  tree_->haltTree();tree_.reset();
  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><EpochGuard>"
    "<EpochBackUp action_timeout_sec='6'/></EpochGuard></BehaviorTree></root>",blackboard_));
  ASSERT_TRUE(until([&] {return accepted.load();}));
  const auto end=std::chrono::steady_clock::now()+2300ms;
  while (std::chrono::steady_clock::now()<end) {ASSERT_EQ(tick(),BT::NodeStatus::RUNNING);}
  {std::lock_guard<std::mutex> lock(mutex_);backup_handle->succeed(std::make_shared<BackupAction::Result>());}
  BT::NodeStatus status=BT::NodeStatus::RUNNING;
  const auto result_deadline=std::chrono::steady_clock::now()+2s;
  while (status==BT::NodeStatus::RUNNING && std::chrono::steady_clock::now()<result_deadline) {status=tick();}
  ASSERT_EQ(status,BT::NodeStatus::SUCCESS);
  RCLCPP_INFO(client_->get_logger(), "Maximum complete BT tick %.3f ms", maximum_tick_us_/1000.);
}

TEST_F(EpochBT, BackupAcknowledgmentWithTwentyHzFeedbackAndTenHzTree)
{
  using BackupAction = navigo_epoch_msgs::action::BackUpEpoch;
  std::atomic<bool> accepted{false},release{false};
  std::unique_ptr<navigo_util::SimpleActionServer<BackupAction>> backup_server;
  backup_server=std::make_unique<navigo_util::SimpleActionServer<BackupAction>>(
    mock_,"back_up_epoch",[&] {
      accepted=true;const auto deadline=std::chrono::steady_clock::now()+3s;
      while (!release && std::chrono::steady_clock::now()<deadline) {
        backup_server->publish_feedback(std::make_shared<BackupAction::Feedback>());
        std::this_thread::sleep_for(50ms);
      }
      backup_server->succeeded_current(std::make_shared<BackupAction::Result>());
    },nullptr,500ms,true);
  backup_server->activate();
  tree_->haltTree();tree_.reset();
  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><EpochGuard>"
    "<EpochBackUp action_timeout_sec='6'/></EpochGuard></BehaviorTree></root>",blackboard_));
  const auto discovery_deadline=std::chrono::steady_clock::now()+4s;
  while (!accepted && std::chrono::steady_clock::now()<discovery_deadline) {tick();std::this_thread::sleep_for(90ms);}
  ASSERT_TRUE(accepted);
  const auto end=std::chrono::steady_clock::now()+2300ms;
  BT::NodeStatus during=BT::NodeStatus::RUNNING;
  while (std::chrono::steady_clock::now()<end && during==BT::NodeStatus::RUNNING) {during=tick();std::this_thread::sleep_for(90ms);}
  release=true;
  const auto cleanup_deadline=std::chrono::steady_clock::now()+1s;
  while (backup_server->is_running() && std::chrono::steady_clock::now()<cleanup_deadline) {std::this_thread::sleep_for(1ms);}
  ASSERT_EQ(during,BT::NodeStatus::RUNNING);
  BT::NodeStatus status=BT::NodeStatus::RUNNING;
  const auto result_deadline=std::chrono::steady_clock::now()+2s;
  while (status==BT::NodeStatus::RUNNING && std::chrono::steady_clock::now()<result_deadline) {status=tick();}
  ASSERT_EQ(status,BT::NodeStatus::SUCCESS);
  RCLCPP_INFO(client_->get_logger(), "Maximum complete BT tick %.3f ms", maximum_tick_us_/1000.);
}
