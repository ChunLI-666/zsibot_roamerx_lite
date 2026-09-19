// Integration contract tests for the actual dynamically loaded BT plugin.
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_epoch_msgs/msg/epoch_path.hpp"
#include "navigo_epoch_msgs/msg/localization_epoch.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

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
    const auto result = tree_->tickRoot();
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
