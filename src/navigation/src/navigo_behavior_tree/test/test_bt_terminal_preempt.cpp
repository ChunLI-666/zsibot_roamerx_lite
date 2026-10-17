// Regression for a pending goal arriving during the old tree's terminal tick.
// Uses the production BtActionServer, BehaviorTreeEngine and SimpleActionServer;
// only the leaf node is replaced to make the timing deterministic.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include "behaviortree_cpp_v3/action_node.h"
#include "navigo_behavior_tree/bt_action_server.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{
using namespace std::chrono_literals;
using Action = nav2_msgs::action::NavigateToPose;
using Server = navigo_behavior_tree::BtActionServer<Action>;
using ClientHandle = rclcpp_action::ClientGoalHandle<Action>;

struct TickBarrier
{
  std::mutex mutex;
  std::condition_variable condition;
  bool entered_a{false}, entered_b{false}, release_a{false}, release_b{false};
  bool timed_out{false};
  unsigned ticks_a{0}, ticks_b{0};

  void releaseAll()
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_a = release_b = true;
    condition.notify_all();
  }
};

class TerminalLeaf : public BT::SyncActionNode
{
public:
  TerminalLeaf(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    const auto barrier = config().blackboard->get<std::shared_ptr<TickBarrier>>("tick_barrier");
    const auto goal_id = config().blackboard->get<int>("business_goal_id");
    std::unique_lock<std::mutex> lock(barrier->mutex);
    if (goal_id == 1) {barrier->entered_a = true; ++barrier->ticks_a;}
    else if (goal_id == 2) {barrier->entered_b = true; ++barrier->ticks_b;}
    else {return BT::NodeStatus::FAILURE;}
    barrier->condition.notify_all();
    const bool released = barrier->condition.wait_for(lock, 8s, [&] {
      return goal_id == 1 ? barrier->release_a : barrier->release_b;
    });
    if (!released) {barrier->timed_out = true; return BT::NodeStatus::FAILURE;}
    return BT::NodeStatus::SUCCESS;
  }
};

class FixtureEngine : public navigo_behavior_tree::BehaviorTreeEngine
{
public:
  FixtureEngine() : BehaviorTreeEngine({})
  {
    factory_.registerNodeType<TerminalLeaf>("TerminalLeaf");
  }
};

class FixtureServer : public Server
{
public:
  using Server::Server;
  void installFixtureEngine() {bt_ = std::make_unique<FixtureEngine>();}
  bool hasPendingGoal() const {return action_server_->is_preempt_requested();}
};

class TerminalPreempt : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }

  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    static std::atomic<unsigned> serial{0};
    const std::string suffix = std::to_string(::getpid()) + "_" + std::to_string(++serial);
    const std::string endpoint = "terminal_preempt_" + suffix;
    xml_ = std::filesystem::temp_directory_path() / (endpoint + ".xml");
    std::ofstream(xml_) <<
      "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
      "<TerminalLeaf/></BehaviorTree></root>";
    parent_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("terminal_server_" + suffix);
    client_node_ = std::make_shared<rclcpp::Node>("terminal_client_" + suffix);
    barrier_ = std::make_shared<TickBarrier>();
    server_ = std::make_unique<FixtureServer>(parent_, endpoint, std::vector<std::string>{},
      xml_.string(),
      [this](Action::Goal::ConstSharedPtr goal) {
        ++goal_received_calls_;
        server_->getBlackboard()->set<int>("business_goal_id", static_cast<int>(goal->pose.pose.position.x));
        return true;
      },
      [] {},
      [this](Action::Goal::ConstSharedPtr) {
        ++preempt_calls_;
        const auto accepted = server_->acceptPendingGoal();
        server_->getBlackboard()->set<int>("business_goal_id", static_cast<int>(accepted->pose.pose.position.x));
      },
      [](Action::Result::SharedPtr, navigo_behavior_tree::BtStatus) {});
    ASSERT_TRUE(server_->on_configure());
    configured_ = true;
    server_->installFixtureEngine();
    server_->getBlackboard()->set("tick_barrier", barrier_);
    ASSERT_TRUE(server_->on_activate());
    activated_ = true;
    client_ = rclcpp_action::create_client<Action>(client_node_, endpoint);
    executor_.add_node(parent_->get_node_base_interface());
    executor_.add_node(client_node_);
    spin_thread_ = std::thread([this] {executor_.spin();});
    ASSERT_TRUE(client_->wait_for_action_server(3s));
  }

  void TearDown() override
  {
    // Always release blocked leaf ticks, including after an ASSERT failure.
    if (barrier_) {barrier_->releaseAll();}
    if (server_ && activated_) {server_->on_deactivate();}
    executor_.cancel();
    if (spin_thread_.joinable()) {spin_thread_.join();}
    if (server_ && configured_ && !server_->getTree().nodes.empty()) {server_->on_cleanup();}
    server_.reset(); client_.reset();
    if (parent_) {executor_.remove_node(parent_->get_node_base_interface());}
    if (client_node_) {executor_.remove_node(client_node_);}
    client_node_.reset(); parent_.reset();
    std::error_code error;
    std::filesystem::remove(xml_, error);
  }

  template<class Predicate>
  bool waitUntil(Predicate predicate, std::chrono::seconds timeout = 3s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }

  std::shared_future<ClientHandle::SharedPtr> send(int id)
  {
    Action::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = id;
    goal.pose.pose.orientation.w = 1;
    return client_->async_send_goal(goal);
  }

  std::filesystem::path xml_;
  std::shared_ptr<TickBarrier> barrier_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  std::unique_ptr<FixtureServer> server_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::atomic<unsigned> goal_received_calls_{0}, preempt_calls_{0};
  bool configured_{false}, activated_{false};
};

TEST_F(TerminalPreempt, TerminalSuccessBelongsToAAndPendingBMustExecuteItsOwnTree)
{
  auto ack_a = send(1);
  ASSERT_EQ(ack_a.wait_for(3s), std::future_status::ready);
  const auto handle_a = ack_a.get(); ASSERT_NE(handle_a, nullptr);
  const auto result_a = client_->async_get_result(handle_a);
  ASSERT_TRUE(waitUntil([&] {
    std::lock_guard<std::mutex> lock(barrier_->mutex); return barrier_->entered_a;
  }));

  // A is inside its terminal tick. Queue B and confirm the actual server's
  // pending slot before allowing A to return SUCCESS (no scheduling guess).
  auto ack_b = send(2);
  ASSERT_EQ(ack_b.wait_for(3s), std::future_status::ready);
  const auto handle_b = ack_b.get(); ASSERT_NE(handle_b, nullptr);
  const auto result_b = client_->async_get_result(handle_b);
  ASSERT_TRUE(waitUntil([&] {return server_->hasPendingGoal();}));
  {
    std::lock_guard<std::mutex> lock(barrier_->mutex);
    barrier_->release_a = true; barrier_->condition.notify_all();
  }

  ASSERT_EQ(result_a.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(result_a.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(waitUntil([&] {
    std::lock_guard<std::mutex> lock(barrier_->mutex); return barrier_->entered_b;
  })) << "Pending B inherited A's terminal result without a B tree tick";
  EXPECT_EQ(result_b.wait_for(50ms), std::future_status::timeout)
    << "B must not succeed while its own leaf has not completed";
  {
    std::lock_guard<std::mutex> lock(barrier_->mutex);
    EXPECT_EQ(barrier_->ticks_a, 1u); EXPECT_EQ(barrier_->ticks_b, 1u);
    EXPECT_FALSE(barrier_->timed_out);
    barrier_->release_b = true; barrier_->condition.notify_all();
  }
  ASSERT_EQ(result_b.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(result_b.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(goal_received_calls_.load(), 2u);
  EXPECT_EQ(preempt_calls_.load(), 0u)
    << "Terminal onLoop must leave B pending for the next executeCallback";
}
}  // namespace
