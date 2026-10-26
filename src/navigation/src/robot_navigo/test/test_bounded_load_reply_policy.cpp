// Regression fixture includes the product source; no policy reimplementation.
#define main bounded_container_product_main
#include "../src/bounded_component_container.cpp"
#undef main
#include <gtest/gtest.h>
#include <rclcpp_components/node_factory_template.hpp>
#include <array>
#include <algorithm>
#include <cstring>

namespace
{
using Load = composition_interfaces::srv::LoadNode;
struct SendRecord
{
  const rcl_service_t * service;
  rmw_request_id_t request;
  Load::Response response;
  rcl_ret_t result;
};
std::vector<rcl_ret_t> injected_returns;
std::vector<SendRecord> send_records;
size_t return_index = 0;
unsigned int construction_count = 0;
unsigned int resource_count = 0;
bool throw_constructor = false;

class CountedNode : public rclcpp::Node
{
public:
  explicit CountedNode(const rclcpp::NodeOptions & options) : Node("counted", options)
  {
    ++construction_count;
    if (throw_constructor) {throw std::runtime_error("injected constructor failure");}
  }
};
class FixtureManager : public RetryingLoadComponentManager
{
public:
  explicit FixtureManager(std::weak_ptr<rclcpp::Executor> exec)
  : RetryingLoadComponentManager(exec) {}
  std::vector<ComponentResource> get_component_resources(
    const std::string &, const std::string & = "rclcpp_components") const override
  {
    ++resource_count;
    return {{"CountedNode", "no-shared-library-needed"}};
  }
  std::shared_ptr<rclcpp_components::NodeFactory> create_component_factory(
    const ComponentResource &) override
  {
    return std::make_shared<rclcpp_components::NodeFactoryTemplate<CountedNode>>();
  }
protected:
  // Keep tests synchronous. Base::on_load_node is still the actual installed body,
  // with actual component factory/Node construction and unchanged response fields.
  void add_node_to_executor(uint64_t) override {}
};
struct LoadReplyTestAccess
{
  static void dispatch(RetryingLoadComponentManager & manager,
    const std::shared_ptr<rmw_request_id_t> & id, const std::shared_ptr<Load::Request> & request)
  {
    manager.loadNode_srv_->handle_request(id, request);
  }
  static size_t pending(const RetryingLoadComponentManager & manager)
  {return manager.pending_responses_.size();}
  static void due(RetryingLoadComponentManager & manager)
  {
    for (auto & entry : manager.pending_responses_) {
      entry.next_due = RetryingLoadComponentManager::Clock::now() - std::chrono::seconds(1);
    }
  }
  static void expire(RetryingLoadComponentManager & manager)
  {manager.pending_responses_.front().deadline = RetryingLoadComponentManager::Clock::now();}
  static void exhaust(RetryingLoadComponentManager & manager)
  {manager.pending_responses_.front().attempts = 10;}
  static void tick(RetryingLoadComponentManager & manager) {manager.retry_pending_responses();}
  static bool timer_stopped(RetryingLoadComponentManager & manager)
  {return manager.retry_timer_->is_canceled();}
  static size_t live_load_wrappers(RetryingLoadComponentManager & manager)
  {
    size_t count = 0;
    auto service_handle = manager.loadNode_srv_->get_service_handle();
    manager.get_node_base_interface()->get_default_callback_group()->find_service_ptrs_if(
      [&](const auto & service) {
        if (service->get_service_handle().get() == service_handle.get()) {++count;}
        return false;
      });
    return count;
  }
};
std::shared_ptr<rmw_request_id_t> header(int64_t sequence, unsigned char guid = 7)
{
  auto result = std::make_shared<rmw_request_id_t>();
  std::memset(result.get(), 0, sizeof(*result));
  std::memset(result->writer_guid, guid, sizeof(result->writer_guid));
  result->sequence_number = sequence;
  return result;
}
std::shared_ptr<Load::Request> request(const std::string & name = "test_component")
{
  auto result = std::make_shared<Load::Request>();
  result->package_name = "fixture"; result->plugin_name = "CountedNode";
  result->node_name = name;
  return result;
}
void same_response(const SendRecord & a, const SendRecord & b)
{
  EXPECT_EQ(a.service, b.service);
  EXPECT_EQ(a.request.sequence_number, b.request.sequence_number);
  EXPECT_EQ(std::memcmp(a.request.writer_guid, b.request.writer_guid,
    sizeof(a.request.writer_guid)), 0);
  EXPECT_EQ(a.response.success, b.response.success);
  EXPECT_EQ(a.response.error_message, b.response.error_message);
  EXPECT_EQ(a.response.full_node_name, b.response.full_node_name);
  EXPECT_EQ(a.response.unique_id, b.response.unique_id);
}
class ReplyPolicy : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    construction_count = resource_count = 0; throw_constructor = false;
    send_records.clear(); injected_returns = {RCL_RET_OK}; return_index = 0;
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    manager = std::make_shared<FixtureManager>(executor);
  }
  void TearDown() override
  {
    manager.reset(); executor.reset();
    if (rclcpp::ok()) {rclcpp::shutdown();}
    rcl_reset_error();
  }
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<FixtureManager> manager;
};
TEST_F(ReplyPolicy, FirstOkIsExactlyOneSendAndOneConstruction)
{
  LoadReplyTestAccess::dispatch(*manager, header(1), request());
  ASSERT_EQ(send_records.size(), 1u);
  EXPECT_TRUE(send_records[0].response.success);
  EXPECT_EQ(construction_count, 1u); EXPECT_EQ(resource_count, 1u);
  EXPECT_EQ(LoadReplyTestAccess::pending(*manager), 0u);
  EXPECT_TRUE(LoadReplyTestAccess::timer_stopped(*manager));
  LoadReplyTestAccess::tick(*manager);
  EXPECT_EQ(send_records.size(), 1u);
}
TEST_F(ReplyPolicy, TimeoutThenOkPreservesWholeIdAndResponseAndConstructsOnce)
{
  injected_returns = {RCL_RET_TIMEOUT, RCL_RET_OK};
  auto id = header(2); LoadReplyTestAccess::dispatch(*manager, id, request());
  ASSERT_EQ(LoadReplyTestAccess::pending(*manager), 1u);
  EXPECT_FALSE(LoadReplyTestAccess::timer_stopped(*manager));
  // The original request metadata is not the retained response metadata.
  id->sequence_number = 999; std::memset(id->writer_guid, 99, sizeof(id->writer_guid));
  LoadReplyTestAccess::due(*manager); LoadReplyTestAccess::tick(*manager);
  ASSERT_EQ(send_records.size(), 2u); same_response(send_records[0], send_records[1]);
  EXPECT_EQ(construction_count, 1u); EXPECT_EQ(resource_count, 1u);
  EXPECT_EQ(LoadReplyTestAccess::pending(*manager), 0u);
  EXPECT_TRUE(LoadReplyTestAccess::timer_stopped(*manager));
}
TEST_F(ReplyPolicy, NoAttemptBeforeDue)
{
  injected_returns = {RCL_RET_TIMEOUT};
  LoadReplyTestAccess::dispatch(*manager, header(3), request());
  LoadReplyTestAccess::tick(*manager);
  EXPECT_EQ(send_records.size(), 1u);
}
TEST_F(ReplyPolicy, InitialNonTimeoutThrowsWithOriginalDetail)
{
  injected_returns = {RCL_RET_ERROR};
  try {LoadReplyTestAccess::dispatch(*manager, header(4), request()); FAIL();}
  catch (const rclcpp::exceptions::RCLError & e) {
    EXPECT_NE(std::string(e.what()).find("injected reply failure"), std::string::npos);
  }
  EXPECT_EQ(send_records.size(), 1u); EXPECT_EQ(construction_count, 1u);
  EXPECT_EQ(LoadReplyTestAccess::pending(*manager), 0u);
}
TEST_F(ReplyPolicy, TimerNonTimeoutAlsoThrows)
{
  injected_returns = {RCL_RET_TIMEOUT, RCL_RET_ERROR};
  LoadReplyTestAccess::dispatch(*manager, header(5), request());
  LoadReplyTestAccess::due(*manager);
  EXPECT_THROW(LoadReplyTestAccess::tick(*manager), rclcpp::exceptions::RCLError);
  EXPECT_EQ(send_records.size(), 2u); EXPECT_EQ(construction_count, 1u);
}
TEST_F(ReplyPolicy, DeadlineDoesNotStartAnotherSend)
{
  injected_returns = {RCL_RET_TIMEOUT};
  LoadReplyTestAccess::dispatch(*manager, header(6), request());
  LoadReplyTestAccess::due(*manager); LoadReplyTestAccess::expire(*manager);
  LoadReplyTestAccess::tick(*manager);
  EXPECT_EQ(send_records.size(), 1u); EXPECT_EQ(construction_count, 1u);
  EXPECT_EQ(LoadReplyTestAccess::pending(*manager), 0u);
}
TEST_F(ReplyPolicy, MaximumTenAttemptsIncludingInitial)
{
  injected_returns = {RCL_RET_TIMEOUT};
  LoadReplyTestAccess::dispatch(*manager, header(7), request());
  for (unsigned int i = 0; i < 12; ++i) {
    LoadReplyTestAccess::due(*manager); LoadReplyTestAccess::tick(*manager);
  }
  EXPECT_EQ(send_records.size(), 10u); EXPECT_EQ(construction_count, 1u);
  EXPECT_EQ(LoadReplyTestAccess::pending(*manager), 0u);
}
TEST_F(ReplyPolicy, QueueFullHardFailsAfterOneConstructionPerRequest)
{
  injected_returns = {RCL_RET_TIMEOUT};
  for (unsigned int i = 0; i < 32; ++i) {
    LoadReplyTestAccess::dispatch(*manager, header(100+i), request("node_"+std::to_string(i)));
  }
  ASSERT_EQ(LoadReplyTestAccess::pending(*manager), 32u);
  EXPECT_THROW(LoadReplyTestAccess::dispatch(*manager, header(200), request("overflow")),
    std::runtime_error);
  EXPECT_EQ(send_records.size(), 33u); EXPECT_EQ(construction_count, 33u);
  EXPECT_EQ(LoadReplyTestAccess::pending(*manager), 32u);
}
TEST_F(ReplyPolicy, ConstructorFailureResponseIsNotTurnedIntoSuccessOrReconstructed)
{
  throw_constructor = true; injected_returns = {RCL_RET_TIMEOUT, RCL_RET_OK};
  LoadReplyTestAccess::dispatch(*manager, header(8), request());
  ASSERT_EQ(send_records.size(), 1u); EXPECT_FALSE(send_records[0].response.success);
  EXPECT_NE(send_records[0].response.error_message.find("injected constructor failure"), std::string::npos);
  LoadReplyTestAccess::due(*manager); LoadReplyTestAccess::tick(*manager);
  ASSERT_EQ(send_records.size(), 2u); same_response(send_records[0], send_records[1]);
  EXPECT_EQ(construction_count, 1u); EXPECT_EQ(resource_count, 1u);
}
TEST_F(ReplyPolicy, TwoGuidSameSequenceNeverOverwriteEachOther)
{
  injected_returns = {RCL_RET_TIMEOUT, RCL_RET_TIMEOUT, RCL_RET_OK, RCL_RET_OK};
  LoadReplyTestAccess::dispatch(*manager, header(9,1), request("first"));
  LoadReplyTestAccess::dispatch(*manager, header(9,2), request("second"));
  ASSERT_EQ(LoadReplyTestAccess::pending(*manager), 2u);
  LoadReplyTestAccess::due(*manager); LoadReplyTestAccess::tick(*manager);
  ASSERT_EQ(send_records.size(), 4u);
  same_response(send_records[0], send_records[2]); same_response(send_records[1], send_records[3]);
  EXPECT_NE(send_records[0].response.unique_id, send_records[1].response.unique_id);
  EXPECT_EQ(construction_count, 2u); EXPECT_EQ(LoadReplyTestAccess::pending(*manager), 0u);
}
TEST_F(ReplyPolicy, ContextShutdownClearsPendingWithoutSending)
{
  injected_returns = {RCL_RET_TIMEOUT};
  LoadReplyTestAccess::dispatch(*manager, header(10), request());
  rclcpp::shutdown(); LoadReplyTestAccess::due(*manager); LoadReplyTestAccess::tick(*manager);
  EXPECT_EQ(send_records.size(), 1u); EXPECT_EQ(LoadReplyTestAccess::pending(*manager), 0u);
  EXPECT_TRUE(LoadReplyTestAccess::timer_stopped(*manager));
}
TEST_F(ReplyPolicy, DestroyWithPendingHasNoRetainedManagerOrTimerCycle)
{
  injected_returns = {RCL_RET_TIMEOUT};
  LoadReplyTestAccess::dispatch(*manager, header(11), request());
  std::weak_ptr<FixtureManager> weak = manager; manager.reset();
  EXPECT_TRUE(weak.expired()); EXPECT_EQ(send_records.size(), 1u);
}
TEST_F(ReplyPolicy, ExactlyOneLiveServiceWrapperForOriginalHandle)
{
  EXPECT_EQ(LoadReplyTestAccess::live_load_wrappers(*manager), 1u);
}
}  // namespace

extern "C" rcl_ret_t __wrap_rcl_send_response(
  const rcl_service_t * service, rmw_request_id_t * id, void * response)
{
  const auto position = std::min(return_index++, injected_returns.size()-1);
  const rcl_ret_t result = injected_returns.at(position);
  send_records.push_back({service, *id, *static_cast<Load::Response *>(response), result});
  if (result != RCL_RET_OK) {RCL_SET_ERROR_MSG("injected reply failure");}
  return result;
}
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
