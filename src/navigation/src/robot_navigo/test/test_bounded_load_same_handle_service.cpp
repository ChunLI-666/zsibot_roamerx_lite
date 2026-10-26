// Real ROS service regression; includes the product helper and manager.
#define main bounded_container_product_main
#include "../src/bounded_component_container.cpp"
#undef main
#include <gtest/gtest.h>
#include <rclcpp_components/node_factory_template.hpp>
#include <rmw_fastrtps_shared_cpp/custom_service_info.hpp>
#include <atomic>
#include <thread>
#include <cstring>

namespace
{
using Load = composition_interfaces::srv::LoadNode;
bool inject_first_timeout = false;
struct RealSendRecord {const rcl_service_t * service;rmw_request_id_t id;Load::Response response;rcl_ret_t result;};
std::vector<RealSendRecord> real_send_records;
bool until(const std::function<bool()> & condition,
  std::chrono::seconds budget = std::chrono::seconds(5))
{
  const auto end = std::chrono::steady_clock::now()+budget;
  while (std::chrono::steady_clock::now() < end) {
    if (condition()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return condition();
}
void equal_qos(const rmw_qos_profile_t & a, const rmw_qos_profile_t & b)
{
  EXPECT_EQ(a.history,b.history); EXPECT_EQ(a.depth,b.depth);
  EXPECT_EQ(a.reliability,b.reliability); EXPECT_EQ(a.durability,b.durability);
  EXPECT_EQ(a.deadline.sec,b.deadline.sec); EXPECT_EQ(a.deadline.nsec,b.deadline.nsec);
  EXPECT_EQ(a.lifespan.sec,b.lifespan.sec); EXPECT_EQ(a.lifespan.nsec,b.lifespan.nsec);
  EXPECT_EQ(a.liveliness,b.liveliness);
  EXPECT_EQ(a.liveliness_lease_duration.sec,b.liveliness_lease_duration.sec);
  EXPECT_EQ(a.liveliness_lease_duration.nsec,b.liveliness_lease_duration.nsec);
  EXPECT_EQ(a.avoid_ros_namespace_conventions,b.avoid_ros_namespace_conventions);
}
class RealService : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0,nullptr);threads_per_component=1;}
  void TearDown() override {if(rclcpp::ok()){rclcpp::shutdown();}}
};
TEST_F(RealService, QueuedRequestGuidQosAndRmwHandleSurviveOldWrapperRelease)
{
  auto node=std::make_shared<rclcpp::Node>("same_handle_fixture");
  unsigned int old_calls=0,new_calls=0;
  std::atomic<unsigned int> queued{0};
  auto service=node->create_service<Load>("queued_load",
    [&](const std::shared_ptr<Load::Request>,std::shared_ptr<Load::Response>){++old_calls;},
    rclcpp::ServicesQoS().keep_last(200));
  service->set_on_new_request_callback([&](size_t count){queued.fetch_add(count);});
  std::weak_ptr<rclcpp::Service<Load>> old_weak=service;
  auto original_handle=service->get_service_handle();
  const auto * original_rmw=rcl_service_get_rmw_handle(original_handle.get());
  ASSERT_NE(original_rmw,nullptr);
  ASSERT_STREQ(original_rmw->implementation_identifier,"rmw_fastrtps_cpp");
  auto * info=static_cast<CustomServiceInfo *>(original_rmw->data);
  ASSERT_NE(info,nullptr);ASSERT_NE(info->request_reader_,nullptr);ASSERT_NE(info->response_writer_,nullptr);
  const auto request_guid=info->request_reader_->guid();
  const auto response_guid=info->response_writer_->guid();
  const auto request_qos=service->get_request_subscription_actual_qos().get_rmw_qos_profile();
  const auto response_qos=service->get_response_publisher_actual_qos().get_rmw_qos_profile();
  auto client_node=std::make_shared<rclcpp::Node>("same_handle_client");
  auto client=client_node->create_client<Load>("/queued_load");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  ASSERT_TRUE(until([&](){
    eprosima::fastdds::dds::PublicationMatchedStatus status;
    info->response_writer_->get_publication_matched_status(status);
    return status.current_count>0;
  }));
  auto request=std::make_shared<Load::Request>();
  request->package_name="queued_sentinel_package";request->plugin_name="queued_sentinel_plugin";
  request->node_name="queued_sentinel_node";
  auto future=client->async_send_request(request);
  // No server executor has been added/spun: this is an actual queued RMW request.
  ASSERT_TRUE(until([&](){return queued.load()>0;}));
  EXPECT_EQ(old_calls,0u);
  rmw_request_id_t observed_header{};
  rclcpp::AnyServiceCallback<Load> callback;
  std::function<void(std::shared_ptr<rmw_request_id_t>,std::shared_ptr<Load::Request>)> deferred=
    [&](std::shared_ptr<rmw_request_id_t> header,std::shared_ptr<Load::Request> got){
      ++new_calls;observed_header=*header;
      EXPECT_EQ(got->package_name,request->package_name);
      EXPECT_EQ(got->plugin_name,request->plugin_name);
      EXPECT_EQ(got->node_name,request->node_name);
      Load::Response response;response.success=true;response.full_node_name="/queued_preserved";
      response.unique_id=42;
      EXPECT_EQ(rcl_send_response(original_handle.get(),header.get(),&response),RCL_RET_OK);
    };
  callback.set(deferred);
  rebind_service_callback<Load>(*node,service,callback);
  EXPECT_TRUE(old_weak.expired());
  EXPECT_EQ(service->get_service_handle().get(),original_handle.get());
  EXPECT_EQ(rcl_service_get_rmw_handle(service->get_service_handle().get()),original_rmw);
  EXPECT_TRUE(info->request_reader_->guid()==request_guid);
  EXPECT_TRUE(info->response_writer_->guid()==response_guid);
  equal_qos(request_qos,service->get_request_subscription_actual_qos().get_rmw_qos_profile());
  equal_qos(response_qos,service->get_response_publisher_actual_qos().get_rmw_qos_profile());
  rclcpp::executors::SingleThreadedExecutor server_exec,client_exec;
  server_exec.add_node(node);client_exec.add_node(client_node);
  ASSERT_TRUE(until([&](){
    server_exec.spin_some();client_exec.spin_some();
    return future.wait_for(std::chrono::seconds(0))==std::future_status::ready;
  }));
  EXPECT_EQ(new_calls,1u);EXPECT_EQ(old_calls,0u);
  EXPECT_EQ(observed_header.sequence_number,future.request_id);
  bool nonzero=false;for(auto byte:observed_header.writer_guid){nonzero=nonzero||byte!=0;}
  EXPECT_TRUE(nonzero);
  const auto response=future.get();
  ASSERT_TRUE(response->success);EXPECT_EQ(response->unique_id,42u);
  EXPECT_EQ(response->full_node_name,"/queued_preserved");
  // Old new-request callback was removed, not retained with stale callback owner.
  const auto queued_before=queued.load();
  auto second=client->async_send_request(request);
  ASSERT_TRUE(until([&](){server_exec.spin_some();client_exec.spin_some();
    return second.wait_for(std::chrono::seconds(0))==std::future_status::ready;}));
  EXPECT_EQ(queued.load(),queued_before);EXPECT_EQ(new_calls,2u);EXPECT_EQ(old_calls,0u);
  server_exec.remove_node(node);client_exec.remove_node(client_node);
}
std::atomic<unsigned int> actual_constructions{0};
class ActualCountedNode:public rclcpp::Node
{
public:
  explicit ActualCountedNode(const rclcpp::NodeOptions & options):Node("actual_counted",options)
  {++actual_constructions;}
};
class ActualFixtureManager:public RetryingLoadComponentManager
{
public:
  explicit ActualFixtureManager(std::weak_ptr<rclcpp::Executor> exec)
  :RetryingLoadComponentManager(exec){}
  std::vector<ComponentResource> get_component_resources(const std::string &,
    const std::string & = "rclcpp_components") const override
  {return {{"ActualCountedNode","no-library-needed"}};}
  std::shared_ptr<rclcpp_components::NodeFactory> create_component_factory(const ComponentResource &) override
  {return std::make_shared<rclcpp_components::NodeFactoryTemplate<ActualCountedNode>>();}
};
struct LoadReplyTestAccess
{
  static std::string service_name(RetryingLoadComponentManager & manager)
  {return manager.loadNode_srv_->get_service_name();}
  static std::shared_ptr<rcl_service_t> service_handle(RetryingLoadComponentManager & manager)
  {return manager.loadNode_srv_->get_service_handle();}
};
void actual_candidate_client(bool force_first_timeout)
{
  actual_constructions=0;
  auto executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager=std::make_shared<ActualFixtureManager>(executor);
  auto client_node=std::make_shared<rclcpp::Node>("actual_candidate_client");
  auto client=client_node->create_client<Load>(LoadReplyTestAccess::service_name(*manager));
  executor->add_node(manager);executor->add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  auto handle=LoadReplyTestAccess::service_handle(*manager);
  auto * info=static_cast<CustomServiceInfo *>(rcl_service_get_rmw_handle(handle.get())->data);
  ASSERT_TRUE(until([&](){eprosima::fastdds::dds::PublicationMatchedStatus status;
    info->response_writer_->get_publication_matched_status(status);return status.current_count>0;}));
  real_send_records.clear();inject_first_timeout=force_first_timeout;
  auto request=std::make_shared<Load::Request>();request->package_name="fixture";
  request->plugin_name="ActualCountedNode";request->node_name="actual_child";
  auto future=client->async_send_request(request);
  ASSERT_TRUE(until([&](){executor->spin_some();return future.wait_for(std::chrono::seconds(0))==std::future_status::ready;}));
  const auto response=future.get();
  ASSERT_TRUE(response->success);EXPECT_EQ(actual_constructions.load(),1u);
  EXPECT_EQ(response->full_node_name,"/actual_child");
  ASSERT_EQ(real_send_records.size(),force_first_timeout?2u:1u);
  if(force_first_timeout){
    EXPECT_EQ(real_send_records.front().result,RCL_RET_TIMEOUT);
    EXPECT_EQ(real_send_records.back().result,RCL_RET_OK);
    EXPECT_EQ(real_send_records[0].service,real_send_records[1].service);
    EXPECT_EQ(real_send_records[0].id.sequence_number,real_send_records[1].id.sequence_number);
    EXPECT_EQ(std::memcmp(real_send_records[0].id.writer_guid,real_send_records[1].id.writer_guid,
      sizeof(real_send_records[0].id.writer_guid)),0);
    EXPECT_EQ(real_send_records[0].response,real_send_records[1].response);
  }
  auto list_client=client_node->create_client<composition_interfaces::srv::ListNodes>(
    "/ComponentManager/_container/list_nodes");
  ASSERT_TRUE(list_client->wait_for_service(std::chrono::seconds(5)));
  auto list_future=list_client->async_send_request(std::make_shared<composition_interfaces::srv::ListNodes::Request>());
  ASSERT_TRUE(until([&](){executor->spin_some();return list_future.wait_for(std::chrono::seconds(0))==std::future_status::ready;}));
  const auto list_response=list_future.get();
  ASSERT_EQ(list_response->full_node_names.size(),1u);
  EXPECT_EQ(list_response->full_node_names.front(),"/actual_child");
  EXPECT_EQ(list_response->unique_ids.front(),response->unique_id);
  EXPECT_EQ(actual_constructions.load(),1u);
  executor->remove_node(manager);executor->remove_node(client_node);
  manager.reset();
}
TEST_F(RealService, ActualCandidateRealClientAndIsolatedComponentConstructOnce)
{actual_candidate_client(false);}
TEST_F(RealService, FirstTimeoutThenRealReplyCompletesOneFutureAndConstructsOnce)
{actual_candidate_client(true);}
} // namespace
extern "C" rcl_ret_t __real_rcl_send_response(const rcl_service_t *,rmw_request_id_t *,void *);
extern "C" rcl_ret_t __wrap_rcl_send_response(const rcl_service_t * service,rmw_request_id_t * id,void * message)
{
  const auto header=*id;const auto response=*static_cast<Load::Response *>(message);
  rcl_ret_t result;
  if(inject_first_timeout){inject_first_timeout=false;RCL_SET_ERROR_MSG("injected first response timeout");result=RCL_RET_TIMEOUT;}
  else{result=__real_rcl_send_response(service,id,message);}
  real_send_records.push_back({service,header,response,result});
  return result;
}
int main(int argc,char **argv)
{
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
