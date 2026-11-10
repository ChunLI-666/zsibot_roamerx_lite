// Synthetic DDS probe. The inherited resultCallback executes the linked DSO.
// The seam exposes the production child-client/one-executor sequence, not a
// counterfeit std::promise or a replacement result callback implementation.
#include <nav2_waypoint_follower/waypoint_follower.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
using namespace std::chrono_literals;
using Nav = nav2_msgs::action::NavigateToPose;
using Handle = rclcpp_action::ServerGoalHandle<Nav>;
std::mutex output_mutex;
void event(const std::string & kind, const std::string & extra="") {
  std::lock_guard<std::mutex> lock(output_mutex);
  auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  std::cout << "{\"kind\":\"" << kind << "\",\"steady_ns\":" << ns << extra << "}" << std::endl;
}
std::string id(const rclcpp_action::GoalUUID & uuid) {
  std::ostringstream out;for(auto c:uuid)out<<std::hex<<std::setfill('0')<<std::setw(2)<<int(c);return out.str();
}
template<class F> void until(F fn, double seconds, const char * why) {
  auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
  while(!fn()) {if(std::chrono::steady_clock::now()>end){event("FAIL",",\"reason\":\""+std::string(why)+"\"");std::_Exit(3);}std::this_thread::sleep_for(1ms);}
}
class Probe: public nav2_waypoint_follower::WaypointFollower {
 public:
  std::atomic<int> entered{0},exited{0};
  std::optional<rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult> old_result;
  explicit Probe(const rclcpp::NodeOptions & options):WaypointFollower(options){}
  void send() {
    ClientT::Goal goal;goal.pose.header.frame_id="map";goal.pose.pose.orientation.w=1.;
    typename ActionClient::SendGoalOptions options;
    options.goal_response_callback=[this](const auto & h){event("goal_response",",\"accepted\":"+std::string(h?"true":"false"));goalResponseCallback(h);};
    options.result_callback=[this](const auto & result){
      if (!old_result) old_result=result;
      ++entered;event("actual_DSO_resultCallback_enter",",\"uuid\":\""+id(result.goal_id)+"\"");
      resultCallback(result);
      ++exited;event("actual_DSO_resultCallback_exit");
    };
    future_goal_handle_=nav_to_pose_client_->async_send_goal(goal,options);
    current_goal_status_.status=nav2_waypoint_follower::ActionStatus::PROCESSING;
    event("new_member_future_assigned");
  }
  bool server_ready(){return nav_to_pose_client_->wait_for_action_server(2s);}
  bool response_ready(){return future_goal_handle_.valid() && future_goal_handle_.wait_for(0s)==std::future_status::ready;}
  void spin(){callback_group_executor_.spin_some();}
  int status(){return static_cast<int>(current_goal_status_.status);}
  void check_invalid() {
    current_goal_status_.status=nav2_waypoint_follower::ActionStatus::PROCESSING;
    rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult result;
    result.code=rclcpp_action::ResultCode::SUCCEEDED;
    result.result=std::make_shared<ClientT::Result>();
    resultCallback(result);
    if(current_goal_status_.status!=nav2_waypoint_follower::ActionStatus::PROCESSING)std::_Exit(8);
    event("invalid_future_ignored");
  }
  void replay_old() {
    auto before=current_goal_status_.status;
    resultCallback(old_result.value());
    if(current_goal_status_.status!=before)std::_Exit(9);
    event("old_result_after_new_response_ignored");
  }
};
int main(int argc,char ** argv) {
  if(argc!=2 || !std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID"))!="190" ||
      !std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") || std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE"))!="LOCALHOST")return 2;
  std::string mode=argv[1];bool inverted=mode!="normal";bool rejected=mode=="reject";
  if(mode!="normal" && mode!="inverted" && mode!="reject")return 2;
  std::thread([]{std::this_thread::sleep_for(10s);event("WATCHDOG_EXPIRED");std::_Exit(4);}).detach();
  int ros_argc=1;rclcpp::init(ros_argc,argv);
  rclcpp::NodeOptions options;options.arguments({"--ros-args","-r","__ns:=/astra_b40_callback_fixture"});
  auto probe=std::make_shared<Probe>(options);probe->configure();probe->check_invalid();
  auto peer=std::make_shared<rclcpp::Node>("synthetic_navigate_server","/astra_b40_callback_fixture");
  auto group=peer->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  std::atomic<int> requests{0};std::atomic<bool> new_request{false},release_response{false},new_terminal{false};
  std::mutex handle_mutex;std::shared_ptr<Handle> old_handle;
  auto server=rclcpp_action::create_server<Nav>(peer,"navigate_to_pose",
    [&](const auto & uuid,const auto &){
      int n=++requests;event("server_goal_request",",\"number\":"+std::to_string(n)+",\"uuid\":\""+id(uuid)+"\"");
      if(n==2){new_request=true;until([&]{return release_response.load();},4.,"release new response");if(rejected)return rclcpp_action::GoalResponse::REJECT;}
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },[](auto){return rclcpp_action::CancelResponse::ACCEPT;},
    [&](std::shared_ptr<Handle> h){
      if(requests==1){std::lock_guard<std::mutex> lock(handle_mutex);old_handle=h;}
      else {h->succeed(std::make_shared<Nav::Result>());new_terminal=true;event("server_new_goal_terminal_success",",\"uuid\":\""+id(h->get_goal_id())+"\"");}
    },rcl_action_server_get_default_options(),group);
  rclcpp::executors::MultiThreadedExecutor peer_executor(rclcpp::ExecutorOptions(),3);
  peer_executor.add_node(peer);std::thread peer_thread([&]{peer_executor.spin();});
  if(!probe->server_ready())std::_Exit(5);
  probe->send();until([&]{probe->spin();return probe->response_ready();},2.,"old response");
  until([&]{std::lock_guard<std::mutex> lock(handle_mutex);return bool(old_handle);},1.,"old accepted handle");
  std::this_thread::sleep_for(50ms); // Permit the real old GetResult request to reach the server.
  {std::lock_guard<std::mutex> lock(handle_mutex);old_handle->abort(std::make_shared<Nav::Result>());event("server_old_goal_terminal",",\"uuid\":\""+id(old_handle->get_goal_id())+"\"");}
  std::this_thread::sleep_for(100ms); // Callback group intentionally not spun in this gap.
  if(!inverted)until([&]{probe->spin();return probe->exited==1;},1.,"normal old result drained");
  probe->send();until([&]{return new_request.load();},1.,"new request reached server");
  std::atomic<bool> pumping{true};
  std::thread worker([&]{while(pumping){probe->spin();std::this_thread::sleep_for(1ms);}});
  if(inverted)until([&]{return probe->entered>=1;},1.,"old callback entered DSO");
  if(inverted) {
    until([&]{return probe->entered==probe->exited;},1.,"old callback returned without pending response");
    if(probe->status()!=static_cast<int>(nav2_waypoint_follower::ActionStatus::PROCESSING))return 10;
  }
  release_response=true;event("new_response_released");
  if(!rejected)until([&]{return new_terminal.load();},1.,"server new terminal");
  std::this_thread::sleep_for(400ms);
  bool blocked=probe->entered>probe->exited && !probe->response_ready();
  std::ifstream maps("/proc/self/maps");std::string line;
  while(std::getline(maps,line))if(line.find("waypoint_follower_core.so")!=std::string::npos)event("mapped_core",",\"mapping\":\""+line+"\"");
  event("OBSERVATION",",\"mode\":\""+mode+"\",\"blocked\":"+(blocked?std::string("true"):std::string("false"))+
    ",\"entered\":"+std::to_string(probe->entered)+",\"exited\":"+std::to_string(probe->exited)+
    ",\"new_response_ready\":"+(probe->response_ready()?std::string("true"):std::string("false"))+
    ",\"server_new_success\":"+(new_terminal?std::string("true"):std::string("false")));
  if(blocked){event("UNEXPECTED_BLOCKING_HAZARD");std::_Exit(6);}
  pumping=false;worker.join();probe->replay_old();peer_executor.cancel();peer_thread.join();rclcpp::shutdown();
  if(probe->entered!=probe->exited || !probe->response_ready() ||
      probe->status()!=static_cast<int>(rejected ? nav2_waypoint_follower::ActionStatus::FAILED : nav2_waypoint_follower::ActionStatus::SUCCEEDED))return 7;
  event("GUARDED_ASSERTIONS_PASSED");
  event("NORMAL_EXIT",",\"internal_status\":"+std::to_string(probe->status()));return 0;
}
