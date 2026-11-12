// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include "matrix_recovery_bt/srv/recovery_handoff.hpp"
#include "matrix_recovery_bt/stop_barrier.hpp"

using namespace std::chrono_literals;
namespace matrix_recovery_bt {
class Gate : public rclcpp::Node {
public:
  Gate() : Node("matrix_zero_handoff") {
    run_=declare_parameter<std::string>("run_id", "");
    raw_=declare_parameter<std::string>("raw_topic", "/matrix/controller_cmd_vel");
    if (run_.empty()) {throw std::invalid_argument("run_id required");}
    pub_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav",rclcpp::QoS(10).reliable());
    events_=create_publisher<std_msgs::msg::String>("/experiment/recovery/events",100);
    sub_=create_subscription<geometry_msgs::msg::Twist>(raw_,rclcpp::QoS(10).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr cmd,const rclcpp::MessageInfo & info) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (epoch_.mode!=ZeroEpoch::Mode::Controller) {return;}
        const auto & m=info.get_rmw_message_info();
        const auto wall=rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
        // Timestamp is DDS transport metadata, never a fabricated sensor header.
        if (m.source_timestamp<=released_wall_ || m.received_timestamp<=released_wall_ ||
          wall<m.source_timestamp || wall-m.source_timestamp>300000000 ||
          !std::isfinite(cmd->linear.x)||!std::isfinite(cmd->linear.y)||
          !std::isfinite(cmd->angular.z)) {return;}
        auto endpoints=get_publishers_info_by_topic(raw_);
        if (endpoints.size()!=1 || endpoints[0].node_name()!="controller_server" ||
          !std::equal(endpoints[0].endpoint_gid().begin(),
            endpoints[0].endpoint_gid().end(),m.publisher_gid.data)) {
          return;
        }
        pub_->publish(*cmd);++sequence_;
      });
    service_=create_service<srv::RecoveryHandoff>("/matrix/recovery_handoff",
      [this](srv::RecoveryHandoff::Request::SharedPtr req,srv::RecoveryHandoff::Response::SharedPtr res) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (req->run_id!=run_) {res->reason="run identity mismatch";return;}
        auto key=req->operation_id;
        if (key.empty()) {res->reason="operation identity required";return;}
        auto previous=replies_.find(key);
        if (previous!=replies_.end()) {
          if (previous->second.first==req->operation+":"+std::to_string(req->epoch)) {*res=previous->second.second;}
          else {res->reason="operation id reused with different request";}
          return;
        }
        if (req->operation=="ACQUIRE") {
          // Compare-and-swap prevents a delayed acquire from closing a newer controller epoch.
          if (req->epoch==epoch_.epoch) {epoch_.acquire();publishZero();res->accepted=true;}
        } else if (req->operation=="RELEASE") {
          res->accepted=epoch_.release(req->epoch);
          if (res->accepted) {released_wall_=rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();}
        } else if (req->operation=="CLOSE") {
          res->accepted=epoch_.close(req->epoch);
          if (res->accepted) {publishZero();}
        } else if (req->operation=="INSPECT") {res->accepted=true;}
        res->epoch=epoch_.epoch;res->mode=mode();res->last_output_sequence=sequence_;
        res->last_recovery_zero_sequence=last_zero_;
        if (!res->accepted) {res->reason="stale epoch or invalid transition";}
        emit(req->operation,res->accepted);
        if (replies_.size()<4096) {replies_[key]={req->operation+":"+std::to_string(req->epoch),*res};}
      });
    timer_=create_wall_timer(50ms,[this] {
      std::lock_guard<std::mutex> guard(mutex_);
      // No delayed timer may publish zero after RELEASE under the same lock.
      if (epoch_.zeroAllowed(epoch_.epoch)) {publishZero();}
    });
  }
private:
  std::string mode() const {
    return epoch_.mode==ZeroEpoch::Mode::Hold?"HOLD":epoch_.mode==ZeroEpoch::Mode::Controller?"CONTROLLER":"CLOSED";
  }
  void publishZero() {pub_->publish(geometry_msgs::msg::Twist{});last_zero_=++sequence_;}
  void emit(const std::string & operation,bool accepted) {
    std::ostringstream out;out<<"{\"schema\":\"matrix.recovery.handoff.v1\",\"run_id\":\""<<run_<<"\",\"operation\":\""<<operation
      <<"\",\"accepted\":"<<(accepted?"true":"false")<<",\"epoch\":"<<epoch_.epoch
      <<",\"mode\":\""<<mode()<<"\",\"last_output_sequence\":"<<sequence_
      <<",\"last_recovery_zero_sequence\":"<<last_zero_<<"}";
    std_msgs::msg::String msg;msg.data=out.str();events_->publish(msg);
  }
  std::mutex mutex_;ZeroEpoch epoch_;std::string run_,raw_;
  std::uint64_t sequence_=0,last_zero_=0;std::int64_t released_wall_=0;
  std::map<std::string,std::pair<std::string,srv::RecoveryHandoff::Response>> replies_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Service<srv::RecoveryHandoff>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<matrix_recovery_bt::Gate>());
  rclcpp::shutdown();return 0;
}
