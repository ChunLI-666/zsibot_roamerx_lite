// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include "robot_feasible_navigation/footprint_sweep.hpp"
#include "matrix_recovery_bt/stop_barrier.hpp"
#include "matrix_recovery_bt/srv/recovery_handoff.hpp"

namespace matrix_recovery_bt {
using Clock=std::chrono::steady_clock;
using Service=srv::RecoveryHandoff;
using Footprint=robot_feasible_navigation::sweep::Footprint;
static std::int64_t steady() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
static double yaw(const geometry_msgs::msg::Pose & p) {
  return std::atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
    1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z));
}
static double angle(double a,double b) {return std::atan2(std::sin(a-b),std::cos(a-b));}

struct Inputs {
  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  rclcpp::Client<Service>::SharedPtr client;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr map_client;
  rclcpp::TimerBase::SharedPtr map_timer;
  bool map_pending=false;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  std::mutex mutex;
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  nav2_msgs::msg::Costmap::ConstSharedPtr map;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  geometry_msgs::msg::Twist safe;
  std::int64_t odom_receive=0,map_receive=0,scan_receive=0,safe_receive=0,loc_receive=0,valid_receive=0;
  bool normal=false,valid=false,follow_active=false;
  StopBarrier barrier;
  std::uint64_t epoch=0;
  bool released=false;
  std::string run;
  double scan_age=.3;
  Footprint footprint;
  geometry_msgs::msg::PoseStamped planned_start;

  explicit Inputs(const BT::TreeNode & owner) {
    node=owner.config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    run=owner.getInput<std::string>("run_id").value();
    scan_age=owner.getInput<double>("scan_max_age").value();
    auto body=owner.getInput<std::string>("footprint").value();
    if (run.empty() || !nav2_costmap_2d::makeFootprintFromString(body,footprint)) {
      throw BT::RuntimeError("Recovery requires run identity and full footprint");
    }
    nav2_costmap_2d::padFootprint(footprint,owner.getInput<double>("padding").value());
    group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    rclcpp::SubscriptionOptions options;options.callback_group=group;
    auto sensor=rclcpp::SensorDataQoS().keep_last(10);
    subs.push_back(node->create_subscription<nav_msgs::msg::Odometry>(
      owner.getInput<std::string>("odom_topic").value(),sensor,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr v) {
        std::lock_guard<std::mutex> l(mutex);odom=v;odom_receive=steady();
        barrier.sample(rclcpp::Time(v->header.stamp).nanoseconds(),odom_receive,basicHealthy(),
          v->twist.twist.linear.x,v->twist.twist.linear.y,v->twist.twist.angular.z);
      },options));
    subs.push_back(node->create_subscription<sensor_msgs::msg::LaserScan>(
      owner.getInput<std::string>("scan_topic").value(),sensor,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr v) {
        std::lock_guard<std::mutex> l(mutex);scan=v;scan_receive=steady();
      },options));
    subs.push_back(node->create_subscription<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw",
      rclcpp::QoS(1).reliable().transient_local(),[this](nav2_msgs::msg::Costmap::ConstSharedPtr v) {
        std::lock_guard<std::mutex> l(mutex);map=v;map_receive=steady();
      },options));
    subs.push_back(node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_safe",10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr v) {
        std::lock_guard<std::mutex> l(mutex);safe=*v;safe_receive=steady();
        if (!safeStopped()) {barrier.invalidate();}
      },options));
    subs.push_back(node->create_subscription<std_msgs::msg::UInt8>("/lightning/loc_status",10,
      [this](std_msgs::msg::UInt8::ConstSharedPtr v) {
        std::lock_guard<std::mutex> l(mutex);normal=v->data==1;loc_receive=steady();
        if (!normal) {barrier.invalidate();}
      },options));
    subs.push_back(node->create_subscription<std_msgs::msg::Bool>("/lightning/pose_valid",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr v) {
        std::lock_guard<std::mutex> l(mutex);valid=v->data;valid_receive=steady();
        if (!valid) {barrier.invalidate();}
      },options));
    subs.push_back(node->create_subscription<action_msgs::msg::GoalStatusArray>("/follow_path/_action/status",
      rclcpp::QoS(10).reliable().transient_local(),[this](action_msgs::msg::GoalStatusArray::ConstSharedPtr v) {
        std::lock_guard<std::mutex> l(mutex);follow_active=false;
        for (const auto & s:v->status_list) {if (s.status>=1 && s.status<=3) {follow_active=true;}}
        if (follow_active) {barrier.invalidate();}
      },options));
    tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener=std::make_shared<tf2_ros::TransformListener>(*tf);
    client=node->create_client<Service>("/matrix/recovery_handoff",rmw_qos_profile_services_default,group);
    map_client=node->create_client<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap",rmw_qos_profile_services_default,group);
    // Normal costmap publication is 1 Hz. Query the current locked map while
    // stopped; never pretend a publication is a fresh sensor sample.
    map_timer=node->create_wall_timer(std::chrono::milliseconds(100),[this] {
      if (map_pending || !map_client->service_is_ready()) {return;}
      map_pending=true;
      map_client->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>(),
        [this](rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture f) {
          std::lock_guard<std::mutex> l(mutex);map_pending=false;
          map=std::make_shared<nav2_msgs::msg::Costmap>(f.get()->map);map_receive=steady();
        });
    },group);
    executor.add_callback_group(group,node->get_node_base_interface());
    thread=std::thread([this]{executor.spin();});
  }
  ~Inputs() {executor.cancel();if (thread.joinable()) {thread.join();}}
  bool recent(std::int64_t receipt,double limit=.3) const {
    auto delta=steady()-receipt;return receipt>0 && delta>=0 && delta<=limit*1e9;
  }
  bool stampFresh(const builtin_interfaces::msg::Time & stamp,double limit=.3) const {
    const auto t=rclcpp::Time(stamp,node->get_clock()->get_clock_type());
    const auto age=(node->now()-t).seconds();return t.nanoseconds()>0 && age>=-.05 && age<=limit;
  }
  bool safeStopped() const {
    return recent(safe_receive) && std::isfinite(safe.linear.x) && std::isfinite(safe.linear.y) &&
      std::isfinite(safe.angular.z) && std::hypot(safe.linear.x,safe.linear.y)<=.01 && std::abs(safe.angular.z)<=.01;
  }
  bool basicHealthy() const {
    return !follow_active && odom && odom->header.frame_id=="odom" && odom->child_frame_id=="base_link" &&
      stampFresh(odom->header.stamp) && recent(odom_receive) && normal && valid &&
      recent(loc_receive,.2) && recent(valid_receive,.2) && scan && stampFresh(scan->header.stamp,scan_age) &&
      recent(scan_receive,scan_age) && safeStopped();
  }
  geometry_msgs::msg::PoseStamped body(const std::string & frame) {
    auto t=tf->lookupTransform(frame,"base_link",tf2::TimePointZero);
    if (!stampFresh(t.header.stamp)) {throw std::runtime_error("stale body TF");}
    geometry_msgs::msg::PoseStamped p;p.header=t.header;p.pose.position.x=t.transform.translation.x;
    p.pose.position.y=t.transform.translation.y;p.pose.position.z=t.transform.translation.z;
    p.pose.orientation=t.transform.rotation;
    const auto & q=p.pose.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(p.pose.position.x)||!std::isfinite(p.pose.position.y)||!std::isfinite(yaw(p.pose))) {
      throw std::runtime_error("nonfinite body");
    }
    if (!std::isfinite(norm)||std::abs(norm-1.)>.01) {throw std::runtime_error("invalid body quaternion");}
    return p;
  }
  bool certified(const nav_msgs::msg::Path * path=nullptr) {
    using namespace robot_feasible_navigation;
    if (!basicHealthy() || !map || !recent(map_receive) || !stampFresh(map->header.stamp)) {return false;}
    const auto & m=map->metadata;
    if (!m.size_x||!m.size_y||!std::isfinite(m.resolution)||m.resolution<=0 ||
      m.size_x>10000 || m.size_y>10000 || map->data.size()!=std::size_t(m.size_x)*m.size_y ||
      !std::isfinite(m.origin.position.x)||!std::isfinite(m.origin.position.y)) {return false;}
    if (std::abs(m.origin.orientation.x)>1e-6 || std::abs(m.origin.orientation.y)>1e-6 ||
      std::abs(m.origin.orientation.z)>1e-6 || std::abs(m.origin.orientation.w-1)>1e-6) {return false;}
    auto actual=body(map->header.frame_id);
    nav2_costmap_2d::Costmap2D cost(m.size_x,m.size_y,m.resolution,m.origin.position.x,m.origin.position.y);
    std::copy(map->data.begin(),map->data.end(),cost.getCharMap());
    if (!sweep::commandSweepFree(cost,footprint,actual.pose.position.x,actual.pose.position.y,
      yaw(actual.pose),0,0,1.)) {return false;}
    if (!path) {return true;}
    if (path->poses.empty() || path->header.frame_id!="map") {return false;}
    auto current=body("map");
    if (std::hypot(current.pose.position.x-planned_start.pose.position.x,
      current.pose.position.y-planned_start.pose.position.y)>.03 ||
      std::abs(angle(yaw(current.pose),yaw(planned_start.pose)))>.08) {return false;}
    auto first=path->poses.front();first.header.frame_id=path->header.frame_id;
    // Transform a derived intent pose at current TF, never rewrite a sample header.
    first.header.stamp=actual.header.stamp;
    auto local=tf->transform(first,map->header.frame_id);
    if (std::hypot(local.pose.position.x-actual.pose.position.x,local.pose.position.y-actual.pose.position.y)>.05 ||
      std::abs(angle(yaw(local.pose),yaw(actual.pose)))>.25) {return false;}
    return sweep::sweepFree(cost,footprint,actual.pose.position.x,actual.pose.position.y,yaw(actual.pose),
      local.pose.position.x,local.pose.position.y,yaw(local.pose));
  }
};

class StopNode : public BT::StatefulActionNode {
public:
  StopNode(const std::string & n,const BT::NodeConfig & c,bool release=false)
  :BT::StatefulActionNode(n,c),release_(release) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("run_id"),BT::InputPort<std::string>("footprint"),
      BT::InputPort<double>("padding",0.,""),BT::InputPort<std::string>("odom_topic","/odom/current_pose",""),
      BT::InputPort<std::string>("scan_topic","/laser_scan",""),BT::InputPort<double>("scan_max_age",.3,""),
      BT::InputPort<double>("timeout",30.,""),BT::InputPort<double>("health_stable",1.,""),
      BT::InputPort<nav_msgs::msg::Path>("path")};
  }
  BT::NodeStatus onStart() override {
    if (!config().blackboard->get("matrix_stop_inputs",inputs_)) {
      inputs_=std::make_shared<Inputs>(*this);config().blackboard->set("matrix_stop_inputs",inputs_);
    }
    started_=steady();stable_=0;pending_=false;acquired_=false;inspected_=false;
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    auto & x=*inputs_;
    if (steady()-started_>getInput<double>("timeout").value()*1e9) {
      cancelPending();return BT::NodeStatus::FAILURE;
    }
    std::lock_guard<std::mutex> l(x.mutex);
    if (!x.client->service_is_ready()) {return BT::NodeStatus::RUNNING;}
    if (!release_ && !acquired_) {
      if (!pending_) {send(inspected_?"ACQUIRE":"INSPECT",x.epoch);return BT::NodeStatus::RUNNING;}
      if (future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {return BT::NodeStatus::RUNNING;}
      auto r=future_.get();pending_=false;
      if (!inspected_) {
        if (!r->accepted) {return BT::NodeStatus::FAILURE;}
        x.epoch=r->epoch;inspected_=true;return BT::NodeStatus::RUNNING;
      }
      if (!r->accepted || r->mode!="HOLD") {return BT::NodeStatus::FAILURE;}
      x.epoch=r->epoch;x.released=false;acquired_=true;
      x.barrier.arm(steady(),x.odom?rclcpp::Time(x.odom->header.stamp).nanoseconds():0);
    }
    bool healthy=false;
    nav_msgs::msg::Path path;
    try {
      if (release_) {path=getInput<nav_msgs::msg::Path>("path").value();}
      healthy=x.certified(release_?&path:nullptr) && x.barrier.ready(steady());
    } catch (const std::exception &) {healthy=false;}
    if (!healthy) {
      stable_=0;
      if (release_ && pending_) {cancelPending();return BT::NodeStatus::FAILURE;}
      return BT::NodeStatus::RUNNING;
    }
    if (!release_) {
      if (!stable_) {stable_=steady();}
      if (steady()-stable_<getInput<double>("health_stable").value()*1e9) {return BT::NodeStatus::RUNNING;}
      x.planned_start=x.body("map");return BT::NodeStatus::SUCCESS;
    }
    if (!pending_) {send("RELEASE",x.epoch);return BT::NodeStatus::RUNNING;}
    if (future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {return BT::NodeStatus::RUNNING;}
    auto r=future_.get();pending_=false;
    if (!r->accepted || r->epoch!=x.epoch || r->mode!="CONTROLLER") {return BT::NodeStatus::FAILURE;}
    x.released=true;return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {
    cancelPending();
    // Gate owns all zero publication. In particular success cleanup and stale
    // halts never publish or close a released epoch under the next FollowPath.
  }
private:
  void send(const std::string & op,std::uint64_t epoch) {
    auto req=std::make_shared<Service::Request>();req->run_id=inputs_->run;req->epoch=epoch;req->operation=op;
    static std::atomic<std::uint64_t> count{0};
    req->operation_id=inputs_->run+":"+std::to_string(steady())+":"+std::to_string(++count);
    auto f=inputs_->client->async_send_request(req);request_id_=f.request_id;future_=f.future.share();pending_=true;
  }
  void cancelPending() {if (pending_ && inputs_) {inputs_->client->remove_pending_request(request_id_);pending_=false;}}
  bool release_,pending_=false,acquired_=false,inspected_=false;
  std::int64_t started_=0,stable_=0,request_id_=0;
  std::shared_ptr<Inputs> inputs_;
  rclcpp::Client<Service>::SharedFuture future_;
};
class ReleaseNode : public StopNode {
public:
  ReleaseNode(const std::string & n,const BT::NodeConfig & c):StopNode(n,c,true) {}
};
}
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<matrix_recovery_bt::StopNode>("WaitForStoppedAndHealthy");
  factory.registerNodeType<matrix_recovery_bt::ReleaseNode>("ValidateStoppedPlanStart");
}
