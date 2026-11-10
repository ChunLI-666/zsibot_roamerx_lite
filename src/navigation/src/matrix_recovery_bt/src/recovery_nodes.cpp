// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <mutex>
#include <thread>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
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
#include "matrix_recovery_bt/wait_diagnostics.hpp"
#include "matrix_recovery_bt/recovery_errors.hpp"
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
static std::string poseDiagnostic(const geometry_msgs::msg::PoseStamped & p) {
  std::ostringstream s;s<<"{\"frame\":"<<jsonText(p.header.frame_id)<<",\"stamp_ns\":"<<rclcpp::Time(p.header.stamp).nanoseconds()
    <<",\"xyz\":["<<jsonNumber(p.pose.position.x)<<','<<jsonNumber(p.pose.position.y)<<','<<jsonNumber(p.pose.position.z)
    <<"],\"quaternion_xyzw\":["<<jsonNumber(p.pose.orientation.x)<<','<<jsonNumber(p.pose.orientation.y)<<','
    <<jsonNumber(p.pose.orientation.z)<<','<<jsonNumber(p.pose.orientation.w)<<"],\"yaw\":"<<jsonNumber(yaw(p.pose))<<'}';return s.str();
}

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
  bool start_captured=false,plan_invalidated=false,hold_confirmed=false,ownership_unknown=false;
  std::int64_t start_capture_ros=0,start_capture_steady=0,body_lookup_ros=0,body_lookup_steady=0;
  nav_msgs::msg::Path certificate_path;
  int recovery_error=None;
  Certificate certificate=Certificate::InputWait;
  WaitCounters diagnostics;
  std::uint64_t map_sequence=0,map_requests=0,map_responses=0,collision_map_sequence=0;
  std::int64_t map_requested=0,map_responded=0,collision_checked=0;
  const char * map_source="none";
  std::string last_tf_frame,last_exception;
  geometry_msgs::msg::PoseStamped last_body,collision_body;
  bool body_observed=false,collision_free=false;
  nav2_msgs::msg::Costmap::ConstSharedPtr collision_map;

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
        ++map_sequence;map_source="raw_topic";
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
      {std::lock_guard<std::mutex> l(mutex);map_pending=true;++map_requests;map_requested=steady();}
      map_client->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>(),
        [this](rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture f) {
          std::lock_guard<std::mutex> l(mutex);map_pending=false;
          map=std::make_shared<nav2_msgs::msg::Costmap>(f.get()->map);map_receive=steady();
          ++map_sequence;++map_responses;map_responded=steady();map_source="GetCostmap";
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
  bool basicHealthy() {
    return diagnostics.check(!follow_active,Reject::FollowActive) && diagnostics.check(bool(odom),Reject::OdomMissing) &&
      diagnostics.check(odom->header.frame_id=="odom" && odom->child_frame_id=="base_link",Reject::OdomFrame) &&
      diagnostics.check(stampFresh(odom->header.stamp),Reject::OdomStamp) && diagnostics.check(recent(odom_receive),Reject::OdomReceive) &&
      diagnostics.check(normal,Reject::Localization) && diagnostics.check(valid,Reject::PoseValid) &&
      diagnostics.check(recent(loc_receive,.2),Reject::LocalizationReceive) && diagnostics.check(recent(valid_receive,.2),Reject::ValidReceive) &&
      diagnostics.check(bool(scan),Reject::ScanMissing) && diagnostics.check(stampFresh(scan->header.stamp,scan_age),Reject::ScanStamp) &&
      diagnostics.check(recent(scan_receive,scan_age),Reject::ScanReceive) && diagnostics.check(safeStopped(),Reject::SafeCommand);
  }
  geometry_msgs::msg::PoseStamped body(const std::string & frame) {
    last_tf_frame=frame;
    body_lookup_ros=node->now().nanoseconds();body_lookup_steady=steady();
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
    last_body=p;body_observed=true;return p;
  }
  Certificate certified(const nav_msgs::msg::Path * path=nullptr) {
    using namespace robot_feasible_navigation;
    if (!basicHealthy() || !diagnostics.check(bool(map),Reject::MapMissing) ||
      !diagnostics.check(recent(map_receive),Reject::MapReceive) ||
      !diagnostics.check(stampFresh(map->header.stamp),Reject::MapStamp)) {return Certificate::InputWait;}
    const auto & m=map->metadata;
    if (!m.size_x||!m.size_y||!std::isfinite(m.resolution)||m.resolution<=0 ||
      m.size_x>10000 || m.size_y>10000 || map->data.size()!=std::size_t(m.size_x)*m.size_y ||
      !std::isfinite(m.origin.position.x)||!std::isfinite(m.origin.position.y)) {diagnostics.check(false,Reject::MapMetadata);return Certificate::InputWait;}
    if (std::abs(m.origin.orientation.x)>1e-6 || std::abs(m.origin.orientation.y)>1e-6 ||
      std::abs(m.origin.orientation.z)>1e-6 || std::abs(m.origin.orientation.w-1)>1e-6) {diagnostics.check(false,Reject::MapOrientation);return Certificate::InputWait;}
    auto actual=body(map->header.frame_id);
    geometry_msgs::msg::PoseStamped local;
    if (path) {
      if (!diagnostics.check(!path->poses.empty(),Reject::PathMissing) ||
        !diagnostics.check(path->header.frame_id=="map",Reject::PathFrame)) {return Certificate::InvalidPath;}
      if (!start_captured) {return Certificate::InvalidPath;}
      auto current=body("map");
      if (std::hypot(current.pose.position.x-planned_start.pose.position.x,
        current.pose.position.y-planned_start.pose.position.y)>.03 ||
        std::abs(angle(yaw(current.pose),yaw(planned_start.pose)))>.08) {
        diagnostics.check(false,Reject::BodyDrift);return Certificate::PlanInvalidated;
      }
      auto first=path->poses.front();first.header.frame_id=path->header.frame_id;
      // Transform derived intent at current TF; retain every source sample header.
      first.header.stamp=actual.header.stamp;
      local=tf->transform(first,map->header.frame_id);
      if (std::hypot(local.pose.position.x-actual.pose.position.x,local.pose.position.y-actual.pose.position.y)>.05 ||
        std::abs(angle(yaw(local.pose),yaw(actual.pose)))>.25) {
        diagnostics.check(false,Reject::PathStart);return Certificate::PlanInvalidated;
      }
    }
    nav2_costmap_2d::Costmap2D cost(m.size_x,m.size_y,m.resolution,m.origin.position.x,m.origin.position.y);
    std::copy(map->data.begin(),map->data.end(),cost.getCharMap());
    const bool free=sweep::commandSweepFree(cost,footprint,actual.pose.position.x,actual.pose.position.y,
      yaw(actual.pose),0,0,1.);
    collision_map=map;collision_body=actual;collision_map_sequence=map_sequence;collision_checked=steady();collision_free=free;
    if (!diagnostics.check(free,Reject::BodyCollision)) {return Certificate::Blocked;}
    if (!path) {return Certificate::Valid;}
    return diagnostics.check(sweep::sweepFree(cost,footprint,actual.pose.position.x,actual.pose.position.y,yaw(actual.pose),
      local.pose.position.x,local.pose.position.y,yaw(local.pose)),Reject::PathSweep) ?
      Certificate::Valid : Certificate::Blocked;
  }
  // Called at most once/second while waiting, plus a terminal failure snapshot.
  // No extra TF lookup, sensor publication, map copy, filesystem write or guard evaluation.
  std::string snapshot(const char * event,const char * phase,std::int64_t started,std::int64_t stable,
    double timeout,double health_stable) const {
    const auto now=steady(),ros_now=node->now().nanoseconds();const auto b=barrier.diagnostic();
    auto age=[now](std::int64_t receipt){return receipt?jsonNumber((now-receipt)/1e9):"null";};
    auto header=[ros_now](const builtin_interfaces::msg::Time & stamp){
      auto ns=rclcpp::Time(stamp).nanoseconds();return "{\"stamp_ns\":"+std::to_string(ns)+",\"age_sec\":"+jsonNumber((ros_now-ns)/1e9)+"}";};
    auto twist=[](const geometry_msgs::msg::Twist & v){return '['+jsonNumber(v.linear.x)+','+jsonNumber(v.linear.y)+','+jsonNumber(v.angular.z)+']';};
    std::ostringstream s;s<<std::boolalpha<<"{\"schema\":\"matrix.recovery.wait_diagnostic.v1\",\"event\":"<<jsonText(event)
      <<",\"phase\":"<<jsonText(phase)<<",\"run_id\":"<<jsonText(run)<<",\"epoch\":"<<epoch
      <<",\"ros_now_ns\":"<<ros_now<<",\"steady_now_ns\":"<<now<<",\"elapsed_sec\":"<<jsonNumber((now-started)/1e9)
      <<",\"timeout_sec\":"<<jsonNumber(timeout)<<",\"health_stable_required_sec\":"<<jsonNumber(health_stable)
      <<",\"health_stable_elapsed_sec\":"<<(stable?jsonNumber((now-stable)/1e9):"0")
      <<",\"last_rejection\":"<<jsonText(diagnostics.last)<<",\"rejections\":{";
    bool first=true;for(std::size_t i=0;i<diagnostics.rejected.size();++i) {
      if (!diagnostics.rejected[i]) {continue;}if(!first){s<<',';}first=false;
      s<<jsonText(reject_names[i])<<':'<<diagnostics.rejected[i];
    }
    s<<"},\"recovery_error_code\":"<<recovery_error
      <<",\"ownership_unknown\":"<<ownership_unknown<<",\"hold_confirmed\":"<<hold_confirmed
      <<",\"plan_invalidated\":"<<plan_invalidated
      <<",\"planned_start\":"<<(start_captured?poseDiagnostic(planned_start):"null")
      <<",\"planned_start_capture_ros_ns\":"<<start_capture_ros
      <<",\"planned_start_capture_steady_ns\":"<<start_capture_steady
      <<",\"body_lookup_ros_ns\":"<<body_lookup_ros<<",\"body_lookup_steady_ns\":"<<body_lookup_steady
      <<",\"path_header\":{\"frame\":"<<jsonText(certificate_path.header.frame_id)
      <<",\"source\":"<<header(certificate_path.header.stamp)
      <<",\"poses\":"<<certificate_path.poses.size()<<'}';
    if (!certificate_path.poses.empty()) {
      s<<",\"path_first\":"<<poseDiagnostic(certificate_path.poses.front());
    }
    if(start_captured && body_observed && last_body.header.frame_id==planned_start.header.frame_id) {
      s<<",\"plan_start_delta_xy_m\":"<<jsonNumber(std::hypot(last_body.pose.position.x-planned_start.pose.position.x,
        last_body.pose.position.y-planned_start.pose.position.y))
        <<",\"plan_start_delta_yaw_rad\":"<<jsonNumber(std::abs(angle(yaw(last_body.pose),yaw(planned_start.pose))));
    }
    s<<",\"follow_active\":"<<follow_active<<",\"odom\":";
    if(odom){s<<"{\"frame\":"<<jsonText(odom->header.frame_id)<<",\"child_frame\":"<<jsonText(odom->child_frame_id)
      <<",\"source\":"<<header(odom->header.stamp)<<",\"receive_age_sec\":"<<age(odom_receive)<<",\"twist_xy_wz\":"<<twist(odom->twist.twist)<<'}';}else{s<<"null";}
    s<<",\"scan\":";
    if(scan){s<<"{\"frame\":"<<jsonText(scan->header.frame_id)<<",\"source\":"<<header(scan->header.stamp)
      <<",\"receive_age_sec\":"<<age(scan_receive)<<",\"max_age_sec\":"<<jsonNumber(scan_age)<<'}';}else{s<<"null";}
    s<<",\"normal\":"<<normal<<",\"pose_valid\":"<<valid<<",\"localization_receive_age_sec\":"<<age(loc_receive)
      <<",\"valid_receive_age_sec\":"<<age(valid_receive)<<",\"safe_command\":{\"receive_age_sec\":"<<age(safe_receive)
      <<",\"xy_wz\":"<<twist(safe)<<"},\"stop_dwell\":{\"armed\":"<<b.armed<<",\"qualified\":"<<b.qualified
      <<",\"advancing_and_bounded\":"<<b.advancing_and_bounded<<",\"last_sample_healthy\":"<<b.healthy
      <<",\"samples_total\":"<<b.samples<<",\"zero_source_ns\":"<<b.zero_source<<",\"zero_steady_ns\":"<<b.zero_steady
      <<",\"first_source_ns\":"<<b.first_source<<",\"last_source_ns\":"<<b.last_source
      <<",\"first_steady_ns\":"<<b.first_steady<<",\"last_steady_ns\":"<<b.last_steady
      <<",\"source_span_sec\":"<<(b.first_source?jsonNumber((b.last_source-b.first_source)/1e9):"0")
      <<",\"receive_span_sec\":"<<(b.first_steady?jsonNumber((b.last_steady-b.first_steady)/1e9):"0")
      <<",\"last_receive_age_sec\":"<<age(b.last_steady)<<"},\"tf\":{\"last_requested_frame\":"<<jsonText(last_tf_frame)
      <<",\"last_exception\":"<<jsonText(last_exception)<<",\"last_valid_body\":"<<(body_observed?poseDiagnostic(last_body):"null")
      <<"},\"map\":{\"source\":"<<jsonText(map_source)<<",\"sequence\":"<<map_sequence<<",\"receive_age_sec\":"<<age(map_receive)
      <<",\"rpc_pending\":"<<map_pending<<",\"rpc_requests\":"<<map_requests<<",\"rpc_responses\":"<<map_responses
      <<",\"rpc_request_age_sec\":"<<age(map_requested)<<",\"rpc_response_age_sec\":"<<age(map_responded);
    if(map){const auto & m=map->metadata;s<<",\"frame\":"<<jsonText(map->header.frame_id)<<",\"source_header\":"<<header(map->header.stamp)
      <<",\"size_xy\":["<<m.size_x<<','<<m.size_y<<"],\"data_size\":"<<map->data.size()<<",\"resolution\":"<<jsonNumber(m.resolution)
      <<",\"origin_xyz\":["<<jsonNumber(m.origin.position.x)<<','<<jsonNumber(m.origin.position.y)<<','<<jsonNumber(m.origin.position.z)
      <<"],\"origin_quaternion_xyzw\":["<<jsonNumber(m.origin.orientation.x)<<','<<jsonNumber(m.origin.orientation.y)<<','
      <<jsonNumber(m.origin.orientation.z)<<','<<jsonNumber(m.origin.orientation.w)<<']';}
    s<<"},\"last_body_collision_check\":";
    if(collision_map) {
      const auto & m=collision_map->metadata;std::uint64_t unknown=0,lethal=0;
      for(auto value:collision_map->data){unknown+=value==255;lethal+=value==254;}
      s<<"{\"map_sequence\":"<<collision_map_sequence<<",\"checked_age_sec\":"<<age(collision_checked)
        <<",\"map_frame\":"<<jsonText(collision_map->header.frame_id)<<",\"map_source_header\":"<<header(collision_map->header.stamp)
        <<",\"body\":"<<poseDiagnostic(collision_body)<<",\"full_body_free\":"<<collision_free
        <<",\"map_unknown_cells\":"<<unknown<<",\"map_lethal_cells\":"<<lethal;
      double minx=INFINITY,miny=INFINITY,maxx=-INFINITY,maxy=-INFINITY;
      const auto heading=yaw(collision_body.pose);for(const auto & p:footprint){
        const auto x=collision_body.pose.position.x+std::cos(heading)*p.x-std::sin(heading)*p.y;
        const auto y=collision_body.pose.position.y+std::sin(heading)*p.x+std::cos(heading)*p.y;
        minx=std::min(minx,x);maxx=std::max(maxx,x);miny=std::min(miny,y);maxy=std::max(maxy,y);}
      const auto clamp_index=[](double v,unsigned int size){return static_cast<unsigned int>(std::clamp(v,0.,double(size-1)));};
      const auto x0=clamp_index(std::floor((minx-m.origin.position.x)/m.resolution)-1,m.size_x);
      const auto x1=clamp_index(std::floor((maxx-m.origin.position.x)/m.resolution)+1,m.size_x);
      const auto y0=clamp_index(std::floor((miny-m.origin.position.y)/m.resolution)-1,m.size_y);
      const auto y1=clamp_index(std::floor((maxy-m.origin.position.y)/m.resolution)+1,m.size_y);
      std::uint64_t u=0,l=0;for(auto y=y0;y<=y1;++y){for(auto x=x0;x<=x1;++x){auto v=collision_map->data[std::size_t(y)*m.size_x+x];u+=v==255;l+=v==254;}}
      s<<",\"padded_body_aabb_cells_xyxy\":["<<x0<<','<<y0<<','<<x1<<','<<y1<<"],\"body_aabb_unknown_cells\":"<<u
        <<",\"body_aabb_lethal_cells\":"<<l<<",\"count_boundary\":\"AABB plus one cell; counts are diagnostic, full_body_free is the unchanged polygon sweep result\"} ";
    }else{s<<"null";}
    s<<",\"counter_boundary\":\"First rejecting predicate per call; odom callback and BT tick counts, not elapsed durations\"}";
    return s.str();
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
      BT::InputPort<nav_msgs::msg::Path>("path"),BT::OutputPort<std::uint16_t>("error_code_id")};
  }
  BT::NodeStatus onStart() override {
    if (!config().blackboard->get("matrix_stop_inputs",inputs_)) {
      inputs_=std::make_shared<Inputs>(*this);config().blackboard->set("matrix_stop_inputs",inputs_);
    }
    started_=steady();stable_=0;pending_=false;acquired_=false;inspected_=false;
    {
      std::lock_guard<std::mutex> l(inputs_->mutex);
      inputs_->diagnostics=WaitCounters{};
      inputs_->last_exception.clear();
      inputs_->recovery_error=None;
      inputs_->certificate=Certificate::InputWait;
      if (!release_) {
        inputs_->start_captured=false;
        inputs_->certificate_path=nav_msgs::msg::Path{};
      }
    }
    outputError(None);
    phase_="WAIT_HANDOFF_SERVICE";last_diagnostic_=0;
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    auto & x=*inputs_;
    std::lock_guard<std::mutex> l(x.mutex);
    if (x.ownership_unknown) {
      // Removing a pending client future cannot undo an accepted RELEASE.
      // No automatic retry may treat this as a confirmed zero-owning epoch.
      return fail(OwnershipUnknown,"ownership_requires_reconcile");
    }
    if (steady()-started_>getInput<double>("timeout").value()*1e9) {
      const int code=(pending_ || (!release_ && !acquired_)) ? OwnershipUnknown :
        (x.certificate==Certificate::Blocked ? CollisionWaitTimeout : InputHealthTimeout);
      return fail(code,"timeout");
    }
    if (!x.client->service_is_ready()) {return waiting("WAIT_HANDOFF_SERVICE");}
    if (!release_ && !acquired_) {
      if (!pending_) {send(inspected_?"ACQUIRE":"INSPECT",x.epoch);return waiting(inspected_?"WAIT_ACQUIRE_RESPONSE":"WAIT_INSPECT_RESPONSE");}
      if (future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {return waiting(phase_);}
      auto r=future_.get();pending_=false;
      if (!inspected_) {
        if (!r->accepted) {return fail(OwnershipUnknown,"inspect_rejected");}
        x.epoch=r->epoch;inspected_=true;return waiting("WAIT_ACQUIRE_RESPONSE");
      }
      if (!r->accepted || r->mode!="HOLD") {return fail(OwnershipUnknown,"acquire_rejected");}
      x.epoch=r->epoch;x.released=false;acquired_=true;x.hold_confirmed=true;
      x.barrier.arm(steady(),x.odom?rclcpp::Time(x.odom->header.stamp).nanoseconds():0);
    }
    bool healthy=false;
    nav_msgs::msg::Path path;
    try {
      if (release_) {
        if (!x.hold_confirmed) {return fail(OwnershipUnknown,"release_without_confirmed_hold");}
        if (x.plan_invalidated) {return fail(PlanStartInvalidated,"plan_already_invalidated");}
        auto input=getInput<nav_msgs::msg::Path>("path");
        if (!input) {return fail(InvalidPlanContract,"missing_path_input");}
        path=input.value();x.certificate_path=path;
      }
      x.certificate=x.certified(release_?&path:nullptr);
      healthy=x.certificate==Certificate::Valid && x.diagnostics.check(x.barrier.ready(steady()),Reject::StopDwell);
    } catch (const std::exception & error) {
      x.diagnostics.check(false,Reject::Exception);x.last_exception=error.what();
      x.certificate=Certificate::InputWait;
    }
    if (!healthy) {
      stable_=0;
      if (release_ && pending_) {return fail(OwnershipUnknown,"health_lost_during_release");}
      if (x.certificate==Certificate::PlanInvalidated) {
        x.plan_invalidated=true;
        return fail(PlanStartInvalidated,"plan_start_invalidated");
      }
      if (x.certificate==Certificate::InvalidPath) {return fail(InvalidPlanContract,"invalid_plan_contract");}
      return waiting(release_?"WAIT_PLAN_START_CERTIFICATE":"WAIT_STOP_AND_HEALTH");
    }
    if (!release_) {
      if (!stable_) {stable_=steady();}
      if (steady()-stable_<getInput<double>("health_stable").value()*1e9) {return waiting("WAIT_HEALTH_STABLE");}
      // Capture only after the unchanged stop/health predicates qualify. A TF
      // failure here is a finite input wait, not an exception escaping the BT.
      try {x.planned_start=x.body("map");}
      catch (const std::exception & error) {
        x.last_exception=error.what();x.certificate=Certificate::InputWait;stable_=0;
        return waiting("WAIT_PLAN_START_TF");
      }
      x.start_captured=true;x.plan_invalidated=false;
      x.start_capture_ros=x.node->now().nanoseconds();x.start_capture_steady=steady();
      diagnose("planned_start_captured",false);
      outputError(None);return BT::NodeStatus::SUCCESS;
    }
    if (!pending_) {send("RELEASE",x.epoch);return waiting("WAIT_RELEASE_RESPONSE");}
    if (future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {return waiting("WAIT_RELEASE_RESPONSE");}
    auto r=future_.get();pending_=false;
    if (!r->accepted || r->epoch!=x.epoch || r->mode!="CONTROLLER") {return fail(OwnershipUnknown,"release_rejected");}
    x.released=true;x.hold_confirmed=false;outputError(None);return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {
    if (release_ && pending_ && inputs_) {
      std::lock_guard<std::mutex> l(inputs_->mutex);
      inputs_->ownership_unknown=true;
      inputs_->recovery_error=OwnershipUnknown;outputError(OwnershipUnknown);
      diagnose("halt_during_release",true);
    }
    cancelPending();
    // Gate owns all zero publication. In particular success cleanup and stale
    // halts never publish or close a released epoch under the next FollowPath.
  }
private:
  void outputError(int code) {
    if (config().output_ports.count("error_code_id")) {setOutput("error_code_id",static_cast<std::uint16_t>(code));}
  }
  BT::NodeStatus fail(int code,const char * event) {
    inputs_->recovery_error=code;
    if (code==OwnershipUnknown) {inputs_->ownership_unknown=true;}
    outputError(code);
    cancelPending();diagnose(event,true);
    return BT::NodeStatus::FAILURE;
  }
  BT::NodeStatus waiting(const char * phase) {phase_=phase;diagnose("waiting",false);return BT::NodeStatus::RUNNING;}
  void diagnose(const char * event,bool force) noexcept {
    const auto now=steady();if(!force && last_diagnostic_ && now-last_diagnostic_<1000000000LL){return;}
    last_diagnostic_=now;
    try {
      const auto text=inputs_->snapshot(event,phase_,started_,stable_,getInput<double>("timeout").value(),getInput<double>("health_stable").value());
      if(force){RCLCPP_WARN(inputs_->node->get_logger(),"matrix_stop_wait_v1 %s",text.c_str());}
      else{RCLCPP_INFO(inputs_->node->get_logger(),"matrix_stop_wait_v1 %s",text.c_str());}
    } catch (...) {++diagnostic_drops_;} // Diagnostics never change the BT result.
  }
  void send(const std::string & op,std::uint64_t epoch) {
    auto req=std::make_shared<Service::Request>();req->run_id=inputs_->run;req->epoch=epoch;req->operation=op;
    static std::atomic<std::uint64_t> count{0};
    req->operation_id=inputs_->run+":"+std::to_string(steady())+":"+std::to_string(++count);
    auto f=inputs_->client->async_send_request(req);request_id_=f.request_id;future_=f.future.share();pending_=true;
  }
  void cancelPending() {if (pending_ && inputs_) {inputs_->client->remove_pending_request(request_id_);pending_=false;}}
  bool release_,pending_=false,acquired_=false,inspected_=false;
  std::int64_t started_=0,stable_=0,request_id_=0;
  const char * phase_="NOT_STARTED";
  std::int64_t last_diagnostic_=0;
  std::uint64_t diagnostic_drops_=0;
  std::shared_ptr<Inputs> inputs_;
  rclcpp::Client<Service>::SharedFuture future_;
};
class ReleaseNode : public StopNode {
public:
  ReleaseNode(const std::string & n,const BT::NodeConfig & c):StopNode(n,c,true) {}
};

class BeginStoppedReplanAttempt : public BT::SyncActionNode {
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {
    return {BT::BidirectionalPort<std::uint16_t>("controller_error_code"),
      BT::BidirectionalPort<std::uint16_t>("planner_error_code"),BT::BidirectionalPort<std::uint16_t>("recovery_error_code")};
  }
  BT::NodeStatus tick() override {
    const std::array<int,3> prior={getInput<std::uint16_t>("controller_error_code").value_or(0),
      getInput<std::uint16_t>("planner_error_code").value_or(0),getInput<std::uint16_t>("recovery_error_code").value_or(0)};
    std::vector<std::array<int,3>> history;
    (void)config().blackboard->get("matrix_recovery_cause_history",history);
    if (std::any_of(prior.begin(),prior.end(),[](int code){return code!=0;})) {
      if (history.size()==16) {history.erase(history.begin());}
      history.push_back(prior);config().blackboard->set("matrix_recovery_cause_history",history);
      auto node=config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      RCLCPP_WARN(node->get_logger(),"matrix_recovery_cause_v1 prior_controller=%d prior_planner=%d prior_recovery=%d history_size=%zu",
        prior[0],prior[1],prior[2],history.size());
    }
    setOutput("controller_error_code",std::uint16_t{0});setOutput("planner_error_code",std::uint16_t{0});
    setOutput("recovery_error_code",std::uint16_t{0});
    return BT::NodeStatus::SUCCESS;
  }
};

class WouldAStoppedPlanRecoveryHelp : public BT::ConditionNode {
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<std::uint16_t>("error_code")};}
  BT::NodeStatus tick() override {
    return getInput<std::uint16_t>("error_code").value_or(0)==PlanStartInvalidated ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<matrix_recovery_bt::BeginStoppedReplanAttempt>("BeginStoppedReplanAttempt");
  factory.registerNodeType<matrix_recovery_bt::WouldAStoppedPlanRecoveryHelp>("WouldAStoppedPlanRecoveryHelp");
  factory.registerNodeType<matrix_recovery_bt::StopNode>("WaitForStoppedAndHealthy");
  factory.registerNodeType<matrix_recovery_bt::ReleaseNode>("ValidateStoppedPlanStart");
}
