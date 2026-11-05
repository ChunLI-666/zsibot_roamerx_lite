// SPDX-License-Identifier: Apache-2.0
// Experimental compatibility boundary: all public plugin interfaces are installed Jazzy interfaces.
#include <cmath>
#include <limits>
#include <mutex>
#include <chrono>
#include <unordered_map>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_util/robot_utils.hpp>
#include "official_lattice_rpp_adapter/feasible_path_handler.hpp"
#include "official_lattice_rpp_adapter/planner_goal_contract.hpp"
#include "official_lattice_rpp_adapter/terminal_goal_binding.hpp"
#include "robot_feasible_navigation/footprint_sweep.hpp"

namespace official_lattice_rpp_adapter {
namespace sweep = robot_feasible_navigation::sweep;
namespace {
template<class T> T parameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr & n,
  const std::string & name, T value) {
  if (!n->has_parameter(name)) {n->declare_parameter<T>(name, value);}
  return n->get_parameter(name).get_value<T>();
}
void require(bool ok, const std::string & message) {
  if (!ok) {throw std::runtime_error("official_lattice_rpp_adapter: " + message);}
}
bool finite(const geometry_msgs::msg::Pose & p) {
  const auto & q = p.orientation;
  const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
    std::isfinite(p.position.z) && std::abs(p.position.z) < 1e-5 &&
    std::isfinite(norm) && std::abs(norm-1.) < 1e-3 &&
    std::abs(q.x) < 1e-5 && std::abs(q.y) < 1e-5;
}
double yaw(const geometry_msgs::msg::Pose & p) {return tf2::getYaw(p.orientation);}
double angle(double a, double b) {return std::atan2(std::sin(b-a),std::cos(b-a));}
bool edgeFree(nav2_costmap_2d::Costmap2D & map, const sweep::Footprint & fp,
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) {
  return finite(a) && finite(b) && sweep::sweepFree(map,fp,
    a.position.x,a.position.y,yaw(a),b.position.x,b.position.y,yaw(b));
}
// Configuration is fixed for a run so changes cannot invalidate the adapter's invariants.
auto freeze(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::string prefix) {
  return node->add_on_set_parameters_callback([prefix](const std::vector<rclcpp::Parameter> & ps) {
    rcl_interfaces::msg::SetParametersResult result; result.successful=true;
    for (const auto & p: ps) {
      if (p.get_name().rfind(prefix+".",0)==0) {
        result.successful=false;result.reason="Experiment profile is immutable; reconfigure between recorded runs";
      }
    }
    return result;
  });
}
}

using CompletionState=goal_region::CompletionGate;
static std::shared_ptr<CompletionState> completionState(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,const std::string & controller) {
  static std::mutex mutex;
  static std::unordered_map<std::string,std::weak_ptr<CompletionState>> states;
  std::lock_guard<std::mutex> l(mutex);
  const auto key=std::to_string(reinterpret_cast<std::uintptr_t>(node.get()))+"/"+controller;
  auto state=states[key].lock();
  if (!state) {state=std::make_shared<CompletionState>();states[key]=state;}
  return state;
}

class FeasibleGoalChecker : public nav2_core::GoalChecker {
public:
  FeasibleGoalChecker():loader_("nav2_core","nav2_core::GoalChecker") {}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    auto node=parent.lock();require(bool(node),"missing goal checker node");
    node_=parent;map_=map;tf_=map->getTfBuffer();
    require(!parameter(node,name+".stateful",false),"GoalChecker stateful must also be false");
    const auto controller=parameter<std::string>(node,name+".controller_name","FollowPath");
    state_=completionState(node,controller);
    odom_timeout_=parameter(node,controller+".odom_timeout",0.3);
    require(std::isfinite(odom_timeout_)&&odom_timeout_>0.&&odom_timeout_<=0.5,"invalid goal-checker TF timeout");
    state_->invalidate();
    inner_=loader_.createSharedInstance("nav2_controller::SimpleGoalChecker");
    inner_->initialize(parent,name,map);
    guard_=freeze(node,name);
  }
  void reset() override {
    state_->invalidate();std::lock_guard<std::mutex> lock(inner_mutex_);inner_->reset();
  }
  bool isGoalReached(const geometry_msgs::msg::Pose & pose,const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist & velocity) override {
    const auto token=state_->snapshot();
    geometry_msgs::msg::Pose actual=pose;
    if (token.binding&&!latestBody(token.binding->original_goal.header.frame_id,actual)) {return false;}
    if (!token.ready) {return false;}
    bool reached;
    {
      std::lock_guard<std::mutex> lock(inner_mutex_);
      reached=inner_->isGoalReached(actual,
        token.binding?token.binding->original_goal.pose:goal,velocity);
    }
    return reached&&state_->stillEligible(token);
  }
  bool getTolerances(geometry_msgs::msg::Pose & p,geometry_msgs::msg::Twist & v) override {
    std::lock_guard<std::mutex> lock(inner_mutex_);return inner_->getTolerances(p,v);
  }
  std::shared_ptr<CompletionState> bindingState() const {return state_;}
private:
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader_;
  nav2_core::GoalChecker::Ptr inner_;
  std::shared_ptr<CompletionState> state_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double odom_timeout_=0.3;
  std::mutex inner_mutex_,history_mutex_;
  struct TransformHistory {
    std::int64_t stamp=0;
    std::chrono::steady_clock::time_point advanced;
  };
  std::unordered_map<std::string,TransformHistory> history_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr guard_;
  bool latestBody(const std::string & frame,geometry_msgs::msg::Pose & actual) {
    // The incoming server pose and transformed end_pose can both describe a
    // historical TF epoch. Only the latest real body chain checks current G.
    try {
      auto node=node_.lock();if (!node) {return false;}
      const auto transform=tf_->lookupTransform(frame,map_->getBaseFrameID(),tf2::TimePointZero);
      const auto stamp=rclcpp::Time(transform.header.stamp);
      const double age=(node->now()-stamp).seconds();
      if (stamp.nanoseconds()==0||age<-.05||age>odom_timeout_) {return false;}
      actual.position.x=transform.transform.translation.x;
      actual.position.y=transform.transform.translation.y;
      actual.position.z=transform.transform.translation.z;
      actual.orientation=transform.transform.rotation;
      if (!finite(actual)) {return false;}
      std::lock_guard<std::mutex> lock(history_mutex_);
      auto & history=history_[frame+"/"+map_->getBaseFrameID()];
      const auto steady=std::chrono::steady_clock::now();
      if (stamp.nanoseconds()<history.stamp) {return false;}
      if (stamp.nanoseconds()>history.stamp) {history.stamp=stamp.nanoseconds();history.advanced=steady;}
      return std::chrono::duration<double>(steady-history.advanced).count()<=odom_timeout_;
    } catch (const tf2::TransformException &) {return false;}
  }
};

// Read-only observation of installed RPP's rotation decision, using an independent path transformer.
// Reverse/fixed-curvature/stateful modes are disabled and validated by FeasibleRPP.
class ObservedRPP : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
public:
  void setObservedPlan(const nav_msgs::msg::Path & path) {observation_plan_=path;setPlan(path);}
  bool rotationTarget(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed, double & target_delta) {
    if (!is_rotating_to_heading_) {return false;}
    nav2_regulated_pure_pursuit_controller::PathHandler observer(
      tf2::durationFromSec(params_->transform_tolerance),tf_,costmap_ros_);
    // Start from the exact same pre-compute path; a second search on RPP's pruned
    // state could advance a bounded search window and inspect a different carrot.
    observer.setPlan(observation_plan_);
    const auto local = observer.transformGlobalPlan(pose,params_->max_robot_pose_search_dist,false);
    const auto carrot = getLookAheadPoint(getLookAheadDistance(speed), local);
    if (std::hypot(carrot.pose.position.x,carrot.pose.position.y) < goal_dist_tol_) {
      target_delta = yaw(local.poses.back().pose);
    } else {
      target_delta = std::atan2(carrot.pose.position.y,carrot.pose.position.x);
    }
    return true;
  }
private:
  nav_msgs::msg::Path observation_plan_;
};

// Only RPP's internal rotation trigger is narrowed. The server's checker and
// its original actual-goal acceptance tolerances are never changed.
class RPPTriggerTolerance : public nav2_core::GoalChecker {
public:
  RPPTriggerTolerance(nav2_core::GoalChecker * original,double xy):original_(original),xy_(xy) {}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose & p,const geometry_msgs::msg::Pose & g,
    const geometry_msgs::msg::Twist & v) override {return original_->isGoalReached(p,g,v);}
  bool getTolerances(geometry_msgs::msg::Pose & p,geometry_msgs::msg::Twist & v) override {
    if (!original_->getTolerances(p,v)) {return false;}p.position.x=p.position.y=xy_;return true;
  }
private:
  nav2_core::GoalChecker * original_;double xy_;
};

class FeasibleRPP : public nav2_core::Controller {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    auto node=parent.lock();require(bool(node),"missing lifecycle node");
    map_=map;tf_=tf;name_=name;
    node_=node;
    completion_=completionState(node,name);completion_->clearPlan();
    require(sweep::validFootprint(map_->getRobotFootprint()),"convex body footprint required");
    require(!parameter(node,name+".rpp.stateful",false),"RPP stateful must be false");
    require(!parameter(node,name+".rpp.allow_reversing",false),"reverse not enabled in this experiment");
    require(!parameter(node,name+".rpp.use_fixed_curvature_lookahead",false),"fixed curvature mode unsupported");
    require(!parameter(node,name+".rpp.interpolate_curvature_after_goal",false),"interpolated curvature unsupported");
    require(parameter(node,name+".rpp.use_rotate_to_heading",true),"RPP rotation required");
    require(parameter(node,name+".rpp.use_collision_detection",true),"RPP collision checking required");
    require(parameter(node,name+".handler.enforce_path_rotation",true),"path rotation constraints required");
    parameter(node,name+".handler.enforce_path_inversion",true);
    xy_=parameter(node,name+".handler.inversion_xy_tolerance",0.06);
    parameter(node,name+".handler.inversion_yaw_tolerance",0.08);
    parameter(node,name+".handler.minimum_rotation_angle",0.05);
    const double min_rotation=node->get_parameter(name+".handler.minimum_rotation_angle").as_double();
    require(std::isfinite(min_rotation)&&min_rotation>1e-6&&min_rotation<=0.05,
      "rotation threshold must detect all experimental primitive turns");
    horizon_=parameter(node,name+".sweep_horizon",1.0);
    require(std::isfinite(horizon_)&&horizon_>=0.5&&horizon_<=3.,"invalid sweep horizon");
    require(std::isfinite(xy_)&&xy_>0.&&xy_<=0.2,"invalid constraint XY tolerance");
    remaining_length_=parameter(node,name+".completion_path_length",0.08);
    require(std::isfinite(remaining_length_)&&remaining_length_>0.&&remaining_length_<=0.2,
      "invalid remaining path completion tolerance");
    terminal_binding_=parameter(node,name+".terminal_goal_binding",false);
    terminal_resolution_=parameter(node,name+".terminal_global_resolution",0.01);
    terminal_extent_=parameter(node,name+".terminal_global_extent",20.);
    if (terminal_binding_) {
      require(std::isfinite(terminal_resolution_)&&terminal_resolution_>0.&&
        std::abs(terminal_resolution_-map_->getCostmap()->getResolution())<1e-12,
        "goal binding global/local resolution mismatch");
      require(std::isfinite(terminal_extent_)&&terminal_extent_>0.,"invalid terminal global extent");
    }
    allow_static_path_transform_=parameter(node,name+".allow_static_path_transform",false);
    handler_=std::make_unique<backport::FeasiblePathHandler>();
    handler_->initialize(parent,node->get_logger(),name+".handler",map,tf);
    handler_->enableOrderedRotations(terminal_binding_);
    rpp_.configure(parent,name+".rpp",tf,map);
    segment_pub_=node->create_publisher<nav_msgs::msg::Path>(name+"/feasible_segment",rclcpp::QoS(1).reliable().transient_local());
    odom_timeout_=parameter(node,name+".odom_timeout",0.3);
    require(std::isfinite(odom_timeout_)&&odom_timeout_>0.&&odom_timeout_<=0.5,"invalid odom timeout");
    const auto odom_topic=parameter<std::string>(node,"odom_topic","odom");
    odom_sub_=node->create_subscription<nav_msgs::msg::Odometry>(odom_topic,rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_=msg;odom_receipt_=std::chrono::steady_clock::now();
      });
    guard_=freeze(node,name);
  }
  void cleanup() override {completion_->clearPlan();rpp_.cleanup();handler_.reset();segment_pub_.reset();guard_.reset();odom_sub_.reset();completion_.reset();}
  void activate() override {rpp_.activate();segment_pub_->on_activate();}
  void deactivate() override {completion_->invalidate();rpp_.deactivate();segment_pub_->on_deactivate();}
  void setPlan(const nav_msgs::msg::Path & path) override {
    const auto plan_generation=completion_->bindPlan();has_plan_=false;
    terminal_armed_=terminal_cancelled_=false;binding_.reset();
    require(!path.poses.empty()&&!path.header.frame_id.empty(),"empty path/frame");
    for (const auto & p:path.poses) {
      require(finite(p.pose)&&
        (p.header.frame_id.empty()||p.header.frame_id==path.header.frame_id),"invalid planar path/frame");
    }
    path_frame_=path.header.frame_id;
    auto executable=path;
    if (terminal_binding_) {
      try {executable=goal_region::splitBinding(path,terminal_resolution_,terminal_extent_).executable;}
      catch (const std::exception & e) {throw nav2_core::InvalidPath(e.what());}
    }
    // Jazzy RPP can select a behind-the-body first endpoint on sparse paths.
    // Subdivide the SAME line/yaw geometry; retain the exact original goal in
    // the server and do not reset this handler on ordinary control ticks.
    nav_msgs::msg::Path dense;dense.header=executable.header;dense.poses.push_back(executable.poses.front());
    for (std::size_t i=1;i<executable.poses.size();++i) {
      const auto & a=executable.poses[i-1].pose;
      const auto & b=executable.poses[i].pose;
      const double dyaw=angle(yaw(a),yaw(b));
      const double count=std::ceil(std::max(
        std::hypot(b.position.x-a.position.x,b.position.y-a.position.y)/0.01,std::abs(dyaw)/0.02));
      require(std::isfinite(count)&&count<=1000000.-dense.poses.size(),"path subdivision budget exceeded");
      const auto steps=std::max(1,static_cast<int>(count));
      for (int j=1;j<=steps;++j) {
        auto p=executable.poses[i];const double t=double(j)/steps;
        p.pose.position.x=a.position.x+t*(b.position.x-a.position.x);
        p.pose.position.y=a.position.y+t*(b.position.y-a.position.y);
        p.pose.orientation.x=p.pose.orientation.y=0.;
        p.pose.orientation.z=std::sin((yaw(a)+t*dyaw)/2.);
        p.pose.orientation.w=std::cos((yaw(a)+t*dyaw)/2.);
        dense.poses.push_back(p);
      }
      dense.poses.back()=executable.poses[i];
    }
    if (terminal_binding_) {
      auto wire=dense;wire.poses.push_back(path.poses.back());
      auto binding=goal_region::splitBinding(wire,terminal_resolution_,terminal_extent_);
      binding.generation=plan_generation;
      binding_=std::make_shared<const goal_region::Binding>(std::move(binding));
    }
    // This is the only place that resets upstream constraint progression.
    handler_->setPlan(dense);rpp_.reset();has_plan_=true;
    if (binding_) {completion_->installBinding(binding_);}
  }
  void reset() override {completion_->clearPlan();rpp_.reset();has_plan_=false;binding_.reset();terminal_armed_=false;}
  bool cancel() override {completion_->invalidate();terminal_cancelled_=true;return rpp_.cancel();}
  void setSpeedLimit(const double & limit,const bool & percentage) override {rpp_.setSpeedLimit(limit,percentage);}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override {
    const auto generation=completion_->beginCycle(binding_?binding_->generation:0);
    auto * feasible_checker=dynamic_cast<FeasibleGoalChecker *>(checker);
    require(feasible_checker && feasible_checker->bindingState()==completion_,
      "FeasibleRPP requires the correctly bound FeasibleGoalChecker");
    if (!has_plan_) {throw nav2_core::InvalidPath("No active feasible plan");}
    geometry_msgs::msg::Pose tolerance;geometry_msgs::msg::Twist velocity_tolerance;
    require(checker && checker->getTolerances(tolerance,velocity_tolerance),"goal tolerance unavailable");
    require(tolerance.position.x>0. && tolerance.position.x<=xy_,"RPP goal XY exceeds path handoff XY");
    require(tolerance.position.x<=remaining_length_,"completion path length is smaller than RPP XY threshold");
    const double yaw_tolerance=std::abs(yaw(tolerance));
    if (terminal_binding_) {
      require(binding_&&std::isfinite(yaw_tolerance)&&yaw_tolerance>0.,"missing terminal binding/yaw tolerance");
      require(binding_->error<tolerance.position.x,"original XY tolerance does not contain goal-cell endpoint");
    }
    require(pose.header.frame_id==map_->getGlobalFrameID()&&finite(pose.pose),"pose must be in costmap frame");
    require(std::isfinite(velocity.linear.x)&&std::isfinite(velocity.linear.y)&&
      std::isfinite(velocity.linear.z)&&std::isfinite(velocity.angular.x)&&
      std::isfinite(velocity.angular.y)&&std::isfinite(velocity.angular.z),"nonfinite controller velocity");
    checkTransformFresh(map_->getGlobalFrameID(),map_->getBaseFrameID(),false);
    if (path_frame_!=map_->getGlobalFrameID()) {
      checkTransformFresh(map_->getGlobalFrameID(),path_frame_,allow_static_path_transform_);
    }
    {
      std::lock_guard<std::mutex> odom_lock(odom_mutex_);
      if (!odom_) {throw nav2_core::NoValidControl("No measured odometry");}
      const double age=(node_.lock()->now()-rclcpp::Time(odom_->header.stamp)).seconds();
      const double received=std::chrono::duration<double>(std::chrono::steady_clock::now()-odom_receipt_).count();
      const auto & v=odom_->twist.twist;
      if (age<-.05||age>odom_timeout_||received>odom_timeout_||
        !std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.linear.z)||
        !std::isfinite(v.angular.x)||!std::isfinite(v.angular.y)||!std::isfinite(v.angular.z)) {
        throw nav2_core::NoValidControl("Missing, stale, future or nonfinite measured odometry");
      }
    }
    auto * map=map_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const auto fp=map_->getRobotFootprint();
    sweep::ScopedObstacleIndex index(*map);
    if (!sweep::footprintFree(*map,fp,pose.pose.position.x,pose.pose.position.y,yaw(pose.pose))) {
      throw nav2_core::NoValidControl("Filled actual footprint occupied");
    }
    nav_msgs::msg::Path segment;
    bool in_original_region=false;
    geometry_msgs::msg::PoseStamped path_pose=pose;
    if (terminal_binding_) {
      if (!nav2_util::transformPoseInTargetFrame(pose,path_pose,*tf_,path_frame_,map_->getTransformTolerance())) {
        throw nav2_core::ControllerTFError("Unable to evaluate original goal region");
      }
      in_original_region=std::hypot(path_pose.pose.position.x-binding_->original_goal.pose.position.x,
        path_pose.pose.position.y-binding_->original_goal.pose.position.y)<=tolerance.position.x;
      if (terminal_armed_&&!in_original_region) {
        throw nav2_core::NoValidControl("Actual body left original goal region during terminal rotation");
      }
    }
    if (!terminal_armed_) {
      auto [begin,end]=handler_->findPlanSegment(pose);
      segment=handler_->transformLocalPlan(begin,end);
      if (terminal_binding_&&handler_->constraintRotationActive()) {
        const auto & cursor=handler_->constraintHeadings();
        if (!cursor.complete()&&cursor.next()>0) {
          const double planned=angle(cursor.headings()[cursor.next()-1],cursor.target());
          const double remaining=angle(yaw(path_pose.pose),cursor.target());
          if ((planned>goal_region::kHeadingRoundoff&&remaining<-yaw_tolerance)||
            (planned<-goal_region::kHeadingRoundoff&&remaining>yaw_tolerance)) {
            throw nav2_core::NoValidControl("Actual nonterminal rotation would reverse the ordered checkpoint direction");
          }
        }
      }
      if (terminal_binding_&&in_original_region&&
        handler_->terminalHandoffReady(binding_->suffix_begin,tolerance.position.x)) {
        terminal_armed_=true;terminal_headings_.reset(binding_->terminal_headings);
      }
    }
    if (terminal_armed_) {
      terminal_headings_.advance(yaw(path_pose.pose),yaw_tolerance);
      segment=nav_msgs::msg::Path();
      segment.header=pose.header;segment.poses.push_back(pose);
      auto target=pose;
      const double heading=terminal_headings_.complete()?yaw(binding_->original_goal.pose):terminal_headings_.target();
      const double local_heading=yaw(pose.pose)+angle(yaw(path_pose.pose),heading);
      target.pose.orientation.x=target.pose.orientation.y=0.;
      target.pose.orientation.z=std::sin(local_heading/2.);target.pose.orientation.w=std::cos(local_heading/2.);
      segment.poses.push_back(target);
      double previous=yaw(pose.pose);
      for (std::size_t i=terminal_headings_.next();i<terminal_headings_.headings().size();++i) {
        const double next=yaw(pose.pose)+angle(yaw(path_pose.pose),terminal_headings_.headings()[i]);
        if (!sweep::sweepFree(*map,fp,pose.pose.position.x,pose.pose.position.y,previous,
          pose.pose.position.x,pose.pose.position.y,next)) {
          throw nav2_core::NoValidControl("Filled ordered terminal rotation at actual origin blocked");
        }
        previous=next;
      }
    }
    if (terminal_binding_&&!terminal_armed_&&handler_->constraintRotationActive()) {
      const auto & headings=handler_->constraintHeadings();double previous=yaw(pose.pose);
      for (std::size_t i=headings.next();i<headings.headings().size();++i) {
        const double next=yaw(pose.pose)+angle(yaw(path_pose.pose),headings.headings()[i]);
        if (!sweep::sweepFree(*map,fp,pose.pose.position.x,pose.pose.position.y,previous,
          pose.pose.position.x,pose.pose.position.y,next)) {
          throw nav2_core::NoValidControl("Filled ordered nonterminal rotation at actual origin blocked");
        }
        previous=next;
      }
    }
    segment_pub_->publish(segment);
    rpp_.setObservedPlan(segment); // Reset is harmless only with enforced stateful=false.
    const double trigger=terminal_binding_&&!terminal_armed_&&handler_->finalPrefixActive()?
      tolerance.position.x-binding_->error-64.*std::numeric_limits<double>::epsilon()*
      std::max(1.,tolerance.position.x):tolerance.position.x;
    require(trigger>0.,"nonpositive original-goal triangle trigger");
    RPPTriggerTolerance proxy(checker,trigger);
    auto cmd=rpp_.computeVelocityCommands(pose,velocity,&proxy);
    const auto & v=cmd.twist;
    require(std::isfinite(v.linear.x)&&std::isfinite(v.angular.z)&&v.linear.x>=0.&&
      v.linear.y==0.&&v.linear.z==0.&&v.angular.x==0.&&v.angular.y==0.,"invalid command");
    require(!terminal_armed_||v.linear.x==0.,"terminal goal-region command must be pure rotation");
    double target_delta=0.;
    if (rpp_.rotationTarget(pose,velocity,target_delta)) {
      // Check the complete remaining rotation at the ACTUAL displaced body origin.
      // Subsegments preserve the signed 180-degree direction and avoid shortest-arc ambiguity.
      const auto steps=std::max(1,static_cast<int>(std::ceil(std::abs(target_delta)/0.1)));
      double previous=yaw(pose.pose);
      for (int i=1;i<=steps;++i) {
        const double next=yaw(pose.pose)+target_delta*i/steps;
        if (!sweep::sweepFree(*map,fp,pose.pose.position.x,pose.pose.position.y,previous,
          pose.pose.position.x,pose.pose.position.y,next)) {
          throw nav2_core::NoValidControl("Filled remaining actual-origin rotation blocked");
        }
        previous=next;
      }
    }
    if (!sweep::commandSweepFree(*map,fp,pose.pose.position.x,pose.pose.position.y,
      yaw(pose.pose),v.linear.x,v.angular.z,horizon_)) {
      throw nav2_core::NoValidControl("Filled command sweep blocked");
    }
    completion_->finishCycle(generation,terminal_binding_?
      (terminal_armed_&&!terminal_cancelled_&&terminal_headings_.complete()&&in_original_region):
      handler_->finalPrefixWithin(remaining_length_));
    return cmd;
  }
private:
  ObservedRPP rpp_;
  std::unique_ptr<backport::FeasiblePathHandler> handler_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string path_frame_;
  std::shared_ptr<CompletionState> completion_;
  double remaining_length_=0.08;
  bool terminal_binding_=false,terminal_armed_=false,terminal_cancelled_=false;
  double terminal_resolution_=0.01,terminal_extent_=20.;
  std::shared_ptr<const goal_region::Binding> binding_;
  goal_region::HeadingCursor terminal_headings_;
  bool allow_static_path_transform_=false;
  struct TransformHistory {
    std::int64_t stamp=0;
    std::chrono::steady_clock::time_point advanced;
  };
  std::unordered_map<std::string,TransformHistory> transform_history_;
  double xy_=0.06,horizon_=1.;bool has_plan_=false;
  double odom_timeout_=0.3;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::mutex odom_mutex_;
  std::chrono::steady_clock::time_point odom_receipt_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr segment_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr guard_;
  void checkTransformFresh(const std::string & target,const std::string & source,bool allow_static) {
    try {
      const auto transform=tf_->lookupTransform(target,source,tf2::TimePointZero);
      const auto stamp=rclcpp::Time(transform.header.stamp);
      if (allow_static && stamp.nanoseconds()==0) {return;}
      const double age=(node_.lock()->now()-stamp).seconds();
      if (stamp.nanoseconds()==0 || age<-.05 || age>odom_timeout_) {
        throw nav2_core::ControllerTFError("Stale or future actual TF chain: "+source+" -> "+target);
      }
      auto & history=transform_history_[target+"/"+source];
      const auto steady=std::chrono::steady_clock::now();
      if (stamp.nanoseconds()<history.stamp) {
        throw nav2_core::ControllerTFError("Actual TF stamp regressed: "+source+" -> "+target);
      }
      if (stamp.nanoseconds()>history.stamp) {history.stamp=stamp.nanoseconds();history.advanced=steady;}
      if (std::chrono::duration<double>(steady-history.advanced).count()>odom_timeout_) {
        throw nav2_core::ControllerTFError("Actual TF stamp stopped advancing: "+source+" -> "+target);
      }
    } catch (const tf2::TransformException & e) {throw nav2_core::ControllerTFError(e.what());}
  }
};

class CheckedLattice : public nav2_core::GlobalPlanner {
public:
  CheckedLattice():CheckedLattice("nav2_smac_planner::SmacPlannerLattice",true) {}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    auto node=parent.lock();require(bool(node),"missing planner node");map_=map;
    require(sweep::validFootprint(map_->getRobotFootprint()),"planner requires convex body footprint");
    require(!parameter(node,name+".smooth_path",false),"smoothing must be disabled");
    require(!parameter(node,name+".allow_unknown",false),"unknown space must be blocked");
    if (stop_turn_) {
      require(!parameter(node,name+".allow_reverse_expansion",false),"reverse expansions must be disabled");
    } else {
      require(parameter<std::string>(node,name+".motion_model_for_search","DUBIN")=="DUBIN",
        "Hybrid control uses the explicit forward-only DUBIN model");
    }
    require(parameter(node,name+".tolerance",0.)==0.,"exact goal search required");
    terminal_binding_=parameter(node,name+".terminal_goal_binding",false);
    const double terminal_resolution=parameter(node,name+".terminal_global_resolution",0.01);
    if (terminal_binding_) {
      require(stop_turn_,"goal-cell binding is only verified for stop-turn Lattice");
      require(std::isfinite(terminal_resolution)&&terminal_resolution>0.&&
        std::abs(terminal_resolution-map_->getCostmap()->getResolution())<1e-12,
        "goal binding planner resolution mismatch");
    }
    const double analytic_length=parameter(node,name+".analytic_expansion_max_length",stop_turn_?0.:3.);
    require(std::isfinite(analytic_length)&&analytic_length>=0. && (!stop_turn_||analytic_length==0.),
      "stop-turn Lattice requires no analytic connector; Hybrid uses an explicit nonnegative length");
    planner_=loader_.createSharedInstance(planner_type_);
    planner_->configure(parent,name,tf,map);
    guard_=freeze(node,name);
  }
  void cleanup() override {planner_->cleanup();guard_.reset();planner_.reset();}
  void activate() override {planner_->activate();}
  void deactivate() override {planner_->deactivate();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,std::function<bool()> cancel) override {
    std::lock_guard<std::mutex> request_lock(mutex_);
    require(finite(start.pose)&&finite(goal.pose),"invalid planar planner request");
    require(start.header.frame_id==map_->getGlobalFrameID()&&goal.header.frame_id==start.header.frame_id,
      "planner frame mismatch");
    auto * map=map_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const auto fp=map_->getRobotFootprint();sweep::ScopedObstacleIndex index(*map);
    if (!sweep::footprintFree(*map,fp,start.pose.position.x,start.pose.position.y,yaw(start.pose))) {
      throw nav2_core::StartOccupied("Filled start footprint occupied");
    }
    if (!sweep::footprintFree(*map,fp,goal.pose.position.x,goal.pose.position.y,yaw(goal.pose))) {
      throw nav2_core::GoalOccupied("Filled goal footprint occupied");
    }
    auto raw=planner_->createPlan(start,goal,cancel);
    require(!raw.poses.empty()&&raw.poses.size()<1000000,"invalid official planner output");
    // Jazzy Lattice's same-cell shortcut returns start XY with goal yaw, not
    // float-converted goal XY. Accept only its exact documented singleton and
    // the existing in-place path equivalence; larger subcell moves still fail.
    const bool native_same_cell=stop_turn_ &&
      nativeLatticeSameCellSingleton(raw,start,goal,*map);
    if (terminal_binding_) {
      require(goal_region::proveGoalCell(raw,start,goal,*map).valid,
        "official raw endpoint has no bounded goal-cell proof");
    } else if (native_same_cell) {
      require(withinPathRotationXYEquivalence(start.pose,goal.pose),
        "official same-cell path exceeds rotation XY equivalence");
    }
    // Restore only the documented FLOAT cell/origin round trip, including sameXY rotation samples.
    const auto rounded=[&](const geometry_msgs::msg::Pose & p) {
      return std::pair<double,double>{static_cast<float>(map->getOriginX())+
        static_cast<float>((p.position.x-map->getOriginX())/map->getResolution())*map->getResolution(),
        static_cast<float>(map->getOriginY())+
        static_cast<float>((p.position.y-map->getOriginY())/map->getResolution())*map->getResolution()};
    };
    if (!native_same_cell) {
      for (auto & p:raw.poses) {
        for (const auto * exact:{&start,&goal}) {
          const auto [x,y]=rounded(exact->pose);
          const double bound=64.*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(x),std::abs(y)});
          if (std::abs(p.pose.position.x-x)<=bound) {p.pose.position.x=exact->pose.position.x;}
          if (std::abs(p.pose.position.y-y)<=bound) {p.pose.position.y=exact->pose.position.y;}
        }
      }
    }
    // Other exact-search output must retain the goal XY after proven float restoration.
    const auto sameXY=[](const auto & a,const auto & b) {
      return std::hypot(a.position.x-b.position.x,a.position.y-b.position.y)<1e-9;
    };
    require(terminal_binding_ || native_same_cell || sameXY(raw.poses.back().pose,goal.pose),
      "official path did not preserve exact goal XY");
    // Installed official search emits its first primitive sample, not always the
    // exact start node (observed first sample is 1mm). Preserve the actual start
    // and align to that sample's heading before a checked forward connector.
    const auto & first_raw=raw.poses.front().pose;
    const double dx=first_raw.position.x-start.pose.position.x;
    const double dy=first_raw.position.y-start.pose.position.y;
    const double heading=yaw(first_raw);
    const double roundoff=8.*std::numeric_limits<float>::epsilon()*std::max({1.,
      std::abs(map->getOriginX()),std::abs(map->getOriginY()),
      map->getSizeInMetersX(),map->getSizeInMetersY()});
    require(std::hypot(dx,dy)<=map->getResolution()*std::sqrt(5.)+roundoff,
      "first official sample exceeds a bounded primitive connector");
    if (stop_turn_) {
      require(std::abs(dx*std::sin(heading)-dy*std::cos(heading))<=roundoff &&
        dx*std::cos(heading)+dy*std::sin(heading)>=-roundoff,
        "first lattice sample is not a forward primitive connector");
      auto aligned_start=start;
      aligned_start.pose.orientation=first_raw.orientation;
      raw.poses.insert(raw.poses.begin(),aligned_start);
    }
    // Hybrid retains its curved first SE2 interval. The identical filled
    // path/command guards still apply; it is a control group, not stop-turn search.
    nav_msgs::msg::Path output;output.header=raw.header;
    auto first=start;first.header=output.header;output.poses.push_back(first);
    if (terminal_binding_) {
      auto attitude=raw.poses.back();attitude.pose.orientation=goal.pose.orientation;
      raw.poses.push_back(attitude); // Checked rotation at the actual executable endpoint.
    } else {
      raw.poses.push_back(goal); // Historical numerical co-location mode.
    }
    for (const auto & p:raw.poses) {
      if (cancel&&cancel()) {throw nav2_core::PlannerCancelled("Filled sweep canceled");}
      const auto a=output.poses.back().pose;
      if (!edgeFree(*map,fp,a,p.pose)) {throw nav2_core::NoValidPathCouldBeFound("Filled official path sweep blocked");}
      // Densify rotations so constraint detection retains the complete rotation and its signed direction.
      const double dyaw=angle(yaw(a),yaw(p.pose));
      const int count=std::max(1,static_cast<int>(std::ceil(std::abs(dyaw)/0.02)));
      for (int i=1;i<=count;++i) {
        auto q=p;q.header=output.header;const double t=double(i)/count;
        q.pose.position.x=a.position.x+t*(p.pose.position.x-a.position.x);
        q.pose.position.y=a.position.y+t*(p.pose.position.y-a.position.y);
        q.pose.orientation.x=q.pose.orientation.y=0.;
        q.pose.orientation.z=std::sin((yaw(a)+t*dyaw)/2.);
        q.pose.orientation.w=std::cos((yaw(a)+t*dyaw)/2.);
        output.poses.push_back(q);
      }
    }
    if (terminal_binding_) {
      output.poses.back().pose.orientation=goal.pose.orientation;
      require(edgeFree(*map,fp,output.poses.back().pose,goal.pose),"filled terminal goal-region certificate blocked");
      auto original=goal;original.header=output.header;output.poses.push_back(original);
    } else {output.poses.back().pose=goal.pose;}
    return output;
  }
protected:
  CheckedLattice(std::string type,bool stop_turn):loader_("nav2_core","nav2_core::GlobalPlanner"),
    planner_type_(std::move(type)),stop_turn_(stop_turn) {}
private:
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_;
  std::string planner_type_;
  bool stop_turn_;
  bool terminal_binding_=false;
  nav2_core::GlobalPlanner::Ptr planner_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr guard_;
};
class CheckedHybrid : public CheckedLattice {
public:
  CheckedHybrid():CheckedLattice("nav2_smac_planner::SmacPlannerHybrid",false) {}
};
}
PLUGINLIB_EXPORT_CLASS(official_lattice_rpp_adapter::FeasibleRPP,nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(official_lattice_rpp_adapter::CheckedLattice,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(official_lattice_rpp_adapter::FeasibleGoalChecker,nav2_core::GoalChecker)
PLUGINLIB_EXPORT_CLASS(official_lattice_rpp_adapter::CheckedHybrid,nav2_core::GlobalPlanner)
