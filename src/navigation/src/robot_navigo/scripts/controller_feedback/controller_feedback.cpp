// Test-only integration harness: production controller plugin, direct command
// feedback to an ideal SE(2) plant. No actuator, bag playback, or hardware topics.
#include <fstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <xtensor/xrandom.hpp>
#include <pluginlib/class_loader.hpp>
#include <navigo_core/controller.hpp>
#include <navigo_core/goal_checker.hpp>
#include <navigo_costmap_2d/footprint_collision_checker.hpp>
#include <tf2_ros/buffer.h>
#include <rcl/time.h>
#include <std_msgs/msg/string.hpp>

static double angle(double a) {return std::atan2(std::sin(a), std::cos(a));}
static geometry_msgs::msg::Pose pose(double x,double y,double yaw) {
  geometry_msgs::msg::Pose p; p.position.x=x; p.position.y=y;
  p.orientation.z=std::sin(yaw*.5); p.orientation.w=std::cos(yaw*.5); return p;
}
// Independent rectangle-vs-cell SAT oracle, including polygon interior and OOB.
static bool collides(navigo_costmap_2d::Costmap2D* map,double x,double y,double yaw) {
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)) return true;
  constexpr double hx=.31,hy=.16;
  double c=std::cos(yaw),s=std::sin(yaw),r=map->getResolution()/2;
  double ex=std::abs(c)*hx+std::abs(s)*hy,ey=std::abs(s)*hx+std::abs(c)*hy;
  unsigned x0,y0,x1,y1;
  if(!map->worldToMap(x-ex,y-ey,x0,y0)||!map->worldToMap(x+ex,y+ey,x1,y1))return true;
  for(unsigned j=y0;j<=y1;++j) for(unsigned i=x0;i<=x1;++i) {
    if(map->getCost(i,j)<254)continue;
    double wx,wy;map->mapToWorld(i,j,wx,wy);double dx=wx-x,dy=wy-y;
    if(std::abs(dx)>ex+r || std::abs(dy)>ey+r)continue;
    if(std::abs(c*dx+s*dy)>hx+r*(std::abs(c)+std::abs(s)))continue;
    if(std::abs(-s*dx+c*dy)>hy+r*(std::abs(c)+std::abs(s)))continue;
    return true;
  }
  return false;
}
int main(int argc,char**argv) {
  if(argc==2 && std::string(argv[1])=="--self-test") {
    navigo_costmap_2d::Costmap2D grid(100,100,.05,-2.5,-2.5,0);
    if(collides(&grid,0,0,0)||!collides(&grid,2.4,0,0)) return 10;
    grid.setCost(50,50,254);
    if(!collides(&grid,0,0,0)) return 11;  // obstacle inside footprint, not boundary
    grid.setCost(50,50,255);
    if(!collides(&grid,0,0,0)) return 12;
    grid.setCost(50,50,0);unsigned ix,iy;grid.worldToMap(.05,.29,ix,iy);grid.setCost(ix,iy,254);
    if(collides(&grid,0,0,0)||!collides(&grid,0,0,M_PI_2)) return 13;
    if(!collides(&grid,std::numeric_limits<double>::quiet_NaN(),0,0))return 14;
    std::cout<<"PASS: free, out-of-bounds, interior obstacle, unknown, rotated footprint, nonfinite\n";return 0;
  }
  if(argc<4) {std::cerr<<"usage: controller_feedback CASE.yaml PARAMS.yaml OUTPUT.csv [--ros-args ...]\n";return 2;}
  auto data=YAML::LoadFile(argv[1]);
  rclcpp::init(argc,argv); xt::random::seed(data["seed"].as<unsigned>(42));
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",rclcpp::NodeOptions().arguments({"--ros-args","--params-file",argv[2]}));
  auto clock=node->get_clock();
  if(rcl_enable_ros_time_override(clock->get_clock_handle())!=RCL_RET_OK) return 3;
  auto cm=std::make_shared<navigo_costmap_2d::Costmap2DROS>("feedback_costmap","", "feedback_costmap");
  cm->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
  cm->set_parameter(rclcpp::Parameter("global_frame",std::string("odom")));
  cm->set_parameter(rclcpp::Parameter("footprint_padding",0.0));
  cm->set_parameter(rclcpp::Parameter("footprint",std::string("[[0.31,0.16],[0.31,-0.16],[-0.31,-0.16],[-0.31,0.16]]")));
  cm->on_configure(rclcpp_lifecycle::State());
  const auto m=data["map"]; auto costmap=cm->getCostmap();
  costmap->resizeMap(m["width"].as<unsigned>(),m["height"].as<unsigned>(),m["resolution"].as<double>(),m["origin_x"].as<double>(),m["origin_y"].as<double>());
  std::ifstream input(m["data"].as<std::string>(),std::ios::binary);
  input.read(reinterpret_cast<char*>(costmap->getCharMap()),costmap->getSizeInCellsX()*costmap->getSizeInCellsY());
  if(!input) throw std::runtime_error("costmap byte file missing or short");
  auto tf=std::make_shared<tf2_ros::Buffer>(clock);
  pluginlib::ClassLoader<navigo_core::Controller> loader("navigo_core","navigo_core::Controller");
  std::string motion_mode;
  auto mode_sub=node->create_subscription<std_msgs::msg::String>("/controller_server/FollowPath/motion_mode",10,
    [&motion_mode](std_msgs::msg::String::ConstSharedPtr msg){motion_mode=msg->data;});
  auto controller=loader.createSharedInstance("navigo_mppi_controller::MPPIController");
  controller->configure(node,"FollowPath",tf,cm);controller->activate();
  pluginlib::ClassLoader<navigo_core::GoalChecker> goal_loader("navigo_core","navigo_core::GoalChecker");
  auto checker=goal_loader.createSharedInstance("navigo_path_controller::StoppedGoalChecker");
  checker->initialize(node,"general_goal_checker",cm);
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!checker->getTolerances(pose_tolerance, velocity_tolerance) ||
      std::abs(pose_tolerance.position.x - .25) > 1e-8 ||
      std::abs(2*std::atan2(pose_tolerance.orientation.z, pose_tolerance.orientation.w) - .25) > 1e-8 ||
      std::abs(velocity_tolerance.linear.x - .01) > 1e-8 ||
      std::abs(velocity_tolerance.angular.z - .01) > 1e-8) {
    throw std::runtime_error("Actual goal checker tolerances differ from the frozen test contract");
  }
  std::ofstream out(argv[3]);out<<std::setprecision(12)<<"step,t,x,y,yaw,vx,vy,wz,raw_vx,raw_vy,raw_wz,xy_error,yaw_error,collision,exception,success,motion_mode\n";
  auto initial=data["initial"];double x=initial[0].as<double>(),y=initial[1].as<double>(),yaw=initial[2].as<double>();
  const double dt=data["dt"].as<double>(.1); const int steps=data["steps"].as<int>(1800);
  geometry_msgs::msg::Twist velocity;
  if(data["initial_velocity"]) {velocity.linear.x=data["initial_velocity"][0].as<double>();velocity.linear.y=data["initial_velocity"][1].as<double>();velocity.angular.z=data["initial_velocity"][2].as<double>();}
  nav_msgs::msg::Path path;path.header.frame_id="odom";
  for(auto point:data["path"]) {geometry_msgs::msg::PoseStamped p;p.header.frame_id="odom";p.pose=pose(point[0].as<double>(),point[1].as<double>(),point[2].as<double>());path.poses.push_back(p);}
  controller->setPlan(path);
  auto goal=path.poses.back().pose;
  const double goal_yaw=2*std::atan2(goal.orientation.z,goal.orientation.w);

  int status=1;
  for(int step=0;step<steps;++step) {
    const double t=data["shadow_stamps"] ? data["shadow_stamps"][step].as<double>()-data["shadow_stamps"][0].as<double>() : step*dt;
    if(rcl_set_ros_time_override(clock->get_clock_handle(),static_cast<int64_t>(((data["shadow_stamps"] ? data["shadow_stamps"][step].as<double>() : 1000+t)+(data["reset_step"] && step>=data["reset_step"].as<int>() ? 5.0 : 0.0))*1e9))!=RCL_RET_OK) return 3;
    if(data["speed_limit_step"] && step==data["speed_limit_step"].as<int>()) controller->setSpeedLimit(data["speed_limit"].as<double>(),data["percentage"].as<bool>(true));
    if(data["dynamic_step"] && step==data["dynamic_step"].as<int>()) {
      node->set_parameters({rclcpp::Parameter("FollowPath.vx_max",.3),rclcpp::Parameter("FollowPath.wz_max",.2)});
    }
    if(data["reset_step"] && step==data["reset_step"].as<int>()) {controller->setPlan(path);}
    if(data["perturb_step"] && step==data["perturb_step"].as<int>()) x+=data["perturb_x"].as<double>();
    if(data["shadow"]) {auto row=data["shadow"][step];x=row[0].as<double>();y=row[1].as<double>();yaw=row[2].as<double>();velocity.linear.x=row[3].as<double>();velocity.linear.y=row[4].as<double>();velocity.angular.z=row[5].as<double>();}
    geometry_msgs::msg::PoseStamped query;query.header.frame_id="odom";query.header.stamp=clock->now();query.pose=pose(x,y,yaw);
    geometry_msgs::msg::Twist raw;std::string exception;
    try {raw=controller->computeVelocityCommands(query,velocity,checker.get()).twist;}
    catch(const std::exception&e) {exception=e.what();std::replace(exception.begin(),exception.end(),',',';');}
    rclcpp::spin_some(node->get_node_base_interface());
    auto actual=raw;
    if(data["deadband"].as<bool>(true)) {
      if(std::abs(actual.linear.x)<.05)actual.linear.x=0;
      if(std::abs(actual.linear.y)<.10)actual.linear.y=0;
      if(std::abs(actual.angular.z)<.02)actual.angular.z=0;
    }
    actual.linear.x=std::clamp(actual.linear.x,-.15,.15);actual.linear.y=std::clamp(actual.linear.y,-.15,.15);actual.angular.z=std::clamp(actual.angular.z,-.1,.1);
    double nx=x,ny=y,na=yaw;bool collision=false;
    // At dt=.1, check every .01 s (<=2.13 mm translation / .001 rad yaw).
    for(int sub=0;sub<10;++sub) {
      nx+=(std::cos(na)*actual.linear.x-std::sin(na)*actual.linear.y)*dt/10;
      ny+=(std::sin(na)*actual.linear.x+std::cos(na)*actual.linear.y)*dt/10;
      na=angle(na+actual.angular.z*dt/10);
      if(collides(costmap,nx,ny,na)) {collision=true;break;}
    }
    double xy=std::hypot(goal.position.x-x,goal.position.y-y),ye=std::abs(angle(goal_yaw-yaw));
    bool success=!collision && exception.empty() && checker->isGoalReached(query.pose,goal,velocity) &&
      std::hypot(actual.linear.x,actual.linear.y)<=.01 && std::abs(actual.angular.z)<=.01;
    out<<step<<','<<t<<','<<x<<','<<y<<','<<yaw<<','<<actual.linear.x<<','<<actual.linear.y<<','<<actual.angular.z<<','<<raw.linear.x<<','<<raw.linear.y<<','<<raw.angular.z<<','<<xy<<','<<ye<<','<<collision<<','<<exception<<','<<success<<','<<motion_mode<<'\n';
    if(collision) {status=4;break;}
    if(!data["shadow"]) status=success ? 0 : 1;
    if(!data["shadow"] && success && !data["perturb_step"]) {status=0;break;}
    x=nx;y=ny;yaw=na;velocity=actual;
    if(!exception.empty() && data["stop_on_exception"].as<bool>(false)) {status=5;break;}
  }
  controller->deactivate();controller->cleanup();controller.reset();
  checker.reset();
  cm->on_cleanup(rclcpp_lifecycle::State());rclcpp::shutdown();return status;
}
