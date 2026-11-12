#include "official_bridge.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <nav2_map_server/map_io.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <angles/angles.h>

// Local integration fixture only. No velocity publisher or hardware connection.
int main(int argc,char ** argv) {
  if(argc<10)return 64;
  const std::string root=argv[1],csv_path=argv[2];
  auto current=pose(std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5]));
  const auto requested=pose(std::stod(argv[6]),std::stod(argv[7]),std::stod(argv[8]));
  const bool office=std::string(argv[9])=="office";
  rclcpp::init(argc,argv);int status=0;
  try {
    nav2_msgs::action::FollowPath::Goal request;request.path.header.frame_id="probe_map";
    request.controller_id="RPP";request.goal_checker_id="Checker";
    std::ifstream csv(csv_path);if(!csv)throw std::runtime_error("CSV not readable");
    std::string line;std::getline(csv,line);
    while(std::getline(csv,line)){
      std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
      double x,y,a;if(!(row>>x>>y>>a))throw std::runtime_error("Invalid CSV pose");
      request.path.poses.push_back(pose(x,y,a));
    }
    if(request.path.poses.empty())throw std::runtime_error("CSV has no poses");
    // Explicit diagnostic adaptation: keep the planner's terminal XY and preserve
    // the user's exact yaw after lattice heading discretization. Swept safety is
    // checked below; this is not attributed to the unmodified planner output.
    if(std::getenv("PROBE_APPEND_REQUESTED_GOAL")){
      auto terminal=request.path.poses.back();terminal.pose.orientation=requested.pose.orientation;
      request.path.poses.push_back(terminal);
    }
    const double goal_xy=std::getenv("PROBE_GOAL_XY")?std::stod(std::getenv("PROBE_GOAL_XY")):0.04;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("controller_frequency",10.0),
      rclcpp::Parameter("PathHandler.enforce_path_rotation",true),
      rclcpp::Parameter("PathHandler.minimum_rotation_angle",0.0872665),
      rclcpp::Parameter("PathHandler.inversion_xy_tolerance",goal_xy+0.01),
      rclcpp::Parameter("PathHandler.inversion_yaw_tolerance",0.03),
      rclcpp::Parameter("RPP.stateful",false),
      rclcpp::Parameter("RPP.desired_linear_vel",0.15),
      rclcpp::Parameter("RPP.lookahead_dist",0.3),
      rclcpp::Parameter("RPP.use_rotate_to_heading",true),
      rclcpp::Parameter("RPP.rotate_to_heading_min_angle",0.0872665),
      rclcpp::Parameter("RPP.rotate_to_heading_angular_vel",std::getenv("PROBE_ROTATE_SPEED")?std::stod(std::getenv("PROBE_ROTATE_SPEED")):0.5),
      rclcpp::Parameter("RPP.max_angular_accel",1.0),
      rclcpp::Parameter("RPP.min_approach_linear_velocity",0.1),
      rclcpp::Parameter("RPP.regulated_linear_scaling_min_speed",0.1),
      rclcpp::Parameter("RPP.use_cost_regulated_linear_velocity_scaling",false),
      rclcpp::Parameter("Checker.stateful",false),
      rclcpp::Parameter("Checker.xy_goal_tolerance",goal_xy),
      rclcpp::Parameter("Checker.yaw_goal_tolerance",0.05),
      rclcpp::Parameter("Checker.path_length_tolerance",goal_xy+0.02)});
    options.automatically_declare_parameters_from_overrides(true);
    auto parent=std::make_shared<nav2::LifecycleNode>("planned_path_controller_probe","/official_path_probe",options);
    rclcpp::NodeOptions mo;
    mo.arguments({"--ros-args","-r","__node:=planned_path_costmap","-r","__ns:=/official_path_probe"});
    mo.parameter_overrides({rclcpp::Parameter("global_frame","probe_map"),
      rclcpp::Parameter("robot_base_frame","probe_base_link"),
      rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("footprint","[[0.30,0.15],[0.30,-0.15],[-0.30,-0.15],[-0.30,0.15]]"),
      rclcpp::Parameter("footprint_padding",0.035),rclcpp::Parameter("resolution",0.01),
      rclcpp::Parameter("probe_inflation.inflation_radius",0.4),
      rclcpp::Parameter("probe_inflation.cost_scaling_factor",3.0)});
    auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(mo);map->configure();
    auto layered=map->getLayeredCostmap();auto grid=map->getCostmap();
    if(office){
      nav_msgs::msg::OccupancyGrid occ;
      if(nav2_map_server::loadMapFromYaml(root+"/inputs/global_planner/map.yaml",occ)!=nav2_map_server::LOAD_MAP_SUCCESS)
        throw std::runtime_error("Office map load failed");
      layered->resizeMap(occ.info.width,occ.info.height,occ.info.resolution,occ.info.origin.position.x,occ.info.origin.position.y,true);
      for(unsigned i=0;i<occ.data.size();++i){int v=occ.data[i];grid->getCharMap()[i]=v<0?255:(v>=65?254:0);}
    }else{
      layered->resizeMap(1200,1200,.01,-6,-6,true);
      std::fill(grid->getCharMap(),grid->getCharMap()+1440000,0);
      if(std::string(argv[9])=="door" || std::string(argv[9])=="corridor"){
        for(unsigned x=0;x<1200;++x)for(unsigned y=0;y<1200;++y){
          double wx,wy;grid->mapToWorld(x,y,wx,wy);
          if(std::string(argv[9])=="door"){
            if(wx>=.65 && wx<=.75 && std::abs(wy)>.23)grid->setCost(x,y,254);
          }else{
            const bool horizontal=wx>=-.6 && wx<=1.45 && std::abs(wy)<=.24;
            const bool vertical=std::abs(wx-1)<=.24 && wy>=-.45 && wy<=1.6;
            const bool turn_room=std::hypot(wx-1,wy)<=.5;
            if(!(horizontal || vertical || turn_room))grid->setCost(x,y,254);
          }
        }
      }
    }
    auto inflation=std::make_shared<nav2_costmap_2d::InflationLayer>();
    auto group=map->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    inflation->initialize(layered,"probe_inflation",map->getTfBuffer().get(),map,group);
    inflation->matchSize();std::static_pointer_cast<nav2_costmap_2d::Layer>(inflation)->onFootprintChanged();
    inflation->updateCosts(*grid,0,0,grid->getSizeInCellsX(),grid->getSizeInCellsY());
    LegacyGoalBridge bridge;auto tf=map->getTfBuffer();
    bridge.handler.initialize(parent,parent->get_logger(),"PathHandler",map,tf);
    bridge.checker.initialize(parent,"Checker",map);
    bridge.controller.configure(parent,"RPP",tf,map);bridge.controller.activate();bridge.submit(request);
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> body(grid);
    auto body_safe=[&](double x,double y,double a){
      const double c=body.footprintCostAtPose(x,y,a,map->getRobotFootprint());return c>=0 && c<254;
    };
    geometry_msgs::msg::Twist speed;double travelled=0,max_excursion=0;
    const auto initial=current;bool reached=false,safe=true;std::string error;
    int tick=0;
    for(;tick<1600 && !reached;++tick){
      const double yaw=tf2::getYaw(current.pose.orientation);
      if(!body_safe(current.pose.position.x,current.pose.position.y,yaw)){safe=false;error="Body collision at pose";break;}
      geometry_msgs::msg::TransformStamped t;t.header.frame_id="probe_map";t.child_frame_id="probe_base_link";
      t.transform.translation.x=current.pose.position.x;t.transform.translation.y=current.pose.position.y;
      t.transform.rotation=current.pose.orientation;tf->setTransform(t,"isolated_ideal_body",true);
      geometry_msgs::msg::Twist cmd;
      try{cmd=bridge.step(current,speed,reached);}catch(const std::exception & ex){error=ex.what();break;}
      if(!std::isfinite(cmd.linear.x)||!std::isfinite(cmd.angular.z)){safe=false;error="Nonfinite command";break;}
      if(tick%20==0 || reached)std::cout<<"{\"tick\":"<<tick<<",\"x\":"<<current.pose.position.x
        <<",\"y\":"<<current.pose.position.y<<",\"yaw_deg\":"<<yaw*180/M_PI
        <<",\"vx\":"<<cmd.linear.x<<",\"wz\":"<<cmd.angular.z<<",\"segment_poses\":"<<bridge.segment.poses.size()<<"}\n";
      if(reached)break;
      const double dt=.1;
      // Dense sweep along the exact constant-twist arc, including rotation in place.
      const int n=std::max(1,(int)std::ceil(std::max(std::abs(cmd.linear.x)*dt/.005,std::abs(cmd.angular.z)*dt/(M_PI/180))));
      auto integrate=[&](double elapsed){
        const double a=yaw+cmd.angular.z*elapsed;
        if(std::abs(cmd.angular.z)>1e-9)return pose(current.pose.position.x+cmd.linear.x/cmd.angular.z*(std::sin(a)-std::sin(yaw)),
          current.pose.position.y-cmd.linear.x/cmd.angular.z*(std::cos(a)-std::cos(yaw)),a);
        return pose(current.pose.position.x+cmd.linear.x*std::cos(yaw)*elapsed,current.pose.position.y+cmd.linear.x*std::sin(yaw)*elapsed,a);
      };
      for(int j=1;j<=n;++j){auto p=integrate(dt*j/n);if(!body_safe(p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation))){safe=false;error="Body collision during velocity sweep";break;}}
      if(!safe)break;
      current=integrate(dt);speed=cmd;travelled+=std::abs(cmd.linear.x)*dt;
      max_excursion=std::max(max_excursion,std::hypot(current.pose.position.x-initial.pose.position.x,current.pose.position.y-initial.pose.position.y));
    }
    const double pos_err=std::hypot(current.pose.position.x-requested.pose.position.x,current.pose.position.y-requested.pose.position.y);
    const double yaw_err=std::abs(angles::shortest_angular_distance(tf2::getYaw(current.pose.orientation),tf2::getYaw(requested.pose.orientation)));
    const bool passed=reached && safe && pos_err<=goal_xy && yaw_err<=.05;
    std::cout<<std::setprecision(12)<<"{\"planner_controller_summary\":true,\"success\":"<<passed
      <<",\"checker_reached\":"<<reached<<",\"body_safe\":"<<safe<<",\"ticks\":"<<tick
      <<",\"travelled_m\":"<<travelled<<",\"max_excursion_m\":"<<max_excursion
      <<",\"request_position_error_m\":"<<pos_err<<",\"request_yaw_error_deg\":"<<yaw_err*180/M_PI
      <<",\"error\":"<<std::quoted(error)<<"}\n";
    if(!passed)status=2;
    bridge.cancel();bridge.controller.deactivate();bridge.controller.cleanup();map->cleanup();
  }catch(const std::exception & ex){std::cerr<<"PROBE_ERROR "<<ex.what()<<'\n';status=3;}
  rclcpp::shutdown();return status;
}
