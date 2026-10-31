#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <angles/angles.h>

geometry_msgs::msg::PoseStamped pose(double x,double y,double yaw) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="probe_map";
  p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=std::cos(yaw/2);p.pose.orientation.z=std::sin(yaw/2);return p;
}
struct Scene{std::string name;geometry_msgs::msg::PoseStamped start,goal;bool office;};
int main(int argc,char ** argv) {
  if(argc<2)return 64;const std::string root=argv[1];rclcpp::init(argc,argv);
  try {
    const std::string only=std::getenv("PROBE_PLANNER")?std::getenv("PROBE_PLANNER"):"";
    const std::string tag=std::getenv("PROBE_GLOBAL_TAG")?std::getenv("PROBE_GLOBAL_TAG"):"";
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> params;
    for(std::string name:{"Hybrid","Lattice","LatticeActions"}) {
      params.emplace_back(name+".allow_unknown",false);
      params.emplace_back(name+".smooth_path",false);
      params.emplace_back(name+".cache_obstacle_heuristic",false);
      params.emplace_back(name+".lookup_table_size",1.0);
      params.emplace_back(name+".max_planning_time",3.0);
      params.emplace_back(name+".max_iterations",500000);
      params.emplace_back(name+".tolerance",std::getenv("PROBE_GLOBAL_TOLERANCE")?std::stod(std::getenv("PROBE_GLOBAL_TOLERANCE")):0.01);
      if(name=="Hybrid") {
        params.emplace_back(name+".motion_model_for_search","DUBIN");
        params.emplace_back(name+".minimum_turning_radius",0.5);
        params.emplace_back(name+".angle_quantization_bins",72);
        params.emplace_back(name+".downsample_costmap",false);
        params.emplace_back(name+".downsample_obstacle_heuristic",false);
      } else {
        params.emplace_back(name+".lattice_filepath",std::getenv("PROBE_LATTICE_FILE")?
          std::getenv("PROBE_LATTICE_FILE"):root+"/inputs/global_planner/stop_turn_1cm_h16.json");
        params.emplace_back(name+".analytic_expansion_max_length",name=="LatticeActions"?0.001:3.0);
        params.emplace_back(name+".allow_reverse_expansion",std::getenv("PROBE_REVERSE")!=nullptr);
        if(std::getenv("PROBE_ROTATION_PENALTY"))params.emplace_back(name+".rotation_penalty",std::stod(std::getenv("PROBE_ROTATION_PENALTY")));
      }
    }
    options.parameter_overrides(params);options.automatically_declare_parameters_from_overrides(true);
    auto parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("global_planner_probe","/official_global_probe",options);
    rclcpp::NodeOptions map_options;
    map_options.arguments({"--ros-args","-r","__node:=global_probe_costmap","-r","__ns:=/official_global_probe"});
    map_options.parameter_overrides({rclcpp::Parameter("global_frame","probe_map"),
      rclcpp::Parameter("robot_base_frame","probe_base_link"),rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("footprint","[[0.30,0.15],[0.30,-0.15],[-0.30,-0.15],[-0.30,0.15]]"),
      rclcpp::Parameter("footprint_padding",0.035),rclcpp::Parameter("resolution",0.01),
      rclcpp::Parameter("probe_inflation.inflation_radius",0.4),rclcpp::Parameter("probe_inflation.cost_scaling_factor",3.0)});
    auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(map_options);map->configure();
    nav_msgs::msg::OccupancyGrid office;
    if(nav2_map_server::loadMapFromYaml(root+"/inputs/global_planner/map.yaml",office)!=nav2_map_server::LOAD_MAP_SUCCESS)
      throw std::runtime_error("Office map load failed");
    if(std::abs(tf2::getYaw(office.info.origin.orientation))>1e-9)throw std::runtime_error("Rotated map origin not supported in fixture");
    pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
    const std::vector<Scene> scenes={
      {"free_same_xy_180",pose(0,0,0),pose(0,0,M_PI),false},
      {"free_near_110",pose(0,0,0),pose(.2,0,110*M_PI/180),false},
      {"synthetic_door_goal90",pose(0,0,15*M_PI/180),pose(1.5,0,M_PI/2),false},
      {"synthetic_l_corridor",pose(0,0,0),pose(1,1,M_PI/2),false},
      {"recorded_terminal110_static_map",pose(-3.8007497322782706,4.658441093749194,3.0504716917604155),
        pose(-3.789631256533969,4.698352909931362,1.1210367137250565),true}};
    for(const auto & scene:scenes) {
      auto layered=map->getLayeredCostmap();
      if(scene.office)layered->resizeMap(office.info.width,office.info.height,office.info.resolution,
        office.info.origin.position.x,office.info.origin.position.y,true);
      else layered->resizeMap(1200,1200,.01,-6,-6,true);
      auto grid=map->getCostmap();const auto size=grid->getSizeInCellsX()*grid->getSizeInCellsY();
      if(scene.office)for(unsigned i=0;i<size;++i){const int value=office.data[i];grid->getCharMap()[i]=value<0?255:(value>=65?254:0);}
      else std::fill(grid->getCharMap(),grid->getCharMap()+size,0);
      if(scene.name=="synthetic_door_goal90" || scene.name=="synthetic_l_corridor"){
        for(unsigned x=0;x<grid->getSizeInCellsX();++x)for(unsigned y=0;y<grid->getSizeInCellsY();++y){
          double wx,wy;grid->mapToWorld(x,y,wx,wy);
          if(scene.name=="synthetic_door_goal90"){
            if(wx>=.65 && wx<=.75 && std::abs(wy)>.23)grid->setCost(x,y,254);
          }else{
            const bool horizontal=wx>=-.6 && wx<=1.45 && std::abs(wy)<=.24;
            const bool vertical=std::abs(wx-1)<=.24 && wy>=-.45 && wy<=1.6;
            const bool turn_room=std::hypot(wx-1,wy)<=.5;
            if(!(horizontal || vertical || turn_room))grid->setCost(x,y,254);
          }
        }
      }
      auto inflation=std::make_shared<nav2_costmap_2d::InflationLayer>();
      auto group=map->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      inflation->initialize(layered,"probe_inflation",map->getTfBuffer().get(),map,group);
      inflation->matchSize();std::static_pointer_cast<nav2_costmap_2d::Layer>(inflation)->onFootprintChanged();
      inflation->updateCosts(*grid,0,0,grid->getSizeInCellsX(),grid->getSizeInCellsY());
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> body(grid);
      for(const std::string name:{"Hybrid","Lattice","LatticeActions"}) {
        if(!only.empty() && only!=name)continue;
        const auto start=std::chrono::steady_clock::now();
        auto planner=loader.createUniqueInstance(name=="Hybrid"?"nav2_smac_planner::SmacPlannerHybrid":"nav2_smac_planner::SmacPlannerLattice");
        try {
          planner->configure(parent,name,map->getTfBuffer(),map);planner->activate();
          const auto result=planner->createPlan(scene.start,scene.goal,[&](){return std::chrono::steady_clock::now()-start>std::chrono::seconds(5);});
          double length=0,rotation=0,turn_in_place=0,max_excursion=0;size_t collisions=0,samples=0;
          std::ofstream csv(root+"/evidence/global_"+scene.name+"_"+name+tag+".csv");csv<<"x,y,yaw\n"<<std::setprecision(16);
          for(size_t i=0;i<result.poses.size();++i) {
            const auto & p=result.poses[i];const double yaw=tf2::getYaw(p.pose.orientation);
            csv<<p.pose.position.x<<','<<p.pose.position.y<<','<<yaw<<'\n';
            max_excursion=std::max(max_excursion,std::hypot(p.pose.position.x-scene.start.pose.position.x,p.pose.position.y-scene.start.pose.position.y));
            if(!i)continue;const auto & prev=result.poses[i-1];
            const double dx=p.pose.position.x-prev.pose.position.x,dy=p.pose.position.y-prev.pose.position.y;
            const double ds=std::hypot(dx,dy),old_yaw=tf2::getYaw(prev.pose.orientation);
            const double da=angles::shortest_angular_distance(old_yaw,yaw);
            length+=ds;rotation+=std::abs(da);if(ds<1e-4)turn_in_place+=std::abs(da);
            const int n=std::max(1,(int)std::ceil(std::max(ds/(grid->getResolution()/2),std::abs(da)/(M_PI/180))));
            for(int j=0;j<=n;++j){double t=(double)j/n;
              const double c=body.footprintCostAtPose(prev.pose.position.x+t*dx,prev.pose.position.y+t*dy,old_yaw+t*da,map->getRobotFootprint());
              ++samples;if(c<0 || c>=254)++collisions;
            }
          }
          const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
          std::cout<<"{\"scene\":\""<<scene.name<<"\",\"planner\":\""<<name<<"\",\"poses\":"<<result.poses.size()
            <<",\"variant\":"<<std::quoted(tag)
            <<",\"path_length_m\":"<<length<<",\"max_excursion_m\":"<<max_excursion
            <<",\"inplace_rotation_deg\":"<<turn_in_place*180/M_PI<<",\"body_collision_samples\":"<<collisions
            <<",\"body_samples\":"<<samples<<",\"elapsed_s\":"<<seconds<<"}\n";
          planner->deactivate();planner->cleanup();
        }catch(const std::exception & ex){std::cout<<"{\"scene\":\""<<scene.name<<"\",\"planner\":\""<<name<<"\",\"error\":"<<std::quoted(ex.what())<<"}\n";}
      }
      inflation.reset();
    }
    map->cleanup();
  }catch(const std::exception & ex){std::cerr<<"PROBE_ERROR "<<ex.what()<<'\n';rclcpp::shutdown();return 3;}
  rclcpp::shutdown();return 0;
}
