// Offline diagnostic only: production optimizer and critic plugins, no actuator publishers.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <xtensor/xrandom.hpp>
#include <navigo_mppi_controller/optimizer.hpp>
#include <navigo_core/goal_checker.hpp>
#include <pluginlib/class_loader.hpp>

using Term = std::pair<std::string, xt::xtensor<float, 1>>;
class ScoreProbe : public mppi::CriticManager {
 public:
  std::vector<std::string> skipped;
  std::vector<Term> score(mppi::CriticData & data) {
    skipped.clear();
    std::vector<Term> terms;
    for (size_t i=0; i<critics_.size(); ++i) {
      const auto before=data.costs;
      if (!data.fail_flag) critics_[i]->score(data);
      else skipped.push_back(critic_names_[i]);
      terms.emplace_back(critic_names_[i], xt::eval(data.costs-before));
    }
    return terms;
  }
};

class OptimizerProbe : public mppi::Optimizer {
 public:
  geometry_msgs::msg::Twist nativeControl(const geometry_msgs::msg::PoseStamped & pose,
           const geometry_msgs::msg::Twist & speed, const nav_msgs::msg::Path & path,
           const geometry_msgs::msg::Pose & goal, navigo_core::GoalChecker * checker,
           const std::string & warm) {
    if (warm=="measured") {
      control_sequence_.vx.fill(std::clamp(float(speed.linear.x),settings_.constraints.vx_min,settings_.constraints.vx_max));
      control_sequence_.vy.fill(std::clamp(float(speed.linear.y),-settings_.constraints.vy,settings_.constraints.vy));
      control_sequence_.wz.fill(std::clamp(float(speed.angular.z),-settings_.constraints.wz,settings_.constraints.wz));
    } else if (warm!="zero") throw std::runtime_error("Unknown warm start");
    return evalControl(pose,speed,path,goal,checker).twist;
  }
  void run(const geometry_msgs::msg::PoseStamped & pose,
           const geometry_msgs::msg::Twist & speed, const nav_msgs::msg::Path & path,
           const geometry_msgs::msg::Pose & goal, navigo_core::GoalChecker * checker,
           ScoreProbe & scorer, const YAML::Node & request, const std::string & output) {
    if (settings_.iteration_count != 1 || settings_.forward_alignment || !isHolonomic())
      throw std::runtime_error("Audit requires one iteration, legacy strategy, Omni model");
    prepare(pose,speed,path,goal,checker);
    const std::string warm=request["warm_start"].as<std::string>();
    if (warm=="measured") {
      control_sequence_.vx.fill(std::clamp(float(speed.linear.x),settings_.constraints.vx_min,settings_.constraints.vx_max));
      control_sequence_.vy.fill(std::clamp(float(speed.linear.y),-settings_.constraints.vy,settings_.constraints.vy));
      control_sequence_.wz.fill(std::clamp(float(speed.angular.z),-settings_.constraints.wz,settings_.constraints.wz));
    } else if (warm!="zero") throw std::runtime_error("Unknown warm-start assumption");
    generateNoisedTrajectories();
    const bool fixed=request["fixed_proposals"].as<bool>(false);
    const std::vector<std::string> labels={"stop","reverse","forward","turn_left","turn_right","recorded_command"};
    if (fixed) {
      for (size_t i=0; i<settings_.batch_size; ++i) {
        double vx=0,vy=0,wz=0;
        switch(i%labels.size()) {
          case 1: vx=-.1; break;
          case 2: vx=.1; break;
          case 3: wz=.1; break;
          case 4: wz=-.1; break;
          case 5: vx=request["recorded_command"][0].as<double>();
            vy=request["recorded_command"][1].as<double>();
            wz=request["recorded_command"][2].as<double>(); break;
        }
        for (size_t t=0;t<settings_.time_steps;++t) {
          state_.cvx(i,t)=std::clamp(float(vx),settings_.constraints.vx_min,settings_.constraints.vx_max);
          state_.cvy(i,t)=std::clamp(float(vy),-settings_.constraints.vy,settings_.constraints.vy);
          state_.cwz(i,t)=std::clamp(float(wz),-settings_.constraints.wz,settings_.constraints.wz);
        }
      }
      updateStateVelocities(state_);
      integrateStateVelocities(generated_trajectories_,state_);
    }
    auto terms=scorer.score(critics_data_);
    const auto critic_costs=costs_;
    const bool failed=critics_data_.fail_flag;
    // Compare decomposition with the actual manager on identical rollouts.
    costs_.fill(0);critics_data_.fail_flag=false;
    critics_data_.furthest_reached_path_point.reset();critics_data_.path_pts_valid.reset();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    const double discrepancy=xt::amax(xt::abs(costs_-critic_costs))();
    if (discrepancy>1e-5 || failed!=critics_data_.fail_flag)
      throw std::runtime_error("Per-critic trace differs from production manager");
    updateControlSequence();
    const auto total=costs_;
    const auto regularization=xt::eval(total-critic_costs);
    const auto exp=xt::eval(xt::exp(-(total-xt::amin(total)())/settings_.temperature));
    const auto weights=xt::eval(exp/xt::sum(exp)());
    std::ofstream out(output+".csv");
    if (!out) throw std::runtime_error("Cannot write trace");
    out<<std::setprecision(10)<<"sample,proposal,mean_control_vx,first_control_vx,mean_predicted_vx,reverse_distance,forward_distance,end_x,end_y,end_yaw";
    for (const auto & term:terms) out<<","<<term.first;
    out<<",regularization,total,weight\n";
    for (size_t i=0;i<settings_.batch_size;++i) {
      double control=0,predicted=0,back=0,front=0;
      for (size_t t=0;t<settings_.time_steps;++t) {
        control+=state_.cvx(i,t);predicted+=state_.vx(i,t);
        back+=std::max(0.f,-state_.vx(i,t))*settings_.model_dt;
        front+=std::max(0.f,state_.vx(i,t))*settings_.model_dt;
      }
      const size_t end=settings_.time_steps-1;
      out<<i<<","<<(fixed?labels[i%labels.size()]:"sampled")<<","<<control/settings_.time_steps
         <<","<<state_.cvx(i,settings_.shift_control_sequence?1:0)<<","<<predicted/settings_.time_steps
         <<","<<back<<","<<front<<","<<generated_trajectories_.x(i,end)
         <<","<<generated_trajectories_.y(i,end)<<","<<generated_trajectories_.yaws(i,end);
      for (const auto & term:terms) out<<","<<term.second(i);
      out<<","<<regularization(i)<<","<<total(i)<<","<<weights(i)<<"\n";
    }
    mppi::utils::savitskyGolayFilter(control_sequence_,control_history_,settings_);
    const auto command=getControlFromSequenceAsTwist(path.header.stamp).twist;
    YAML::Node result;
    result["production_manager_max_difference"]=discrepancy;
    result["skipped_after_failure"]=scorer.skipped;
    if (critics_data_.furthest_reached_path_point)
      result["batch_furthest_path_index"]=*critics_data_.furthest_reached_path_point;
    result["all_trajectories_collide"]=failed;
    result["fallback_not_executed"]=true;
    result["command_valid"]=!failed && !fixed;
    result["diagnostic_command"].push_back(command.linear.x);
    result["diagnostic_command"].push_back(command.linear.y);
    result["diagnostic_command"].push_back(command.angular.z);
    result["goal_distance"]=std::hypot(goal.position.x-pose.pose.position.x,goal.position.y-pose.pose.position.y);
    result["path_end_distance"]=std::hypot(path.poses.back().pose.position.x-pose.pose.position.x,path.poses.back().pose.position.y-pose.pose.position.y);
    result["sample_count"]=settings_.batch_size;
    result["dt"]=settings_.model_dt;result["time_steps"]=settings_.time_steps;
    result["temperature"]=settings_.temperature;result["gamma"]=settings_.gamma;
    std::ofstream(output+".yaml")<<result;
  }
};

geometry_msgs::msg::Pose readPose(const YAML::Node & row) {
  geometry_msgs::msg::Pose pose;
  pose.position.x=row[0].as<double>();pose.position.y=row[1].as<double>();
  const double yaw=row[2].as<double>();pose.orientation.z=std::sin(yaw/2);pose.orientation.w=std::cos(yaw/2);
  return pose;
}

int main(int argc,char **argv) {
  if (argc<4) {std::cerr<<"cost_probe CASE.yaml PARAMS.yaml OUTPUT_PREFIX\n";return 2;}
  if (std::filesystem::exists(std::string(argv[3])+".yaml") ||
      std::filesystem::exists(std::string(argv[3])+".csv")) {
    std::cerr<<"Refusing to overwrite input or prior trace\n";return 2;
  }
  const auto request=YAML::LoadFile(argv[1]);
  rclcpp::init(argc,argv);xt::random::seed(request["seed"].as<unsigned>());
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",rclcpp::NodeOptions().arguments({"--ros-args","--params-file",argv[2]}));
  auto cm=std::make_shared<navigo_costmap_2d::Costmap2DROS>("audit_costmap","","audit_costmap");
  const bool inflation=request["load_inflation_layer"].as<bool>(false);
  cm->set_parameter(rclcpp::Parameter("plugins",inflation?std::vector<std::string>{"inflation_layer"}:std::vector<std::string>{}));
  if (inflation) {
    cm->declare_parameter("inflation_layer.plugin",std::string("navigo_costmap_2d::InflationLayer"));
    cm->declare_parameter("inflation_layer.inflation_radius",request["inflation_radius"].as<double>());
    cm->declare_parameter("inflation_layer.cost_scaling_factor",request["cost_scaling_factor"].as<double>());
  }
  cm->set_parameter(rclcpp::Parameter("global_frame",std::string("odom")));
  cm->set_parameter(rclcpp::Parameter("footprint",request["footprint"].as<std::string>()));
  cm->set_parameter(rclcpp::Parameter("footprint_padding",request["footprint_padding"].as<double>()));
  cm->on_configure(rclcpp_lifecycle::State());
  auto *map=cm->getCostmap();auto spec=request["map"];
  map->resizeMap(spec["width"].as<unsigned>(),spec["height"].as<unsigned>(),spec["resolution"].as<double>(),spec["origin_x"].as<double>(),spec["origin_y"].as<double>());
  std::ifstream bytes(spec["data"].as<std::string>(),std::ios::binary);
  bytes.read(reinterpret_cast<char*>(map->getCharMap()),map->getSizeInCellsX()*map->getSizeInCellsY());
  if (!bytes) throw std::runtime_error("Missing or short costmap");
  pluginlib::ClassLoader<navigo_core::GoalChecker> loader("navigo_core","navigo_core::GoalChecker");
  auto checker=loader.createSharedInstance("navigo_path_controller::StoppedGoalChecker");
  checker->initialize(node,"general_goal_checker",cm);
  mppi::ParametersHandler parameters(node);
  OptimizerProbe optimizer;optimizer.initialize(node,"FollowPath",cm,&parameters);
  ScoreProbe scorer;scorer.on_configure(node,"FollowPath",cm,&parameters);
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="odom";
  pose.header.stamp=rclcpp::Time(int64_t(request["source_stamp"].as<double>()*1e9));
  pose.pose=readPose(request["pose"]);
  geometry_msgs::msg::Twist speed;speed.linear.x=request["velocity"][0].as<double>();
  speed.linear.y=request["velocity"][1].as<double>();speed.angular.z=request["velocity"][2].as<double>();
  nav_msgs::msg::Path path;path.header=pose.header;
  for (auto row:request["path"]) {geometry_msgs::msg::PoseStamped p;p.header=pose.header;p.pose=readPose(row);path.poses.push_back(p);}
  if (path.poses.empty()) throw std::runtime_error("Missing recorded local reference");
  if (request["native_only"].as<bool>(false)) {
    const auto velocity=optimizer.nativeControl(pose,speed,path,readPose(request["goal"]),checker.get(),request["warm_start"].as<std::string>());
    YAML::Node result;result["command"].push_back(velocity.linear.x);result["command"].push_back(velocity.linear.y);result["command"].push_back(velocity.angular.z);
    std::ofstream(std::string(argv[3])+".yaml")<<result;
  } else {
    optimizer.run(pose,speed,path,readPose(request["goal"]),checker.get(),scorer,request,argv[3]);
  }
  optimizer.shutdown();cm->on_cleanup(rclcpp_lifecycle::State());rclcpp::shutdown();
}
