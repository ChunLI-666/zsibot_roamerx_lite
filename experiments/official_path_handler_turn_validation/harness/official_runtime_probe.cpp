#include "official_bridge.hpp"
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("controller_frequency",10.0),
      rclcpp::Parameter("PathHandler.enforce_path_rotation",true),
      rclcpp::Parameter("PathHandler.minimum_rotation_angle",0.0872665),
      rclcpp::Parameter("PathHandler.inversion_xy_tolerance",std::getenv("PROBE_XY_TOL")?std::stod(std::getenv("PROBE_XY_TOL")):0.05),
      rclcpp::Parameter("PathHandler.inversion_yaw_tolerance",0.03),
      rclcpp::Parameter("RPP.stateful",false),
      rclcpp::Parameter("RPP.desired_linear_vel",0.15),
      rclcpp::Parameter("RPP.lookahead_dist",0.3),
      rclcpp::Parameter("RPP.use_rotate_to_heading",true),
      rclcpp::Parameter("RPP.rotate_to_heading_min_angle",0.0872665),
      rclcpp::Parameter("RPP.rotate_to_heading_angular_vel",0.5),
      rclcpp::Parameter("RPP.max_angular_accel",1.0),
      rclcpp::Parameter("RPP.min_approach_linear_velocity",0.1),
      rclcpp::Parameter("RPP.regulated_linear_scaling_min_speed",0.1),
      rclcpp::Parameter("RPP.use_cost_regulated_linear_velocity_scaling",false),
      rclcpp::Parameter("Checker.stateful",false),
      rclcpp::Parameter("Checker.xy_goal_tolerance",0.04),
      rclcpp::Parameter("Checker.yaw_goal_tolerance",0.05),
      rclcpp::Parameter("Checker.path_length_tolerance",0.06)});
    options.automatically_declare_parameters_from_overrides(true);
    auto parent=std::make_shared<nav2::LifecycleNode>("official_controller_probe","/official_path_probe",options);
    auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "probe_costmap","/official_path_probe","local",false);
    map->set_parameters({rclcpp::Parameter("global_frame","probe_map"),
      rclcpp::Parameter("robot_base_frame","probe_base_link"),
      rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("resolution",0.01),rclcpp::Parameter("width",8),
      rclcpp::Parameter("height",8),rclcpp::Parameter("origin_x",-2.0),
      rclcpp::Parameter("origin_y",-2.0),rclcpp::Parameter("transform_tolerance",0.0)});
    map->configure();
    if(!map->getCostmap()) throw std::runtime_error("Native costmap configuration failed");
    map->getCostmap()->resizeMap(800,800,0.01,-2,-2);
    std::fill(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+640000,0);
    std::vector<geometry_msgs::msg::Point> footprint;
    for(auto [x,y]:std::vector<std::pair<double,double>>{{-.4,-.18},{.4,-.18},{.4,.18},{-.4,.18}}){
      geometry_msgs::msg::Point a;a.x=x;a.y=y;footprint.push_back(a);
    }
    map->setRobotFootprint(footprint);
    auto tf=map->getTfBuffer();
    LegacyGoalBridge bridge;
    bridge.handler.initialize(parent,parent->get_logger(),"PathHandler",map,tf);
    bridge.checker.initialize(parent,"Checker",map);
    bridge.controller.configure(parent,"RPP",tf,map);bridge.controller.activate();
    auto set_pose=[&](const geometry_msgs::msg::PoseStamped & p){
      geometry_msgs::msg::TransformStamped t;t.header.frame_id="probe_map";
      t.child_frame_id="probe_base_link";t.transform.translation.x=p.pose.position.x;
      t.transform.translation.y=p.pose.position.y;t.transform.rotation=p.pose.orientation;
      tf->setTransform(t,"isolated_software_body",true);
    };
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> body_check(map->getCostmap());
    for(std::string scenario:{"same_xy_180","near_terminal110","midway_90","door_straight","cancel_and_new_goal","blocked_rotation"}) {
      std::fill(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+640000,0);
      if(scenario=="blocked_rotation") {
        unsigned mx,my;map->getCostmap()->worldToMap(0,0.32,mx,my);
        map->getCostmap()->setCost(mx,my,254);
      }
      if(scenario=="door_straight") {
        for(unsigned x=0;x<800;++x)for(unsigned y=0;y<800;++y){
          double wx,wy;map->getCostmap()->mapToWorld(x,y,wx,wy);
          if(wx>=.65 && wx<=.75 && std::abs(wy)>.23)map->getCostmap()->setCost(x,y,254);
        }
      }
      nav2_msgs::action::FollowPath::Goal request;request.path.header.frame_id="probe_map";
      request.controller_id="RPP";request.goal_checker_id="Checker";
      if(scenario=="same_xy_180" || scenario=="blocked_rotation") for(int a=0;a<=180;a+=5)request.path.poses.push_back(pose(0,0,a*M_PI/180));
      else if(scenario=="near_terminal110") {
        for(int a=0;a<=4;++a)request.path.poses.push_back(pose(a*.05,0,0));
        for(int a=5;a<=110;a+=5)request.path.poses.push_back(pose(.2,0,a*M_PI/180));
      }
      else if(scenario=="door_straight") {
        for(int a=15;a>=0;a-=5)request.path.poses.push_back(pose(0,0,a*M_PI/180));
        for(int a=1;a<=30;++a)request.path.poses.push_back(pose(a*.05,0,0));
        for(int a=5;a<=90;a+=5)request.path.poses.push_back(pose(1.5,0,a*M_PI/180));
      }
      else {
        for(int a=0;a<=20;++a)request.path.poses.push_back(pose(a*.05,0,0));
        for(int a=5;a<=90;a+=5)request.path.poses.push_back(pose(1,0,a*M_PI/180));
        for(int a=1;a<=20;++a)request.path.poses.push_back(pose(1,a*.05,M_PI/2));
      }
      auto current=pose(0,0,scenario=="door_straight"?15*M_PI/180:0);geometry_msgs::msg::Twist speed;
      bridge.submit(request);bool success=false,rejected=false,seen_corner_rotation=false;
      double first_depart_yaw=-1,resume_yaw=-1;
      for(int tick=0;tick<600;++tick){
        set_pose(current);bool reached=false;geometry_msgs::msg::Twist cmd;
        try {cmd=bridge.step(current,speed,reached);}
        catch(const nav2_core::NoValidControl & ex) {
          if(scenario!="blocked_rotation")throw;
          rejected=true;std::cout<<"{\"scenario\":\"blocked_rotation\",\"rejected\":true,\"tick\":"<<tick<<"}\n";break;
        }
        if(scenario=="cancel_and_new_goal" && tick==5){
          bridge.cancel();bool ignored=false;auto stopped=bridge.step(current,speed,ignored);
          if(stopped.linear.x!=0 || stopped.angular.z!=0)throw std::runtime_error("cancel not zero");
          std::cout<<"{\"scenario\":\"cancel\",\"zero_after_cancel\":true}\n";
          const double yaw=tf2::getYaw(current.pose.orientation);request.path.poses.clear();
          for(int a=0;a<=90;a+=5)request.path.poses.push_back(pose(current.pose.position.x,current.pose.position.y,yaw+a*M_PI/180));
          bridge.submit(request);speed=geometry_msgs::msg::Twist();continue;
        }
        const double yaw=tf2::getYaw(current.pose.orientation);
        if(scenario=="midway_90" && current.pose.position.x>.9) {
          if(cmd.linear.x==0 && std::abs(cmd.angular.z)>.01)seen_corner_rotation=true;
          if(seen_corner_rotation && cmd.linear.x>1e-6 && resume_yaw<0)resume_yaw=yaw*180/M_PI;
        }
        if(body_check.footprintCostAtPose(current.pose.position.x,current.pose.position.y,
            yaw,map->getRobotFootprint())>=254)throw std::runtime_error("Actual simulated body collided");
        if(!std::isfinite(cmd.linear.x)||!std::isfinite(cmd.angular.z))throw std::runtime_error("Nonfinite command");
        if(scenario!="same_xy_180" && current.pose.position.x>.94 && current.pose.position.y>.02 && first_depart_yaw<0)
          first_depart_yaw=yaw*180/M_PI;
        if(tick%10==0 || reached)std::cout<<"{\"scenario\":\""<<scenario<<"\",\"tick\":"<<tick
          <<",\"x\":"<<current.pose.position.x<<",\"y\":"<<current.pose.position.y
          <<",\"yaw_deg\":"<<yaw*180/M_PI<<",\"vx\":"<<cmd.linear.x<<",\"wz\":"<<cmd.angular.z<<"}\n";
        if(reached){success=true;break;}
        const double dt=.1,wyaw=yaw+cmd.angular.z*dt*.5;
        current=pose(current.pose.position.x+cmd.linear.x*std::cos(wyaw)*dt,
          current.pose.position.y+cmd.linear.x*std::sin(wyaw)*dt,yaw+cmd.angular.z*dt);speed=cmd;
      }
      std::cout<<"{\"summary\":\""<<scenario<<"\",\"success\":"<<success
        <<",\"first_depart_yaw_deg\":"<<first_depart_yaw
        <<",\"resume_yaw_deg\":"<<resume_yaw<<"}\n";
      if(scenario=="blocked_rotation" ? !rejected : !success)result=2;
      bridge.cancel();
    }
    {
      // Actual legacy action transport in the isolated domain. No /cmd_vel publication.
      using Action=nav2_msgs::action::FollowPath;
      using Handle=rclcpp_action::ServerGoalHandle<Action>;
      std::fill(map->getCostmap()->getCharMap(),map->getCostmap()->getCharMap()+640000,0);
      auto body=pose(0,0,0);geometry_msgs::msg::Twist measured;
      std::shared_ptr<Handle> active;
      auto server=rclcpp_action::create_server<Action>(parent,"~/follow_path_probe",
        [](const rclcpp_action::GoalUUID &,std::shared_ptr<const Action::Goal> g){
          const bool valid=!g->path.poses.empty() && g->path.header.frame_id=="probe_map" &&
            (g->controller_id.empty() || g->controller_id=="RPP") &&
            (g->goal_checker_id.empty() || g->goal_checker_id=="Checker") && g->progress_checker_id.empty();
          return valid?rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE:rclcpp_action::GoalResponse::REJECT;
        },
        [&](const std::shared_ptr<Handle> handle){
          if(handle==active)bridge.cancel();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<Handle> handle){
          if(active && active->is_active()) {
            auto res=std::make_shared<Action::Result>();res->error_code=Action::Result::UNKNOWN;
            res->error_msg="preempted_by_new_goal";active->abort(res);
          }
          active=handle;bridge.submit(*handle->get_goal());measured=geometry_msgs::msg::Twist();
        });
      auto timer=parent->create_wall_timer(std::chrono::milliseconds(10),[&](){
        if(!active || !active->is_active())return;
        auto res=std::make_shared<Action::Result>();
        if(active->is_canceling()){bridge.cancel();active->canceled(res);return;}
        try {
          set_pose(body);bool reached=false;auto cmd=bridge.step(body,measured,reached);
          auto feedback=std::make_shared<Action::Feedback>();
          const auto & goal=active->get_goal()->path.poses.back();
          feedback->distance_to_goal=std::hypot(body.pose.position.x-goal.pose.position.x,body.pose.position.y-goal.pose.position.y);
          feedback->speed=std::hypot(cmd.linear.x,cmd.linear.y);active->publish_feedback(feedback);
          if(reached){active->succeed(res);return;}
          const double yaw=tf2::getYaw(body.pose.orientation),dt=.1;
          body=pose(body.pose.position.x+cmd.linear.x*std::cos(yaw+cmd.angular.z*dt/2)*dt,
            body.pose.position.y+cmd.linear.x*std::sin(yaw+cmd.angular.z*dt/2)*dt,yaw+cmd.angular.z*dt);
          measured=cmd;
        }catch(const std::exception & ex){bridge.cancel();res->error_code=Action::Result::NO_VALID_CONTROL;res->error_msg=ex.what();active->abort(res);}
      });
      auto client_node=std::make_shared<rclcpp::Node>("legacy_follow_path_client","/official_path_probe");
      auto client=rclcpp_action::create_client<Action>(client_node,
        "/official_path_probe/official_controller_probe/follow_path_probe");
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(parent->get_node_base_interface());exec.add_node(client_node);
      if(!client->wait_for_action_server(std::chrono::seconds(3)))throw std::runtime_error("Legacy action server discovery failed");
      auto invalid=client->async_send_goal(Action::Goal());
      if(exec.spin_until_future_complete(invalid,std::chrono::seconds(3))!=rclcpp::FutureReturnCode::SUCCESS || invalid.get())
        throw std::runtime_error("Empty legacy goal should be rejected");
      std::cout<<"{\"wire_test\":\"legacy_invalid_goal_rejected\",\"success\":true}\n";
      Action::Goal goal;goal.path.header.frame_id="probe_map";goal.controller_id="RPP";goal.goal_checker_id="Checker";
      for(int a=0;a<=180;a+=5)goal.path.poses.push_back(pose(0,0,a*M_PI/180));
      unsigned feedback_count=0;
      rclcpp_action::Client<Action>::SendGoalOptions goal_options;
      goal_options.feedback_callback=[&](auto,auto){++feedback_count;};
      auto submitted=client->async_send_goal(goal,goal_options);
      if(exec.spin_until_future_complete(submitted,std::chrono::seconds(3))!=rclcpp::FutureReturnCode::SUCCESS || !submitted.get())
        throw std::runtime_error("Legacy goal was not accepted");
      auto done=client->async_get_result(submitted.get());
      if(exec.spin_until_future_complete(done,std::chrono::seconds(8))!=rclcpp::FutureReturnCode::SUCCESS ||
         done.get().code!=rclcpp_action::ResultCode::SUCCEEDED || feedback_count==0)
        throw std::runtime_error("Legacy goal/feedback/result roundtrip failed");
      std::cout<<"{\"wire_test\":\"legacy_goal_feedback_result\",\"success\":true,\"feedback_count\":"<<feedback_count<<"}\n";
      body=pose(0,0,0);auto to_cancel=client->async_send_goal(goal);
      if(exec.spin_until_future_complete(to_cancel,std::chrono::seconds(3))!=rclcpp::FutureReturnCode::SUCCESS || !to_cancel.get())
        throw std::runtime_error("Cancel test goal was not accepted");
      auto canceled_result=client->async_get_result(to_cancel.get());
      auto cancel_ack=client->async_cancel_goal(to_cancel.get());
      if(exec.spin_until_future_complete(cancel_ack,std::chrono::seconds(3))!=rclcpp::FutureReturnCode::SUCCESS ||
         exec.spin_until_future_complete(canceled_result,std::chrono::seconds(3))!=rclcpp::FutureReturnCode::SUCCESS ||
         canceled_result.get().code!=rclcpp_action::ResultCode::CANCELED)
        throw std::runtime_error("Legacy cancellation roundtrip failed");
      std::cout<<"{\"wire_test\":\"legacy_cancel_result\",\"success\":true}\n";
      timer->cancel();exec.remove_node(client_node);exec.remove_node(parent->get_node_base_interface());
    }
    bridge.controller.deactivate();bridge.controller.cleanup();
    map->cleanup();
  }catch(const std::exception & ex){std::cerr<<"PROBE_ERROR "<<ex.what()<<'\n';result=3;}
  rclcpp::shutdown();return result;
}
