#include <gtest/gtest.h>
#include <class_loader/class_loader.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>
#include <fstream>
#include "official_lattice_rpp_adapter/terminal_goal_binding.hpp"
#include "robot_feasible_navigation/footprint_sweep.hpp"
#include "official_lattice_rpp_adapter/private_footprint_sweep.hpp"

using namespace std::chrono_literals;
namespace {
geometry_msgs::msg::PoseStamped pose(double x,double y,double yaw=0.) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(yaw/2.);p.pose.orientation.w=std::cos(yaw/2.);return p;
}
class SameHeading : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("same_heading_test");
    node->declare_parameter("GridBased.lattice_filepath",std::string(TEST_LATTICE_FILE));
    node->declare_parameter("GridBased.terminal_goal_binding",true);
    node->declare_parameter("GridBased.private_same_heading_approach",true);
    node->declare_parameter("GridBased.max_planning_time",2.);
    node->declare_parameter("GridBased.lookup_table_size",1.);
    node->declare_parameter("GridBased.cost_penalty",1.);
    node->declare_parameter("GridBased.max_iterations",1000000);
    node->declare_parameter("GridBased.terminal_checking_interval",500);
    map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("same_heading_test_map");
    map->declare_parameter("inflation_layer.plugin",std::string("nav2_costmap_2d::InflationLayer"));
    map->declare_parameter("inflation_layer.inflation_radius",1.4);
    map->declare_parameter("inflation_layer.cost_scaling_factor",3.);
    map->declare_parameter("inflation_layer.inflate_around_unknown",true);
    const auto results=map->set_parameters({rclcpp::Parameter("plugins",std::vector<std::string>{"inflation_layer"}),
      rclcpp::Parameter("global_frame","map"),rclcpp::Parameter("robot_base_frame","base_link"),
      rclcpp::Parameter("width",6),rclcpp::Parameter("height",6),rclcpp::Parameter("origin_x",-3.),
      rclcpp::Parameter("origin_y",-3.),rclcpp::Parameter("resolution",.01),
      rclcpp::Parameter("footprint_padding",.01),rclcpp::Parameter("footprint",std::string("[[-0.2,-0.15],[0.2,-0.15],[0.2,0.15],[-0.2,0.15]]"))});
    for (const auto & r:results) ASSERT_TRUE(r.successful)<<r.reason;
    ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State()),nav2_util::CallbackReturn::SUCCESS);
    auto * grid=map->getCostmap();std::fill_n(grid->getCharMap(),grid->getSizeInCellsX()*grid->getSizeInCellsY(),0);
    const char * library=std::getenv("COL63_PLANNER_TEST_LIBRARY");
    loader=std::make_unique<class_loader::ClassLoader>(library?library:TEST_ADAPTER_LIBRARY);
    planner=loader->createInstance<nav2_core::GlobalPlanner>("official_lattice_rpp_adapter::CheckedLattice");
    planner->configure(node,"GridBased",map->getTfBuffer(),map);planner->activate();
  }
  void TearDown() override {if(planner){planner->deactivate();planner->cleanup();}
    if(map){map->on_cleanup(rclcpp_lifecycle::State());map.reset();}node.reset();planner.reset();loader.reset();}
  nav_msgs::msg::Path makePlan() {return planner->createPlan(pose(-1.6,.02),pose(0.,0.),[]{return false;});}
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::unique_ptr<class_loader::ClassLoader> loader;nav2_core::GlobalPlanner::Ptr planner;
};
TEST_F(SameHeading, ActualPlannerOutputRetainsOriginalGoalAndBoundStraightSuffix) {
  const auto path=makePlan();ASSERT_GT(path.poses.size(),3u);
  const auto b=official_lattice_rpp_adapter::goal_region::splitBinding(path,.01,6.);
  EXPECT_DOUBLE_EQ(b.original_goal.pose.position.x,0.);EXPECT_DOUBLE_EQ(b.original_goal.pose.position.y,0.);
  EXPECT_LT(b.error,.015);EXPECT_LT(b.suffix_begin,b.rotation_begin);
  const auto & end=b.executable.poses.back().pose;EXPECT_NEAR(tf2::getYaw(end.orientation),0.,1e-9);
  EXPECT_NEAR(end.position.x,0.,1e-9);
  EXPECT_GT(b.executable.poses[b.rotation_begin].pose.position.x-
    b.executable.poses[b.suffix_begin].pose.position.x,.8);
}
TEST_F(SameHeading, RecordedNearAxisDeviationRetainsExecutableContinuity) {
  const auto path=planner->createPlan(pose(-1.01865065,.01501456,.00184923),pose(0.,0.),[]{return false;});
  const auto b=official_lattice_rpp_adapter::goal_region::splitBinding(path,.01,6.);
  EXPECT_LT(b.error,.015);EXPECT_LT(b.suffix_begin,b.rotation_begin);
  EXPECT_GT(b.executable.poses[b.rotation_begin].pose.position.x-
    b.executable.poses[b.suffix_begin].pose.position.x,.9);
}
TEST_F(SameHeading, RealControllerAcceptsPlannerBindingAndOrderedPath) {
  const auto path=makePlan();
  node->declare_parameter("FollowPath.terminal_goal_binding",true);
  node->declare_parameter("FollowPath.terminal_global_resolution",.01);
  node->declare_parameter("FollowPath.terminal_global_extent",6.);
  auto controller=loader->createInstance<nav2_core::Controller>("official_lattice_rpp_adapter::FeasibleRPP");
  controller->configure(node,"FollowPath",map->getTfBuffer(),map);controller->activate();
  EXPECT_NO_THROW(controller->setPlan(path));controller->deactivate();controller->cleanup();controller.reset();
}
TEST_F(SameHeading, RealControllerAcceptsLivePreflightPath) {
  const char * file=std::getenv("COL63_LIVE_PATH");if(!file)GTEST_SKIP()<<"requires live planned path";
  std::ifstream input(file);ASSERT_TRUE(input.good());nav_msgs::msg::Path path;path.header.frame_id="map";
  double x,y,a;while(input>>x>>y>>a){auto p=pose(x,y,a);p.header=path.header;path.poses.push_back(p);}
  ASSERT_GT(path.poses.size(),2u);
  const auto b=official_lattice_rpp_adapter::goal_region::splitBinding(path,.01,96.96);
  EXPECT_LT(b.error,.015);EXPECT_LT(b.suffix_begin,b.rotation_begin);
  node->declare_parameter("FollowPath.terminal_goal_binding",true);
  node->declare_parameter("FollowPath.terminal_global_resolution",.01);
  node->declare_parameter("FollowPath.terminal_global_extent",96.96);
  auto controller=loader->createInstance<nav2_core::Controller>("official_lattice_rpp_adapter::FeasibleRPP");
  controller->configure(node,"FollowPath",map->getTfBuffer(),map);controller->activate();
  EXPECT_NO_THROW(controller->setPlan(path));controller->deactivate();controller->cleanup();controller.reset();
}
TEST_F(SameHeading, SameLockedMapThresholdAppliesToTailAndOldInflationValues) {
  auto * grid=map->getCostmap();unsigned int x,y;ASSERT_TRUE(grid->worldToMap(-.5,.15,x,y));
  for (unsigned char value:{251,252,253}) {grid->setCost(x,y,value);EXPECT_NO_THROW(makePlan())<<int(value);}
  for (unsigned char value:{254,255}) {grid->setCost(x,y,value);EXPECT_ANY_THROW(makePlan())<<int(value);}
}
TEST_F(SameHeading, BroadPhaseThresholdAndFirstBlockerMatchNarrowPhase) {
  namespace sweep=robot_feasible_navigation::sweep;
  auto * grid=map->getCostmap();unsigned int x,y;ASSERT_TRUE(grid->worldToMap(-.5,0.,x,y));grid->setCost(x,y,253);
  const auto fp=map->getRobotFootprint();
  {sweep::ScopedObstacleIndex legacy(*grid);EXPECT_TRUE(sweep::footprintFree(*grid,fp,-.5,0.,0.));}
  {official_lattice_rpp_adapter::private_sweep::ScopedObstacleIndex strict(*grid,254);
    official_lattice_rpp_adapter::private_sweep::BlockingCell failure;
    EXPECT_TRUE(official_lattice_rpp_adapter::private_sweep::footprintFree(*grid,fp,-.5,0.,0.,254,&failure));}
  grid->setCost(x,y,254);
  {official_lattice_rpp_adapter::private_sweep::ScopedObstacleIndex strict(*grid,254);
    official_lattice_rpp_adapter::private_sweep::BlockingCell failure;
    EXPECT_FALSE(official_lattice_rpp_adapter::private_sweep::footprintFree(*grid,fp,-.5,0.,0.,254,&failure));
    EXPECT_TRUE(failure.present);EXPECT_EQ(failure.column,x);EXPECT_EQ(failure.row,y);EXPECT_EQ(failure.cost,254);}
}
TEST_F(SameHeading, NativeLatticeOwnsApproachCenter253Policy) {
  auto * grid=map->getCostmap();unsigned int x,y;ASSERT_TRUE(grid->worldToMap(-1.,0.,x,y));
  grid->setCost(x,y,253);
  EXPECT_ANY_THROW(makePlan());
}
TEST_F(SameHeading, SameHeadingTailRejectsCenter253) {
  auto * grid=map->getCostmap();unsigned int x,y;ASSERT_TRUE(grid->worldToMap(-.5,0.,x,y));
  grid->setCost(x,y,253);EXPECT_ANY_THROW(makePlan());
}
TEST_F(SameHeading, OutOfMapFootprintIsRejected) {
  EXPECT_ANY_THROW(planner->createPlan(pose(-2.95,0.),pose(0.,0.),[]{return false;}));
}
TEST_F(SameHeading, FrozenRealMasterAndActualStart) {
  const char * file=std::getenv("COL63_LOCKED_GRID");if(!file)GTEST_SKIP()<<"requires sealed master";
  auto * grid=map->getCostmap();grid->resizeMap(7904,9696,.009999999776482582,-52.79999923706055,-70.55999755859375);
  std::ifstream input(file,std::ios::binary);ASSERT_TRUE(input.good());
  input.read(reinterpret_cast<char *>(grid->getCharMap()),7904ull*9696ull);ASSERT_EQ(input.gcount(),7904ll*9696ll);
  // Costmap fixture already adds its configured 0.01 m padding.
  std::vector<geometry_msgs::msg::Point> fp(4);fp[0].x=-.7870000207424164;fp[0].y=-.669999977350235;
  fp[1].x=.7870000207424164;fp[1].y=-.669999977350235;
  fp[2].x=.7870000207424164;fp[2].y=.669999977350235;
  fp[3].x=-.7870000207424164;fp[3].y=.669999977350235;map->setRobotFootprint(fp);
  ASSERT_NEAR(map->getRobotFootprint()[0].x,-.7970000207424164,1e-7);
  const auto start=pose(6.540855906830973,-46.230724476531634,-.4246158894350498);
  const auto goal=pose(7.449999437439452,-46.649998084613756,-.42662749312687603);
  if (std::getenv("COL63_EXPECT_OLD")) {
    EXPECT_THROW(planner->createPlan(start,goal,[]{return false;}),nav2_core::NoValidPathCouldBeFound);
  } else {
    auto result=planner->createPlan(start,goal,[]{return false;});
    auto binding=official_lattice_rpp_adapter::goal_region::splitBinding(result,grid->getResolution(),96.96);
    EXPECT_LT(binding.error,.015);EXPECT_LT(binding.suffix_begin,binding.rotation_begin);
    if(const char * output=std::getenv("COL63_PLAN_OUT")) {
      std::ofstream saved(output);saved.precision(17);
      saved<<"# x y yaw\n";
      for(const auto & p:result.poses) saved<<p.pose.position.x<<" "<<p.pose.position.y<<" "<<tf2::getYaw(p.pose.orientation)<<"\n";
    }
    node->declare_parameter("FollowPath.terminal_goal_binding",true);
    node->declare_parameter("FollowPath.terminal_global_resolution",grid->getResolution());
    node->declare_parameter("FollowPath.terminal_global_extent",96.96);
    auto controller=loader->createInstance<nav2_core::Controller>("official_lattice_rpp_adapter::FeasibleRPP");
    controller->configure(node,"FollowPath",map->getTfBuffer(),map);controller->activate();
    EXPECT_NO_THROW(controller->setPlan(result));controller->deactivate();controller->cleanup();controller.reset();
  }
}
TEST_F(SameHeading, ExternalCancelInterruptsLockWaitAndNextRequestStillWorks) {
  EXPECT_THROW(planner->createPlan(pose(-1.6,.02),pose(0.,0.),[]{return true;}),nav2_core::PlannerCancelled);
  std::atomic<bool> held{false};std::thread other([&] {std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getCostmap()->getMutex());held=true;std::this_thread::sleep_for(250ms);});
  while(!held)std::this_thread::yield();
  const auto begin=std::chrono::steady_clock::now();
  EXPECT_THROW(planner->createPlan(pose(-1.6,.02),pose(0.,0.),[&]{return std::chrono::steady_clock::now()-begin>=50ms;}),nav2_core::PlannerCancelled);
  EXPECT_LT(std::chrono::steady_clock::now()-begin,200ms);other.join();
  EXPECT_NO_THROW(makePlan());
}
TEST_F(SameHeading, LockWaitDoesNotConsumeNativeSearchBudget) {
  std::atomic<bool> held{false};std::thread other([&] {std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getCostmap()->getMutex());held=true;std::this_thread::sleep_for(2300ms);});
  while(!held)std::this_thread::yield();
  const auto begin=std::chrono::steady_clock::now();EXPECT_NO_THROW(makePlan());
  EXPECT_GE(std::chrono::steady_clock::now()-begin,2s);other.join();
}
}
