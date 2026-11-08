// Copyright 2026. SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include "nav2_core/planner_exceptions.hpp"
#include "robot_feasible_navigation/safe_lattice_planner.hpp"
#include "robot_feasible_navigation/goal_cell_binding.hpp"

using robot_feasible_navigation::SafeLatticePlanner;
namespace sweep = robot_feasible_navigation::sweep;
namespace
{
sweep::Footprint rectangle()
{
  sweep::Footprint out;
  for (const auto & xy : {std::pair<double, double>{-.31, -.16}, {.31, -.16}, {.31, .16}, {-.31, .16}}) {
    geometry_msgs::msg::Point point;
    point.x = xy.first; point.y = xy.second; out.push_back(point);
  }
  return out;
}
geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = "map";
  out.pose.position.x = x; out.pose.position.y = y;
  out.pose.orientation.z = std::sin(yaw / 2); out.pose.orientation.w = std::cos(yaw / 2);
  return out;
}
void obstacle(nav2_costmap_2d::Costmap2D & map, double x, double y, unsigned char value = 254)
{
  unsigned int mx, my;
  ASSERT_TRUE(map.worldToMap(x, y, mx, my)); map.setCost(mx, my, value);
}
nlohmann::json controls()
{
  const auto file = std::filesystem::path(__FILE__).parent_path().parent_path() /
    "params/stop_turn_1cm_h16.json";
  std::ifstream stream(file);
  if (!stream) {throw std::runtime_error("missing exact control set fixture");}
  nlohmann::json value; stream >> value; return value;
}
void initializeMotionHeuristic()
{
  namespace smac = robot_feasible_navigation_smac;
  smac::SearchInfo search_info;
  search_info.lattice_filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() /
    "params/stop_turn_1cm_h16.json").string();
  search_info.minimum_turning_radius = 50.f;
  smac::NodeLattice::precomputeDistanceHeuristic(5.f, smac::MotionModel::STATE_LATTICE, 16u,
    search_info);
  unsigned int width = 400;
  smac::NodeLattice::motion_table.initMotionModel(width, search_info);
}
}  // namespace

TEST(LatticeSafety, MotionHeuristicDoesNotOverchargeAdjacentInPlaceTurns)
{
  namespace smac = robot_feasible_navigation_smac;
  initializeMotionHeuristic();
  ASSERT_EQ(controls()["primitives"].size(), 48u);
  // These actual headings are nonuniform; the final pair also crosses 2pi.
  for (const auto & bins : {std::pair<unsigned int, unsigned int>{1, 2}, {8, 9}, {0, 15}}) {
    for (bool reverse : {false, true}) {
      const auto from = reverse ? bins.second : bins.first;
      const auto to = reverse ? bins.first : bins.second;
      const float heuristic = smac::NodeLattice::getDistanceHeuristic(
        {200.f, 200.f, static_cast<float>(from)},
        {200.f, 200.f, static_cast<float>(to)}, 0.f);
      EXPECT_TRUE(std::isfinite(heuristic));
      EXPECT_GE(heuristic, 0.f);
      EXPECT_LE(heuristic, 5.f);
      EXPECT_FLOAT_EQ(heuristic, 5.f);
    }
  }
}

TEST(LatticeSafety, MotionHeuristicBoundsActualStraightAndMixedStopTurnPaths)
{
  namespace smac = robot_feasible_navigation_smac;
  initializeMotionHeuristic();
  const auto value = controls();
  const float resolution = value["lattice_metadata"]["grid_resolution"].get<float>();
  for (const auto & primitive : value["primitives"]) {
    const float length = primitive["trajectory_length"].get<float>();
    if (length == 0.f) {continue;}
    const auto heading = primitive["start_angle_index"].get<unsigned int>();
    const auto & endpoint = primitive["poses"].back();
    const smac::NodeLattice::Coordinates from(200.f, 200.f, heading);
    const smac::NodeLattice::Coordinates straight(
      200.f + endpoint[0].get<float>() / resolution,
      200.f + endpoint[1].get<float>() / resolution, heading);
    const float straight_cost = length / resolution * .985f;
    const float straight_h = smac::NodeLattice::getDistanceHeuristic(from, straight, 0.f);
    EXPECT_TRUE(std::isfinite(straight_h));
    EXPECT_NEAR(straight_h, straight_cost, 1e-5f);
    auto mixed = straight;
    mixed.theta = (heading + 2u) % 16u;
    const float mixed_h = smac::NodeLattice::getDistanceHeuristic(from, mixed, 0.f);
    EXPECT_TRUE(std::isfinite(mixed_h));
    EXPECT_LE(mixed_h, straight_cost + 10.f + 1e-5f);
    EXPECT_NEAR(mixed_h, straight_h + 10.f, 1e-5f);
  }
  // A realizable path moves three cells east, turns four bins, then moves
  // four cells north. Euclidean translation must not charge more than that.
  const smac::NodeLattice::Coordinates from(0.f, 0.f, 0.f), to(3.f, 4.f, 4.f);
  const float mixed_h = smac::NodeLattice::getDistanceHeuristic(from, to, 0.f);
  EXPECT_TRUE(std::isfinite(mixed_h));
  EXPECT_LE(mixed_h, 7.f * .985f + 4.f * 5.f);
  EXPECT_NEAR(mixed_h, 5.f * .985f + 4.f * 5.f, 1e-5f);
  EXPECT_FLOAT_EQ(smac::NodeLattice::getDistanceHeuristic(from, from, 0.f), 0.f);
  // The motion lower bound stays available outside the former local LUT,
  // including when the separate obstacle heuristic already has a value.
  const smac::NodeLattice::Coordinates distant(300.f, 400.f, 15.f);
  const float distant_h = smac::NodeLattice::getDistanceHeuristic(from, distant, 100.f);
  EXPECT_TRUE(std::isfinite(distant_h));
  EXPECT_NEAR(distant_h, 500.f * .985f + 5.f, 1e-4f);
}

TEST(LatticeSafety, AcceptsActualControlSetAndRejectsFakeResolution)
{
  auto value = controls();
  EXPECT_NO_THROW(SafeLatticePlanner::validateControlSet(value, .01));
  value["lattice_metadata"]["grid_resolution"] = .05;
  EXPECT_THROW(SafeLatticePlanner::validateControlSet(value, .01), std::runtime_error);
}

TEST(LatticeSafety, RejectsIncompleteHeadingAndMovingCurve)
{
  auto value = controls();
  value["primitives"][1]["start_angle_index"] = 8;
  EXPECT_THROW(SafeLatticePlanner::validateControlSet(value, .01), std::runtime_error);
  value = controls();
  value["primitives"][0]["poses"][1][2] = .1;
  EXPECT_THROW(SafeLatticePlanner::validateControlSet(value, .01), std::runtime_error);
}

TEST(LatticeSafety, SameXYRotationCannotSkipBlockedIntermediateAngle)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .01, -2., -2., 0);
  obstacle(map, .22, .22);
  const auto footprint = rectangle();
  EXPECT_TRUE(sweep::footprintFree(map, footprint, 0., 0., 0.));
  EXPECT_TRUE(sweep::footprintFree(map, footprint, 0., 0., M_PI_2));
  nav_msgs::msg::Path path; path.poses.push_back(pose(0., 0., 0.));
  EXPECT_THROW(SafeLatticePlanner::appendCheckedSegment(
    path, pose(0., 0., M_PI_2), map, footprint), nav2_core::NoValidPathCouldBeFound);
  EXPECT_EQ(path.poses.size(), 1u);
}

TEST(LatticeSafety, ActualFilledInteriorAndUnknownReject)
{
  for (auto value : {254u, 255u}) {
    nav2_costmap_2d::Costmap2D map(400, 400, .01, -2., -2., 0);
    obstacle(map, .01, .01, value);
    nav_msgs::msg::Path path; path.poses.push_back(pose(0., 0., 0.));
    EXPECT_THROW(SafeLatticePlanner::appendCheckedSegment(
      path, pose(0., 0., .1), map, rectangle()), nav2_core::NoValidPathCouldBeFound);
  }
}

TEST(LatticeSafety, ExactUserYawRetainedWithDensePureTurn)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .01, -2., -2., 0);
  nav_msgs::msg::Path path; path.poses.push_back(pose(0., 0., 0.));
  const auto target = pose(0., 0., 1.1111111);
  SafeLatticePlanner::appendCheckedSegment(path, target, map, rectangle());
  ASSERT_GT(path.poses.size(), 50u);
  EXPECT_EQ(path.poses.back().pose, target.pose);
  for (const auto & point : path.poses) {
    EXPECT_DOUBLE_EQ(point.pose.position.x, 0.); EXPECT_DOUBLE_EQ(point.pose.position.y, 0.);
  }
}

TEST(LatticeSafety, WrapTakesShortContinuousArcAndCancellationIsHonored)
{
  nav2_costmap_2d::Costmap2D map(400, 400, .01, -2., -2., 0);
  nav_msgs::msg::Path path; path.poses.push_back(pose(0., 0., 3.12));
  SafeLatticePlanner::appendCheckedSegment(path, pose(0., 0., -3.12), map, rectangle());
  EXPECT_LT(path.poses.size(), 10u);
  const auto size = path.poses.size();
  EXPECT_THROW(SafeLatticePlanner::appendCheckedSegment(
    path, pose(0., 0., 0.), map, rectangle(), [] {return true;}), nav2_core::PlannerCancelled);
  EXPECT_EQ(path.poses.size(), size);
}

TEST(LatticeSafety, RealConfigureAcceptsDeclaredOverridesWithoutAutomaticDeclaration)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  const auto file = std::filesystem::path(__FILE__).parent_path().parent_path() /
    "params/stop_turn_1cm_h16.json";
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(false);
  options.parameter_overrides({
      rclcpp::Parameter("GridBased.lattice_filepath", file.string()),
      rclcpp::Parameter("GridBased.smooth_path", false),
      rclcpp::Parameter("GridBased.allow_unknown", false),
      rclcpp::Parameter("GridBased.allow_reverse_expansion", false),
      rclcpp::Parameter("GridBased.analytic_expansion_max_length", 0.0),
      rclcpp::Parameter("GridBased.rotation_penalty", 5.0),
      rclcpp::Parameter("GridBased.lookup_table_size", .05),
      rclcpp::Parameter("GridBased.max_planning_time", .2)});
  auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lattice_unit_parent", options);
  ASSERT_FALSE(parent->has_parameter("GridBased.lattice_filepath"));
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("lattice_unit_map");
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{})).successful);
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("resolution", .01)).successful);
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("width", 4)).successful);
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("height", 4)).successful);
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter(
    "footprint", "[[-0.31,-0.16],[0.31,-0.16],[0.31,0.16],[-0.31,0.16]]")).successful);
  ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
  SafeLatticePlanner planner;
  EXPECT_NO_THROW(planner.configure(parent, "GridBased", nullptr, costmap));
  ASSERT_TRUE(parent->has_parameter("GridBased.lattice_filepath"));
  const auto changed = parent->set_parameter(rclcpp::Parameter("GridBased.allow_unknown", true));
  EXPECT_FALSE(changed.successful);
  auto * grid = costmap->getCostmap();
  std::fill(grid->getCharMap(), grid->getCharMap() + grid->getSizeInCellsX() * grid->getSizeInCellsY(), 0);
  const auto exact_start = pose(2., 2., 0.);
  const auto exact_goal = pose(2., 2., M_PI_2);
  const auto rotation = planner.createPlan(exact_start, exact_goal, [] {return false;});
  EXPECT_GT(rotation.poses.size(), 50u);
  ASSERT_FALSE(rotation.poses.empty());
  EXPECT_EQ(rotation.poses.back().pose, exact_goal.pose);
  obstacle(*grid, 2.22, 2.22);
  EXPECT_THROW(planner.createPlan(exact_start, exact_goal, [] {return false;}),
    nav2_core::NoValidPathCouldBeFound);
  // Exercise the private search edge, not only the wrapper. A safe incoming
  // edge to this same endpoint must never cache-validate a different blocked
  // rotation. This counterexample passes incorrectly with the upstream early
  // _cell_cost return / XY-only intermediate sampling.
  namespace smac = robot_feasible_navigation_smac;
  // configure() precomputes the heuristic; actual search then initializes its
  // motion table from SearchInfo when setCollisionChecker() is called. The
  // same-cell wrapper deliberately did not enter that search path above.
  smac::SearchInfo search_info;
  search_info.lattice_filepath = parent->get_parameter("GridBased.lattice_filepath").as_string();
  search_info.rotation_penalty = parent->get_parameter("GridBased.rotation_penalty").as_double();
  search_info.minimum_turning_radius = .5 / grid->getResolution();
  auto width = grid->getSizeInCellsX();
  smac::NodeLattice::motion_table.initMotionModel(width, search_info);
  smac::GridCollisionChecker checker(costmap, 72u, parent);
  smac::NodeLattice child(0), first_parent(1);
  child.setPose(smac::NodeLattice::Coordinates(200.f, 200.f, 4.f));
  smac::MotionPrimitive safe{};
  safe.start_angle = 4.f; safe.end_angle = 4.f; safe.trajectory_length = 0.f;
  safe.poses.emplace_back(0.f, 0.f, static_cast<float>(M_PI_2), smac::TurnDirection::UNKNOWN);
  safe.poses.emplace_back(0.f, 0.f, static_cast<float>(M_PI_2), smac::TurnDirection::UNKNOWN);
  ASSERT_TRUE(child.isNodeValid(false, &checker, &safe, false));
  child.setMotionPrimitive(&safe);
  auto child_pointer = &child;
  EXPECT_GT(first_parent.getTraversalCost(child_pointer), 0.f);
  smac::MotionPrimitive blocked{};
  blocked.start_angle = 0.f; blocked.end_angle = 4.f; blocked.trajectory_length = 0.f;
  blocked.poses.emplace_back(0.f, 0.f, 0.f, smac::TurnDirection::UNKNOWN);
  blocked.poses.emplace_back(0.f, 0.f, static_cast<float>(M_PI_4), smac::TurnDirection::UNKNOWN);
  blocked.poses.emplace_back(0.f, 0.f, static_cast<float>(M_PI_2), smac::TurnDirection::UNKNOWN);
  EXPECT_FALSE(child.isNodeValid(false, &checker, &blocked, false));
  planner.cleanup();
  costmap->on_cleanup(rclcpp_lifecycle::State());
}

TEST(LatticeSafety, RestoreOnlyDocumentedFloatStartWithoutTinyTurnDetour)
{
  nav2_costmap_2d::Costmap2D map(1000, 1000, .01, -.123456789, -.987654321, 0);
  const auto exact = pose(1.234567891, 2.345678912, .12);
  auto searched = pose(0., 0., .4636476090008061);
  searched.pose.position.x = static_cast<float>(map.getOriginX()) +
    static_cast<float>((exact.pose.position.x - map.getOriginX()) / .01) * .01;
  searched.pose.position.y = static_cast<float>(map.getOriginY()) +
    static_cast<float>((exact.pose.position.y - map.getOriginY()) / .01) * .01;
  ASSERT_NE(searched.pose.position.x, exact.pose.position.x);
  EXPECT_NO_THROW(SafeLatticePlanner::restoreSearchStartRoundoff(searched, exact, map));
  EXPECT_DOUBLE_EQ(searched.pose.position.x, exact.pose.position.x);
  EXPECT_DOUBLE_EQ(searched.pose.position.y, exact.pose.position.y);
  EXPECT_NEAR(tf2::getYaw(searched.pose.orientation), .4636476090008061, 1e-12);
  searched.pose.position.x += .003;
  EXPECT_THROW(SafeLatticePlanner::restoreSearchStartRoundoff(searched, exact, map), std::runtime_error);
}

TEST(LatticeSafety, RestoreWholeEndpointRotationClustersWithoutInventingMovingTurn)
{
  nav2_costmap_2d::Costmap2D map(1792, 1888, .009999999776482582,
    -6.87999963760376, -9.59999942779541, 0);
  const auto start = pose(-3.8007497322782706, 4.658441093749194, 3.0504716917604155);
  const auto goal = pose(-3.789631256533969, 4.698352909931362, 1.1210367137250565);
  auto rounded = [&](const geometry_msgs::msg::PoseStamped & exact, double yaw) {
      return pose(static_cast<float>(map.getOriginX()) +
        static_cast<float>((exact.pose.position.x - map.getOriginX()) / map.getResolution()) *
        map.getResolution(), static_cast<float>(map.getOriginY()) +
        static_cast<float>((exact.pose.position.y - map.getOriginY()) / map.getResolution()) *
        map.getResolution(), yaw);
    };
  nav_msgs::msg::Path searched;
  searched.poses = {rounded(start, M_PI), rounded(start, M_PI - .0025),
    rounded(start, M_PI - .005), pose(-3.79, 4.69, 1.2),
    rounded(goal, 1.2), rounded(goal, 1.1975), rounded(goal, 1.195)};
  const auto unrelated = searched.poses[3];
  // The actual failed runtime had a 0.336um start round-trip. Restoring only
  // front() creates XY motion alongside the second sample's nonzero yaw delta.
  ASSERT_GT(std::hypot(searched.poses.front().pose.position.x - start.pose.position.x,
    searched.poses.front().pose.position.y - start.pose.position.y), 1e-7);
  SafeLatticePlanner::restoreSearchRoundoff(searched, start, goal, map);
  for (std::size_t i : {0u, 1u, 2u}) {
    EXPECT_DOUBLE_EQ(searched.poses[i].pose.position.x, start.pose.position.x);
    EXPECT_DOUBLE_EQ(searched.poses[i].pose.position.y, start.pose.position.y);
  }
  for (std::size_t i : {4u, 5u, 6u}) {
    EXPECT_DOUBLE_EQ(searched.poses[i].pose.position.x, goal.pose.position.x);
    EXPECT_DOUBLE_EQ(searched.poses[i].pose.position.y, goal.pose.position.y);
  }
  EXPECT_EQ(searched.poses[3], unrelated);
  EXPECT_NEAR(tf2::getYaw(searched.poses[1].pose.orientation), M_PI - .0025, 1e-12);
  auto wrong = searched;
  wrong.poses.front().pose.position.x += .003;
  EXPECT_THROW(SafeLatticePlanner::restoreSearchRoundoff(wrong, start, goal, map), std::runtime_error);
}

TEST(LatticeSafety, NearGoalFloatPriorityTieKeepsAcceptedParentAndPrimitiveTogether)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  const auto file = std::filesystem::path(__FILE__).parent_path().parent_path() /
    "params/stop_turn_1cm_h16.json";
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("GridBased.lattice_filepath", file.string()),
      rclcpp::Parameter("GridBased.smooth_path", false),
      rclcpp::Parameter("GridBased.allow_unknown", false),
      rclcpp::Parameter("GridBased.allow_reverse_expansion", false),
      rclcpp::Parameter("GridBased.analytic_expansion_max_length", 0.0),
      rclcpp::Parameter("GridBased.rotation_penalty", 1.0),
      rclcpp::Parameter("GridBased.cost_penalty", 0.0),
      rclcpp::Parameter("GridBased.lookup_table_size", 1.0),
      rclcpp::Parameter("GridBased.max_planning_time", 2.0),
      rclcpp::Parameter("GridBased.max_iterations", 1000000),
      rclcpp::Parameter("GridBased.terminal_goal_binding", true),
      rclcpp::Parameter("GridBased.terminal_global_resolution", .01)});
  auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("near_goal_parent", options);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("near_goal_map");
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{})).successful);
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("resolution", .01)).successful);
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("width", 20)).successful);
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("height", 20)).successful);
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("footprint",
    "[[-0.647,-0.530],[0.647,-0.530],[0.647,0.530],[-0.647,0.530]]")).successful);
  ASSERT_TRUE(costmap->set_parameter(rclcpp::Parameter("footprint_padding", .15)).successful);
  ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
  costmap->getLayeredCostmap()->resizeMap(2000, 2000, .01, -6.28, -5.53, true);
  auto * grid = costmap->getCostmap();
  std::fill(grid->getCharMap(), grid->getCharMap() + 4000000, 0);
  SafeLatticePlanner planner;
  planner.configure(parent, "GridBased", nullptr, costmap);
  // Derived from A7's raw start on an all-free synthetic map. The old binary
  // produced a backwards moving turn here: two accepted g values differ by
  // one float ULP but have the same f priority. This is not a spatial epsilon.
  const auto start = pose(3.7146067237854012, 4.473530673980713, .0245255);
  const auto goal = pose(3.724999519389449, 4.4150007919562455, 0.);
  nav_msgs::msg::Path path;
  EXPECT_NO_THROW(path = planner.createPlan(start, goal, [] {return false;}));
  EXPECT_GT(path.poses.size(), 2u);
  if (!path.poses.empty()) {
    EXPECT_EQ(path.poses.back().pose, goal.pose);
    EXPECT_NO_THROW(robot_feasible_navigation::goal_region::splitBinding(path, .01, 20.));
  }
  planner.cleanup();
  costmap->on_cleanup(rclcpp_lifecycle::State());
}
