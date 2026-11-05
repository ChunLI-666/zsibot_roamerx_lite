// Copyright 2026. SPDX-License-Identifier: Apache-2.0
#include "robot_feasible_navigation/safe_lattice_planner.hpp"

#include <array>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>
#include "nav2_core/planner_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_feasible_navigation
{
namespace
{
double difference(double a, double b) {return std::atan2(std::sin(b - a), std::cos(b - a));}
void require(bool condition, const std::string & reason)
{
  if (!condition) {throw std::runtime_error("SafeLatticePlanner: " + reason);}
}
bool poseFinite(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::abs(p.z) < 1e-5 &&
         std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3 &&
         std::abs(q.x) < 1e-5 && std::abs(q.y) < 1e-5;
}
geometry_msgs::msg::PoseStamped atYaw(
  const geometry_msgs::msg::PoseStamped & pose, double yaw)
{
  auto result = pose;
  result.pose.orientation.x = result.pose.orientation.y = 0.0;
  result.pose.orientation.z = std::sin(yaw * 0.5);
  result.pose.orientation.w = std::cos(yaw * 0.5);
  return result;
}
void connectStopTurn(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & target,
  nav2_costmap_2d::Costmap2D & map, const sweep::Footprint & footprint,
  const std::function<bool()> & canceled)
{
  const auto current = path.poses.back();
  const double dx = target.pose.position.x - current.pose.position.x;
  const double dy = target.pose.position.y - current.pose.position.y;
  if (std::hypot(dx, dy) > 1e-7) {
    // A continuous exact endpoint adapter: align, translate without yaw change,
    // then turn to the original requested yaw. Never silently replace the goal
    // with the nearest lattice heading or introduce lateral body translation.
    const double heading = std::atan2(dy, dx);
    SafeLatticePlanner::appendCheckedSegment(path, atYaw(current, heading), map, footprint, canceled);
    SafeLatticePlanner::appendCheckedSegment(path, atYaw(target, heading), map, footprint, canceled);
  }
  SafeLatticePlanner::appendCheckedSegment(path, target, map, footprint, canceled);
}
}  // namespace

void SafeLatticePlanner::validateControlSet(const nlohmann::json & controls, double resolution)
{
  const auto & metadata = controls.at("lattice_metadata");
  require(std::isfinite(resolution) && std::abs(resolution - 0.01) < 1e-8,
    "candidate requires an actual 1cm costmap");
  require(metadata.at("motion_model").get<std::string>() == "diff", "expected genuine diff model");
  require(std::abs(metadata.at("grid_resolution").get<double>() - resolution) < 1e-8,
    "primitive resolution mismatch");
  const double radius = metadata.at("turning_radius").get<double>();
  require(std::isfinite(radius) && radius > 0.0, "heuristic radius must stay positive");
  const auto headings = metadata.at("heading_angles").get<std::vector<double>>();
  require(headings.size() == 16 && metadata.at("num_of_headings").get<int>() == 16,
    "expected all sixteen nonuniform headings");
  require(std::abs(headings.front()) < 1e-8, "first heading must be zero");
  for (std::size_t i = 0; i < headings.size(); ++i) {
    require(std::isfinite(headings[i]) && headings[i] >= 0.0 && headings[i] < 2.0 * M_PI &&
      (i == 0 || headings[i] > headings[i - 1]), "invalid heading table");
  }
  const auto & primitives = controls.at("primitives");
  require(primitives.size() == 48 && metadata.at("number_of_trajectories").get<int>() == 48,
    "expected exactly 48 stop-turn primitives");
  std::array<std::array<int, 3>, 16> seen{};
  int prior_start = -1;
  for (std::size_t id = 0; id < primitives.size(); ++id) {
    const auto & primitive = primitives[id];
    require(primitive.at("trajectory_id").get<std::size_t>() == id, "non-contiguous trajectory ids");
    const int start = primitive.at("start_angle_index").get<int>();
    const int end = primitive.at("end_angle_index").get<int>();
    require(start >= 0 && start < 16 && end >= 0 && end < 16 &&
      start >= prior_start && start <= prior_start + 1, "invalid or unordered heading bucket");
    prior_start = start;
    const auto poses = primitive.at("poses").get<std::vector<std::vector<double>>>();
    require(poses.size() >= 2 && poses.size() <= 10000, "invalid primitive sample count");
    const double length = primitive.at("trajectory_length").get<double>();
    for (const auto & key : {"trajectory_length", "trajectory_radius", "arc_length", "straight_length"}) {
      const double value = primitive.at(key).get<double>();
      require(std::isfinite(value) && value >= 0.0, "nonfinite or negative primitive metric");
    }
    require(primitive.at("left_turn").is_boolean(), "missing primitive turn direction");
    double total_angle = 0.0;
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const auto & pose = poses[i];
      require(pose.size() == 3 && std::isfinite(pose[0]) && std::isfinite(pose[1]) &&
        std::isfinite(pose[2]), "invalid primitive pose");
      if (i) {total_angle += difference(poses[i - 1][2], pose[2]);}
    }
    require(std::hypot(poses.front()[0], poses.front()[1]) < 1e-8 &&
      std::abs(difference(headings[start], poses.front()[2])) < 1e-7 &&
      std::abs(difference(headings[end], poses.back()[2])) < 1e-7,
      "primitive endpoint heading mismatch");
    if (length < 1e-8) {
      const int direction = end == (start + 1) % 16 ? 1 : end == (start + 15) % 16 ? -1 : 0;
      require(direction != 0, "rotation must end at an adjacent heading");
      const double expected = difference(headings[start], headings[end]);
      require(std::abs(total_angle - expected) < 1e-7, "rotation wraps or reverses unexpectedly");
      for (std::size_t i = 0; i < poses.size(); ++i) {
        require(std::hypot(poses[i][0], poses[i][1]) < 1e-8, "rotation translated");
        if (i) {
          const double delta = difference(poses[i - 1][2], poses[i][2]);
          require(delta * direction >= -1e-10 && std::abs(delta) <= 0.05,
            "rotation sampling direction or angle bound invalid");
        }
      }
      ++seen[start][direction > 0 ? 1 : 2];
    } else {
      require(start == end && std::abs(total_angle) < 1e-8, "moving curve is outside stop-turn model");
      double previous_progress = -1e-8;
      require(length <= std::sqrt(5.0) * resolution + 1e-5,
        "straight stride exceeds the reviewed 1:2 grid-vector candidate");
      for (const auto & pose : poses) {
        const double progress = pose[0] * std::cos(headings[start]) + pose[1] * std::sin(headings[start]);
        require(progress >= previous_progress - 1e-8, "straight samples backtrack");
        previous_progress = progress;
        require(std::abs(difference(headings[start], pose[2])) < 1e-7 &&
          std::abs(pose[0] * std::sin(headings[start]) - pose[1] * std::cos(headings[start])) < 1e-7 &&
          pose[0] * std::cos(headings[start]) + pose[1] * std::sin(headings[start]) >= -1e-8,
          "straight primitive is lateral or reverses");
      }
      const auto & endpoint = poses.back();
      require(std::abs(endpoint[0] / resolution - std::round(endpoint[0] / resolution)) < 1e-6 &&
        std::abs(endpoint[1] / resolution - std::round(endpoint[1] / resolution)) < 1e-6 &&
        std::abs(std::hypot(endpoint[0], endpoint[1]) - length) < 1e-5,
        "straight endpoint is not a true lattice cell");
      ++seen[start][0];
    }
  }
  for (const auto & count : seen) {require(count == std::array<int, 3>{1, 1, 1}, "incomplete heading actions");}
}

void SafeLatticePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  require(static_cast<bool>(node), "missing lifecycle parent");
  // Normal Nav2 LifecycleNode does not automatically declare YAML overrides.
  // Require each safety-critical value to be explicit, declare that override,
  // then validate; never require has_parameter() before reading overrides.
  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & key : {"lattice_filepath", "smooth_path", "allow_unknown",
      "allow_reverse_expansion", "analytic_expansion_max_length", "rotation_penalty"})
  {
    const auto full = name + "." + key;
    if (!node->has_parameter(full)) {
      const auto override = overrides.find(full);
      require(override != overrides.end(), "missing explicit override " + full);
      node->declare_parameter(full, override->second);
    }
  }
  control_file_ = node->get_parameter(name + ".lattice_filepath").as_string();
  std::ifstream stream(control_file_);
  require(stream.good(), "cannot open the control file");
  nlohmann::json controls;
  stream >> controls;
  validateControlSet(controls, costmap_ros->getCostmap()->getResolution());
  require(sweep::validFootprint(costmap_ros->getRobotFootprint()), "requires convex polygon footprint");
  for (const auto & key : {"smooth_path", "allow_unknown", "allow_reverse_expansion"}) {
    require(node->has_parameter(name + "." + key) && !node->get_parameter(name + "." + key).as_bool(),
      std::string(key) + " must explicitly be false");
  }
  require(node->has_parameter(name + ".analytic_expansion_max_length") &&
    node->get_parameter(name + ".analytic_expansion_max_length").as_double() == 0.0,
    "analytic connectors must explicitly be disabled for this diff stop-turn candidate");
  require(node->has_parameter(name + ".rotation_penalty") &&
    std::isfinite(node->get_parameter(name + ".rotation_penalty").as_double()) &&
    node->get_parameter(name + ".rotation_penalty").as_double() > 0.0,
    "positive explicit rotation penalty required");
  robot_feasible_navigation_smac::SmacPlannerLattice::configure(parent, name, tf, costmap_ros);
  // This test candidate is an immutable profile. Avoid a dynamic change silently
  // re-enabling unknown space, smoothing, analytic curves or a different model.
  _dyn_params_handler.reset();
  safe_parameter_guard_ = node->add_on_set_parameters_callback(
    [prefix = name + "."] (const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        if (parameter.get_name().rfind(prefix, 0) == 0) {
          result.successful = false;
          result.reason = "SafeLatticePlanner candidate profile is immutable after configure";
          break;
        }
      }
      return result;
    });
}

void SafeLatticePlanner::restoreSearchStartRoundoff(
  geometry_msgs::msg::PoseStamped & searched_start,
  const geometry_msgs::msg::PoseStamped & exact_start,
  nav2_costmap_2d::Costmap2D & map)
{
  // Exact provenance of upstream float round-trip, not an arbitrary spatial
  // tolerance: worldToMapContinuous casts cell coordinates to float, and
  // getWorldCoords separately casts the origin to float. Reject any different
  // first search point instead of hiding a genuine connector.
  const double resolution = map.getResolution();
  const auto expected = [&](double world, double origin) {
      const float cells = static_cast<float>((world - origin) / resolution);
      return static_cast<double>(static_cast<float>(origin)) + cells * resolution;
    };
  const double ex = expected(exact_start.pose.position.x, map.getOriginX());
  const double ey = expected(exact_start.pose.position.y, map.getOriginY());
  const double bound = 64.0 * std::numeric_limits<double>::epsilon() * std::max(
    {1.0, std::abs(ex), std::abs(ey)});
  require(std::abs(searched_start.pose.position.x - ex) <= bound &&
    std::abs(searched_start.pose.position.y - ey) <= bound,
    "search start differs from its documented float round-trip");
  searched_start.pose.position.x = exact_start.pose.position.x;
  searched_start.pose.position.y = exact_start.pose.position.y;
}

void SafeLatticePlanner::restoreSearchRoundoff(
  nav_msgs::msg::Path & searched,
  const geometry_msgs::msg::PoseStamped & exact_start,
  const geometry_msgs::msg::PoseStamped & exact_goal,
  nav2_costmap_2d::Costmap2D & map)
{
  require(!searched.poses.empty(), "search returned an empty path");
  auto first = searched.poses.front();
  restoreSearchStartRoundoff(first, exact_start, map);  // Validate before mutating any pose.
  const auto rounded = [&](const geometry_msgs::msg::PoseStamped & exact) {
      return std::array<double, 2>{
        static_cast<double>(static_cast<float>(map.getOriginX())) +
        static_cast<float>((exact.pose.position.x - map.getOriginX()) / map.getResolution()) *
        map.getResolution(),
        static_cast<double>(static_cast<float>(map.getOriginY())) +
        static_cast<float>((exact.pose.position.y - map.getOriginY()) / map.getResolution()) *
        map.getResolution()};
    };
  const auto start_xy = rounded(exact_start), goal_xy = rounded(exact_goal);
  const auto matches = [](const geometry_msgs::msg::PoseStamped & pose,
      const std::array<double, 2> & expected) {
      const double bound = 64.0 * std::numeric_limits<double>::epsilon() *
        std::max({1.0, std::abs(expected[0]), std::abs(expected[1])});
      return std::abs(pose.pose.position.x - expected[0]) <= bound &&
             std::abs(pose.pose.position.y - expected[1]) <= bound;
    };
  // A pure-turn primitive has many poses with one identical float XY. Restoring
  // only the first/last pose invents micrometer XY motion while yaw changes.
  // Restore every pose with the SAME proven endpoint round-trip, preserving
  // all yaw samples; genuine sub-cell offsets are not matched or hidden.
  for (auto & pose : searched.poses) {
    if (matches(pose, start_xy)) {
      pose.pose.position.x = exact_start.pose.position.x;
      pose.pose.position.y = exact_start.pose.position.y;
    } else if (matches(pose, goal_xy)) {
      pose.pose.position.x = exact_goal.pose.position.x;
      pose.pose.position.y = exact_goal.pose.position.y;
    }
  }
}

void SafeLatticePlanner::appendCheckedSegment(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & target,
  nav2_costmap_2d::Costmap2D & map, const sweep::Footprint & footprint,
  const std::function<bool()> & cancel_checker)
{
  if (cancel_checker && cancel_checker()) {throw nav2_core::PlannerCancelled("safety sweep canceled");}
  require(!path.poses.empty() && poseFinite(target) && poseFinite(path.poses.back()), "invalid SE2 adapter pose");
  const auto begin = path.poses.back();
  const double x0 = begin.pose.position.x, y0 = begin.pose.position.y, a0 = tf2::getYaw(begin.pose.orientation);
  const double x1 = target.pose.position.x, y1 = target.pose.position.y, a1 = tf2::getYaw(target.pose.orientation);
  if (!sweep::sweepFree(map, footprint, x0, y0, a0, x1, y1, a1)) {
    throw nav2_core::NoValidPathCouldBeFound("filled footprint blocked a normal rotation/translation segment");
  }
  const double dyaw = difference(a0, a1);
  double radius = 0.0;
  for (const auto & point : footprint) {radius = std::max(radius, std::hypot(point.x, point.y));}
  const auto steps = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(std::max(
    (std::hypot(x1 - x0, y1 - y0) + radius * std::abs(dyaw)) / (0.5 * map.getResolution()),
    std::abs(dyaw) / 0.02))));
  require(steps <= 1000000 && path.poses.size() + steps <= 1000000, "adapter output budget exceeded");
  if (std::hypot(x1 - x0, y1 - y0) < 1e-10 && std::abs(dyaw) < 1e-10) {
    path.poses.back() = target;
    return;
  }
  for (std::size_t i = 1; i <= steps; ++i) {
    if (cancel_checker && cancel_checker()) {throw nav2_core::PlannerCancelled("path output canceled");}
    const double ratio = static_cast<double>(i) / steps;
    auto pose = atYaw(target, a0 + ratio * dyaw);
    pose.pose.position.x = x0 + ratio * (x1 - x0);
    pose.pose.position.y = y0 + ratio * (y1 - y0);
    path.poses.push_back(pose);
  }
  path.poses.back() = target;  // exact requested endpoint, not a snapped heading
}

nav_msgs::msg::Path SafeLatticePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> canceled)
{
  std::lock_guard<std::mutex> request_lock(request_mutex_);
  require(_costmap != nullptr, "not configured");
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*_costmap->getMutex());
  require(poseFinite(start) && poseFinite(goal), "invalid planar request");
  if (start.header.frame_id != _global_frame || goal.header.frame_id != _global_frame) {
    throw nav2_core::PlannerTFError("SafeLatticePlanner requires requests in its global frame");
  }
  if (canceled && canceled()) {throw nav2_core::PlannerCancelled("request canceled");}
  const auto footprint = _costmap_ros->getRobotFootprint();
  require(std::abs(_costmap->getResolution() - 0.01) < 1e-8, "runtime map resolution changed");
  // Rebuilt for this request while map_lock prevents all costmap writes. Empty
  // tiles skip only scans that would otherwise inspect exclusively free cells.
  sweep::ScopedObstacleIndex obstacle_index(*_costmap);
  if (!sweep::footprintFree(*_costmap, footprint, start.pose.position.x, start.pose.position.y,
    tf2::getYaw(start.pose.orientation))) {throw nav2_core::StartOccupied("filled start footprint occupied");}
  if (!sweep::footprintFree(*_costmap, footprint, goal.pose.position.x, goal.pose.position.y,
    tf2::getYaw(goal.pose.orientation))) {throw nav2_core::GoalOccupied("filled goal footprint occupied");}
  unsigned int sx, sy, gx, gy;
  require(_costmap->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) &&
    _costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy), "request outside map");
  nav_msgs::msg::Path output;
  output.header.frame_id = _global_frame;
  output.header.stamp = _clock->now();
  auto exact_start = start;
  exact_start.header = output.header;
  output.poses.push_back(exact_start);
  auto exact_goal = goal;
  exact_goal.header = output.header;
  if (sx == gx && sy == gy) {
    connectStopTurn(output, exact_goal, *_costmap, footprint, canceled);
  } else {
    auto searched = robot_feasible_navigation_smac::SmacPlannerLattice::createPlan(start, goal, canceled);
    restoreSearchRoundoff(searched, exact_start, exact_goal, *_costmap);
    auto first = searched.poses.front();
    first.header = output.header;
    // This is now genuinely sameXY: only the initial heading-bin alignment is
    // added. Micrometer float conversion does not create a 180-degree detour.
    appendCheckedSegment(output, first, *_costmap, footprint, canceled);
    for (std::size_t i = 1; i < searched.poses.size(); ++i) {
      auto pose = searched.poses[i];
      pose.header = output.header;
      const auto & previous = output.poses.back().pose;
      const double dx = pose.pose.position.x - previous.position.x;
      const double dy = pose.pose.position.y - previous.position.y;
      const double yaw = tf2::getYaw(previous.orientation);
      const double angular = difference(yaw, tf2::getYaw(pose.pose.orientation));
      if (std::hypot(dx, dy) > 1e-7) {
        // Each upstream cell coordinate is float; one integer-cell addition
        // can round once more. Bound lateral roundoff in cell units rather
        // than accepting a fixed centimeter-scale sideways connector.
        const double cell_scale = std::max({1.0,
          std::abs((previous.position.x - _costmap->getOriginX()) / _costmap->getResolution()),
          std::abs((previous.position.y - _costmap->getOriginY()) / _costmap->getResolution()),
          std::abs((pose.pose.position.x - _costmap->getOriginX()) / _costmap->getResolution()),
          std::abs((pose.pose.position.y - _costmap->getOriginY()) / _costmap->getResolution())});
        const double roundoff = 4.0 * std::numeric_limits<float>::epsilon() *
          (cell_scale * _costmap->getResolution() +
          std::max(std::abs(_costmap->getOriginX()), std::abs(_costmap->getOriginY())));
        require(std::abs(angular) < 1e-7 &&
          std::abs(dx * std::sin(yaw) - dy * std::cos(yaw)) <= roundoff &&
          dx * std::cos(yaw) + dy * std::sin(yaw) >= -1e-8,
          "search output is not an executable forward stop-turn segment");
      }
      appendCheckedSegment(output, pose, *_costmap, footprint, canceled);
    }
    connectStopTurn(output, exact_goal, *_costmap, footprint, canceled);
  }
  require(output.poses.back().pose == exact_goal.pose, "exact original goal was not retained");
  return output;
}
}  // namespace robot_feasible_navigation

PLUGINLIB_EXPORT_CLASS(robot_feasible_navigation::SafeLatticePlanner, nav2_core::GlobalPlanner)
