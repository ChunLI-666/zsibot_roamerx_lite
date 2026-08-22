#ifndef ROBOT_NAVIGO__LIVOX_SCAN_PROJECTION_HPP_
#define ROBOT_NAVIGO__LIVOX_SCAN_PROJECTION_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace robot_navigo
{

struct ScanProjectionConfig
{
  double min_height{0.05};
  double max_height{0.45};
  double angle_min{-3.14159265358979323846};
  double angle_max{3.14159265358979323846};
  double angle_increment{0.0087};
  double range_min{0.1};
  double range_max{50.0};
  bool use_inf{true};
  bool exclude_robot_footprint{true};
  double robot_min_x{-0.35};
  double robot_max_x{0.35};
  double robot_min_y{-0.20};
  double robot_max_y{0.20};
  std::array<double, 3> translation{0.0, 0.0, 0.0};
  std::array<double, 9> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

struct ScanProjectionStats
{
  std::size_t points_used{0};
  std::size_t finite_bins{0};
};

inline std::array<double, 9> rotationFromRpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  return {
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr};
}

inline std::size_t scanBinCount(const ScanProjectionConfig & config)
{
  return static_cast<std::size_t>(
    (config.angle_max - config.angle_min) / config.angle_increment) + 1U;
}

inline std::vector<float> makeEmptyScan(const ScanProjectionConfig & config)
{
  const float empty_value = config.use_inf ?
    std::numeric_limits<float>::infinity() : static_cast<float>(config.range_max);
  return std::vector<float>(scanBinCount(config), empty_value);
}

inline bool projectPoint(
  double lidar_x, double lidar_y, double lidar_z,
  const ScanProjectionConfig & config,
  std::vector<float> & ranges,
  ScanProjectionStats & stats)
{
  const auto & r = config.rotation;
  const double x = r[0] * lidar_x + r[1] * lidar_y + r[2] * lidar_z + config.translation[0];
  const double y = r[3] * lidar_x + r[4] * lidar_y + r[5] * lidar_z + config.translation[1];
  const double z = r[6] * lidar_x + r[7] * lidar_y + r[8] * lidar_z + config.translation[2];
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }

  if (z < config.min_height || z > config.max_height) {
    return false;
  }

  if (config.exclude_robot_footprint &&
    x >= config.robot_min_x && x <= config.robot_max_x &&
    y >= config.robot_min_y && y <= config.robot_max_y)
  {
    return false;
  }

  const double range = std::hypot(x, y);
  if (!std::isfinite(range) || range < config.range_min || range > config.range_max) {
    return false;
  }

  const double angle = std::atan2(y, x);
  if (angle < config.angle_min || angle > config.angle_max) {
    return false;
  }

  const auto index = static_cast<std::size_t>((angle - config.angle_min) / config.angle_increment);
  if (index >= ranges.size()) {
    return false;
  }

  ++stats.points_used;
  const float range_value = static_cast<float>(range);
  if (!std::isfinite(ranges[index])) {
    ++stats.finite_bins;
  }
  if (range_value < ranges[index]) {
    ranges[index] = range_value;
  }
  return true;
}

}  // namespace robot_navigo

#endif  // ROBOT_NAVIGO__LIVOX_SCAN_PROJECTION_HPP_
