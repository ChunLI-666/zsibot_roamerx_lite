#ifndef ROBOT_NAVIGO__LIVOX_SCAN_PROJECTION_HPP_
#define ROBOT_NAVIGO__LIVOX_SCAN_PROJECTION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <string>
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

// Raw IMU values are retained; normalization below is only for rotation math.
struct ImuOrientationSample
{
  std::int64_t stamp_ns{0};
  std::array<double, 4> xyzw{0.0, 0.0, 0.0, 0.0};
  double orientation_covariance_0{0.0};
};

inline std::string invalidImuOrientation(const ImuOrientationSample & sample)
{
  if (sample.stamp_ns <= 0) {
    return "imu_stamp_invalid";
  }
  if (!std::isfinite(sample.orientation_covariance_0) ||
    sample.orientation_covariance_0 == -1.0)
  {
    return "imu_orientation_unavailable";
  }
  double squared_norm = 0.0;
  for (const double value : sample.xyzw) {
    if (!std::isfinite(value)) {
      return "imu_quaternion_nonfinite";
    }
    squared_norm += value * value;
  }
  const double norm = std::sqrt(squared_norm);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1.0e-3) {
    return "imu_quaternion_not_unit";
  }
  return {};
}

class ImuOrientationBuffer
{
public:
  explicit ImuOrientationBuffer(std::size_t capacity = 2048)
  : capacity_(std::max<std::size_t>(1, capacity)) {}

  bool add(const ImuOrientationSample & sample)
  {
    last_rejection_ = invalidImuOrientation(sample);
    if (!last_rejection_.empty()) {
      return false;
    }
    if (!samples_.empty() && sample.stamp_ns <= samples_.back().stamp_ns) {
      last_rejection_ = "imu_stamp_not_increasing";
      return false;
    }
    samples_.push_back(sample);
    while (samples_.size() > capacity_) {
      samples_.pop_front();
    }
    return true;
  }

  const ImuOrientationSample * nearest(std::int64_t stamp_ns) const
  {
    if (samples_.empty()) {
      return nullptr;
    }
    const auto after = std::lower_bound(
      samples_.begin(), samples_.end(), stamp_ns,
      [](const ImuOrientationSample & sample, std::int64_t stamp) {
        return sample.stamp_ns < stamp;
      });
    if (after == samples_.begin()) {
      return &*after;
    }
    if (after == samples_.end()) {
      return &samples_.back();
    }
    const auto before = std::prev(after);
    // A tie deterministically uses the earlier original sample.
    return stamp_ns - before->stamp_ns <= after->stamp_ns - stamp_ns ? &*before : &*after;
  }

  const std::string & lastRejection() const {return last_rejection_;}
  std::size_t size() const {return samples_.size();}
  std::size_t capacity() const {return capacity_;}

private:
  std::size_t capacity_;
  std::deque<ImuOrientationSample> samples_;
  std::string last_rejection_;
};

inline std::array<double, 9> multiplyRotation(
  const std::array<double, 9> & a, const std::array<double, 9> & b)
{
  std::array<double, 9> result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      for (std::size_t k = 0; k < 3; ++k) {
        result[row * 3 + col] += a[row * 3 + k] * b[k * 3 + col];
      }
    }
  }
  return result;
}

inline std::array<double, 9> transposeRotation(const std::array<double, 9> & r)
{
  return {r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]};
}

inline std::array<double, 3> rotateVector(
  const std::array<double, 9> & r, const std::array<double, 3> & p)
{
  return {
    r[0] * p[0] + r[1] * p[1] + r[2] * p[2],
    r[3] * p[0] + r[4] * p[1] + r[5] * p[2],
    r[6] * p[0] + r[7] * p[1] + r[8] * p[2]};
}

inline std::array<double, 9> imuRotation(const ImuOrientationSample & sample)
{
  const auto & q = sample.xyzw;
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  const double x = q[0] / norm, y = q[1] / norm, z = q[2] / norm, w = q[3] / norm;
  return {
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)};
}

struct FrameProjectionConfig
{
  ScanProjectionConfig config;
  bool ready{false};
  std::string drop_reason;
  std::int64_t imu_stamp_ns{0};
  std::int64_t absolute_stamp_delta_ns{0};
  std::array<double, 4> original_imu_xyzw{0.0, 0.0, 0.0, 0.0};
};

inline FrameProjectionConfig projectionConfigForFrame(
  const ScanProjectionConfig & base, bool stabilize_with_imu,
  const ImuOrientationBuffer & orientations, std::int64_t cloud_stamp_ns,
  std::int64_t max_stamp_delta_ns)
{
  FrameProjectionConfig result;
  result.config = base;
  if (!stabilize_with_imu) {
    result.ready = true;
    return result;
  }
  if (cloud_stamp_ns <= 0 || max_stamp_delta_ns <= 0) {
    result.drop_reason = "cloud_or_imu_delta_stamp_invalid";
    return result;
  }
  const auto * sample = orientations.nearest(cloud_stamp_ns);
  if (sample == nullptr) {
    result.drop_reason = "imu_orientation_missing";
    if (!orientations.lastRejection().empty()) {
      result.drop_reason += ":" + orientations.lastRejection();
    }
    return result;
  }
  result.imu_stamp_ns = sample->stamp_ns;
  result.absolute_stamp_delta_ns = cloud_stamp_ns >= sample->stamp_ns ?
    cloud_stamp_ns - sample->stamp_ns : sample->stamp_ns - cloud_stamp_ns;
  result.original_imu_xyzw = sample->xyzw;
  if (result.absolute_stamp_delta_ns > max_stamp_delta_ns) {
    result.drop_reason = "imu_stamp_delta_exceeded";
    return result;
  }
  // Contract: IMU and LiDAR axes are parallel (R_I_L=I). R_B_L is the
  // unchanged fixed scan mount rotation, so R_W_B=R_W_I*inverse(R_B_L).
  const auto world_body = multiplyRotation(imuRotation(*sample), transposeRotation(base.rotation));
  const double body_yaw = std::atan2(world_body[3], world_body[0]);
  const auto level_body = multiplyRotation(rotationFromRpy(0.0, 0.0, -body_yaw), world_body);
  // Rotate the complete mount transform, including its translation:
  // p_level=Rz(-yaw(R_W_B))*R_W_B*(R_B_L*p_L+t_B_L).
  result.config.rotation = multiplyRotation(level_body, base.rotation);
  result.config.translation = rotateVector(level_body, base.translation);
  result.ready = true;
  return result;
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
