#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robots_dog_msgs/msg/livox_bridge_debug.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "robot_navigo/livox_scan_projection.hpp"

namespace robot_navigo
{

class LivoxScanProjector : public rclcpp::Node
{
public:
  LivoxScanProjector()
  : Node("livox_scan_projector")
  {
    qos_depth_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter("sensor_qos_depth", 1)));
    max_sensor_age_sec_ = declare_parameter("max_sensor_age_sec", 0.3);
    target_frame_ = declare_parameter("target_frame", "base_link");
    scan_time_ = declare_parameter("scan_time", 0.1);

    const double lidar_x = declare_parameter("lidar_x", 0.0);
    const double lidar_y = declare_parameter("lidar_y", 0.0);
    const double lidar_z = declare_parameter("lidar_z", 0.0);
    const double lidar_roll = declare_parameter("lidar_roll", 0.0);
    const double lidar_pitch = declare_parameter("lidar_pitch", -0.2618);
    const double lidar_yaw = declare_parameter("lidar_yaw", 0.0);

    config_.translation = {lidar_x, lidar_y, lidar_z};
    // Keep the established bridge convention: lidar_* is the physical mount
    // direction and the inverse RPY levels raw points into base_link.
    config_.rotation = rotationFromRpy(-lidar_roll, -lidar_pitch, -lidar_yaw);
    config_.min_height = declare_parameter("min_height", 0.05);
    config_.max_height = declare_parameter("max_height", 0.45);
    config_.angle_min = declare_parameter("angle_min", -3.14159265358979323846);
    config_.angle_max = declare_parameter("angle_max", 3.14159265358979323846);
    config_.angle_increment = declare_parameter("angle_increment", 0.0087);
    config_.range_min = declare_parameter("range_min", 0.1);
    config_.range_max = declare_parameter("range_max", 50.0);
    config_.use_inf = declare_parameter("use_inf", true);
    config_.exclude_robot_footprint = declare_parameter("exclude_robot_footprint", true);
    config_.robot_min_x = declare_parameter("robot_min_x", -0.35);
    config_.robot_max_x = declare_parameter("robot_max_x", 0.35);
    config_.robot_min_y = declare_parameter("robot_min_y", -0.20);
    config_.robot_max_y = declare_parameter("robot_max_y", 0.20);

    validateConfig();

    const std::string input_type = declare_parameter("input_type", "livox_custom");
    const std::string input_topic = declare_parameter("livox_input_topic", "/livox/lidar");
    const std::string output_topic = declare_parameter("laserscan_output_topic", "/laser_scan");
    const std::string debug_topic = declare_parameter(
      "bridge_debug_topic", "/lightning_bridge/debug");

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth_)).best_effort().durability_volatile();
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos);
    debug_pub_ = create_publisher<robots_dog_msgs::msg::LivoxBridgeDebug>(debug_topic, qos);
    if (input_type == "livox_custom") {
      livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic, qos,
        std::bind(&LivoxScanProjector::onLivox, this, std::placeholders::_1));
    } else if (input_type == "pointcloud2") {
      pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, qos,
        std::bind(&LivoxScanProjector::onPointCloud2, this, std::placeholders::_1));
    } else {
      throw std::invalid_argument("input_type must be 'livox_custom' or 'pointcloud2'");
    }

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTransform(lidar_x, lidar_y, lidar_z, -lidar_roll, -lidar_pitch, -lidar_yaw);

    RCLCPP_INFO(
      get_logger(),
      "Direct scan projection (%s): %s -> %s, frame=%s, bins=%zu, height=[%.2f, %.2f]",
      input_type.c_str(), input_topic.c_str(), output_topic.c_str(), target_frame_.c_str(),
      scanBinCount(config_),
      config_.min_height, config_.max_height);
  }

private:
  void validateConfig() const
  {
    if (config_.angle_increment <= 0.0 || config_.angle_max <= config_.angle_min) {
      throw std::invalid_argument("invalid LaserScan angle configuration");
    }
    if (config_.max_height < config_.min_height) {
      throw std::invalid_argument("max_height must be >= min_height");
    }
    if (config_.range_min < 0.0 || config_.range_max <= config_.range_min) {
      throw std::invalid_argument("invalid LaserScan range configuration");
    }
    if (config_.robot_max_x < config_.robot_min_x || config_.robot_max_y < config_.robot_min_y) {
      throw std::invalid_argument("invalid robot footprint exclusion bounds");
    }
  }

  void publishStaticTransform(
    double x, double y, double z, double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = target_frame_;
    transform.child_frame_id = "livox_frame";
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    transform.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    transform.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    transform.transform.rotation.z = cr * cp * sy - sr * sp * cy;
    static_tf_broadcaster_->sendTransform(transform);
  }

  void onLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    projectMessage(
      msg->header, static_cast<std::uint32_t>(msg->points.size()),
      [msg](const ScanProjectionConfig & config, std::vector<float> & ranges,
      ScanProjectionStats & stats) {
        for (const auto & point : msg->points) {
          projectPoint(point.x, point.y, point.z, config, ranges, stats);
        }
      });
  }

  void onPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    projectMessage(
      msg->header, msg->width * msg->height,
      [msg](const ScanProjectionConfig & config, std::vector<float> & ranges,
      ScanProjectionStats & stats) {
        sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
        for (; x != x.end(); ++x, ++y, ++z) {
          projectPoint(*x, *y, *z, config, ranges, stats);
        }
      });
  }

  template<typename ProjectPoints>
  void projectMessage(
    const std_msgs::msg::Header & header, std::uint32_t input_points,
    ProjectPoints project_points)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    const auto callback_now = now();

    robots_dog_msgs::msg::LivoxBridgeDebug debug;
    debug.header.stamp = callback_now;
    debug.header.frame_id = target_frame_;
    debug.input_stamp = header.stamp;
    debug.input_frame_id = header.frame_id;
    debug.callback_sequence = ++callback_sequence_;
    debug.input_points = input_points;
    debug.subscriber_qos_depth = static_cast<std::uint32_t>(qos_depth_);

    const rclcpp::Time input_stamp(header.stamp, get_clock()->get_clock_type());
    if (input_stamp.nanoseconds() > 0) {
      debug.input_age_sec = (callback_now - input_stamp).seconds();
    }
    if (max_sensor_age_sec_ > 0.0 && debug.input_age_sec > max_sensor_age_sec_) {
      debug.stale_input = true;
      debug.dropped_input = true;
      debug.drop_reason = "input_age_exceeded";
      finishAndPublishDebug(callback_start, debug);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping stale Livox frame: age=%.3fs limit=%.3fs",
        debug.input_age_sec, max_sensor_age_sec_);
      return;
    }

    const auto scan_start = std::chrono::steady_clock::now();
    auto ranges = makeEmptyScan(config_);
    ScanProjectionStats stats;
    try {
      project_points(config_, ranges, stats);
    } catch (const std::runtime_error & error) {
      debug.dropped_input = true;
      debug.drop_reason = std::string("invalid_pointcloud_fields: ") + error.what();
      finishAndPublishDebug(callback_start, debug);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s", debug.drop_reason.c_str());
      return;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header = header;
    scan.header.frame_id = target_frame_;
    scan.angle_min = static_cast<float>(config_.angle_min);
    scan.angle_max = static_cast<float>(config_.angle_max);
    scan.angle_increment = static_cast<float>(config_.angle_increment);
    scan.time_increment = 0.0F;
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(config_.range_min);
    scan.range_max = static_cast<float>(config_.range_max);
    scan.ranges = std::move(ranges);
    scan_pub_->publish(scan);

    debug.scan_points_used = static_cast<std::uint32_t>(stats.points_used);
    debug.finite_scan_bins = static_cast<std::uint32_t>(stats.finite_bins);
    debug.laserscan_published = true;
    debug.laserscan_build_ms = elapsedMs(scan_start);
    finishAndPublishDebug(callback_start, debug);
  }

  static double elapsedMs(const std::chrono::steady_clock::time_point & start)
  {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  }

  void finishAndPublishDebug(
    const std::chrono::steady_clock::time_point & callback_start,
    robots_dog_msgs::msg::LivoxBridgeDebug & debug)
  {
    debug.callback_duration_ms = elapsedMs(callback_start);
    debug_pub_->publish(debug);
  }

  ScanProjectionConfig config_;
  double max_sensor_age_sec_{0.3};
  double scan_time_{0.1};
  std::string target_frame_;
  std::uint64_t callback_sequence_{0};
  std::size_t qos_depth_{1};
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<robots_dog_msgs::msg::LivoxBridgeDebug>::SharedPtr debug_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

}  // namespace robot_navigo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_navigo::LivoxScanProjector>());
  rclcpp::shutdown();
  return 0;
}
