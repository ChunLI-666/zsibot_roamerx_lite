#include "gamepad_lcmt.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <lcm/lcm-cpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "robot_navigo/motion_limits.hpp"

class VelCmdLcmPublisher : public rclcpp::Node
{
public:
    VelCmdLcmPublisher()
        : Node("vel_cmd_lcm_publisher"),
          lc("udpm://239.255.76.67:7667?ttl=255")
    {
        const auto cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        lcm_channel_ = this->declare_parameter<std::string>("lcm_channel", "vel_cmd_lcm_data");
        lcm_type_hash_override_ = static_cast<uint64_t>(
            this->declare_parameter<int64_t>("lcm_type_hash_override", 0));
        matrix_legacy_gamepad_schema_ =
            this->declare_parameter<bool>("matrix_legacy_gamepad_schema", false);
        navigation_mode_ = this->declare_parameter<int>("navigation_mode", 0);
        const auto cmd_timeout_ms = this->declare_parameter<int>("cmd_timeout_ms", 300);
        const auto publish_rate_hz = std::clamp(
            this->declare_parameter<double>("publish_rate_hz", 50.0), 1.0, 200.0);
        min_abs_vx_ = std::max(0.0, this->declare_parameter<double>(
            "min_abs_vx", robot_navigo::motion_limits::kExecutableMinVx));
        min_abs_vy_ = std::max(0.0, this->declare_parameter<double>(
            "min_abs_vy", robot_navigo::motion_limits::kExecutableMinVy));
        min_abs_wz_ = std::max(0.0, this->declare_parameter<double>(
            "min_abs_wz", robot_navigo::motion_limits::kExecutableMinWz));
        vx_to_stick_scale_ = PositiveScale("vx_to_stick_scale");
        vy_to_stick_scale_ = PositiveScale("vy_to_stick_scale");
        wz_to_stick_scale_ = PositiveScale("wz_to_stick_scale");
        invert_lateral_axis_ =
            this->declare_parameter<bool>("invert_lateral_axis", false);
        const auto enable_stance_service =
            this->declare_parameter<bool>("enable_stance_service", false);
        stand_button_hold_ = std::chrono::milliseconds(std::max<int64_t>(
            50, this->declare_parameter<int>("stand_button_hold_ms", 500)));
        cmd_timeout_ = std::chrono::milliseconds(std::max<int64_t>(1, cmd_timeout_ms));
        last_command_time_ = std::chrono::steady_clock::now();
        planner_vel_cmd_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            std::bind(&VelCmdLcmPublisher::HandlPlannerVelCallback, this, std::placeholders::_1));
        command_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz)),
            std::bind(&VelCmdLcmPublisher::PublishLatestCommand, this));
        if (enable_stance_service) {
            stand_up_service_ = this->create_service<std_srvs::srv::Trigger>(
                "~/stand_up",
                std::bind(&VelCmdLcmPublisher::HandleStandUp, this,
                          std::placeholders::_1, std::placeholders::_2));
        }
        RCLCPP_INFO(this->get_logger(),
                    "vel_cmd_lcm_publisher started: topic=%s, channel=%s, rate=%.1fHz, timeout=%ldms, hash_override=0x%016lx, matrix_legacy_schema=%s, stick_scale=[%.3f %.3f %.3f], invert_lateral_axis=%s",
                    cmd_vel_topic.c_str(), lcm_channel_.c_str(),
                    publish_rate_hz,
                    static_cast<long>(cmd_timeout_ms),
                    static_cast<unsigned long>(lcm_type_hash_override_),
                    matrix_legacy_gamepad_schema_ ? "true" : "false",
                    vx_to_stick_scale_, vy_to_stick_scale_, wz_to_stick_scale_,
                    invert_lateral_axis_ ? "true" : "false");
    }

private:
    double PositiveScale(const std::string & parameter_name)
    {
        const double value = this->declare_parameter<double>(parameter_name, 1.0);
        if (std::isfinite(value) && value > 0.0) {
            return value;
        }
        RCLCPP_WARN(this->get_logger(), "Invalid %s=%.3f; using 1.0", parameter_name.c_str(), value);
        return 1.0;
    }

    void HandlPlannerVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(planner_vel_mutex_);
        last_command_time_ = std::chrono::steady_clock::now();
        latest_vx_ = std::isfinite(msg->linear.x) ? msg->linear.x : 0.0;
        latest_vy_ = std::isfinite(msg->linear.y) ? msg->linear.y : 0.0;
        latest_wz_ = std::isfinite(msg->angular.z) ? msg->angular.z : 0.0;
    }

    void PublishLatestCommand()
    {
        std::lock_guard<std::mutex> lk(planner_vel_mutex_);
        if (std::chrono::steady_clock::now() - last_command_time_ > cmd_timeout_) {
            PublishCommand(0.0, 0.0, 0.0);
        } else {
            PublishCommand(latest_vx_, latest_vy_, latest_wz_);
        }
    }

    void HandleStandUp(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lk(planner_vel_mutex_);
        // A stance transition invalidates any pre-existing motion command.
        // Keep the timer on zero until a fresh ROS command arrives.
        latest_vx_ = 0.0;
        latest_vy_ = 0.0;
        latest_wz_ = 0.0;
        gamepad_lcmt press{};
        press.navigation_mode = navigation_mode_;
        press.leftBumper = 1;
        press.y = 1;
        const bool press_ok = PublishLcm(press);
        std::this_thread::sleep_for(stand_button_hold_);

        gamepad_lcmt release{};
        release.navigation_mode = navigation_mode_;
        const bool release_ok = PublishLcm(release);
        last_command_time_ = std::chrono::steady_clock::now();
        response->success = press_ok && release_ok;
        response->message = response->success
            ? "Matrix stand-up button sequence published"
            : "Failed to publish Matrix stand-up button sequence";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    bool PublishLcm(const gamepad_lcmt & message)
    {
        if (lcm_type_hash_override_ == 0) {
            return lc.publish(lcm_channel_, &message) == 0;
        }

        std::vector<uint8_t> encoded(static_cast<size_t>(message.getEncodedSize()));
        const int encoded_size = message.encode(
            encoded.data(), 0, static_cast<int>(encoded.size()));
        if (encoded_size < 8) {
            RCLCPP_ERROR(this->get_logger(), "Failed to encode gamepad_lcmt: %d", encoded_size);
            return false;
        }
        auto payload_size = static_cast<unsigned int>(encoded_size);
        if (matrix_legacy_gamepad_schema_) {
            // The packaged Matrix mc_ctrl expects one additional int32 after
            // the 13 button/mode fields. Its generated decoder rejects the
            // repository's native 88-byte payload and accepts this 92-byte
            // layout; keep robot_state_flag at the end.
            constexpr size_t kLegacyFieldOffset = 8 + 13 * sizeof(int32_t);
            if (encoded.size() < kLegacyFieldOffset) {
                RCLCPP_ERROR(this->get_logger(), "gamepad_lcmt is too short for Matrix schema");
                return false;
            }
            encoded.insert(encoded.begin() + kLegacyFieldOffset, sizeof(int32_t), 0);
            payload_size = static_cast<unsigned int>(encoded.size());
        }
        for (size_t index = 0; index < 8; ++index) {
            encoded[index] = static_cast<uint8_t>(
                lcm_type_hash_override_ >> (8 * (7 - index)));
        }
        return lc.publish(
            lcm_channel_, encoded.data(), payload_size) == 0;
    }

    void PublishCommand(double vx, double vy, double wz)
    {
        gamepad_lcmt lcmt{};
        lcmt.navigation_mode = navigation_mode_;
        lcmt.leftStickAnalog[1] = std::fabs(vx) < min_abs_vx_
            ? 0.0 : std::clamp(vx * vx_to_stick_scale_, -1.0, 1.0);
        const double lateral_direction = invert_lateral_axis_ ? -1.0 : 1.0;
        lcmt.leftStickAnalog[0] = std::fabs(vy) < min_abs_vy_
            ? 0.0 : std::clamp(
                lateral_direction * vy * vy_to_stick_scale_, -1.0, 1.0);
        lcmt.rightStickAnalog[0] = std::fabs(wz) < min_abs_wz_
            ? 0.0 : std::clamp(-wz * wz_to_stick_scale_, -1.0, 1.0);

        if (!PublishLcm(lcmt)) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Failed to publish velocity command on LCM channel %s",
                lcm_channel_.c_str());
        }
    }

    lcm::LCM                                                   lc;
    std::mutex                                                 planner_vel_mutex_;
    std::chrono::steady_clock::time_point                      last_command_time_;
    std::chrono::milliseconds                                  cmd_timeout_{300};
    std::chrono::milliseconds                                  stand_button_hold_{500};
    double                                                     min_abs_vx_{robot_navigo::motion_limits::kExecutableMinVx};
    double                                                     min_abs_vy_{robot_navigo::motion_limits::kExecutableMinVy};
    double                                                     min_abs_wz_{robot_navigo::motion_limits::kExecutableMinWz};
    double                                                     vx_to_stick_scale_{1.0};
    double                                                     vy_to_stick_scale_{1.0};
    double                                                     wz_to_stick_scale_{1.0};
    double                                                     latest_vx_{0.0};
    double                                                     latest_vy_{0.0};
    double                                                     latest_wz_{0.0};
    std::string                                                lcm_channel_{"vel_cmd_lcm_data"};
    uint64_t                                                   lcm_type_hash_override_{0};
    bool                                                       matrix_legacy_gamepad_schema_{false};
    bool                                                       invert_lateral_axis_{false};
    int                                                        navigation_mode_{0};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr planner_vel_cmd_subscriber;
    rclcpp::TimerBase::SharedPtr                               command_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr         stand_up_service_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VelCmdLcmPublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
