#pragma once
// Explicit test-only compatibility adapter, not upstream code or a ROS distro upgrade.
#include <rclcpp_lifecycle/lifecycle_node.hpp>
namespace nav2 {
template<class T> using Publisher = rclcpp_lifecycle::LifecyclePublisher<T>;
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using WeakPtr = std::weak_ptr<LifecycleNode>;
  using SharedPtr = std::shared_ptr<LifecycleNode>;
  using rclcpp_lifecycle::LifecycleNode::LifecycleNode;
  template<class T> auto create_publisher(
    const std::string & topic, const rclcpp::QoS & qos = rclcpp::QoS(10)) {
    return rclcpp_lifecycle::LifecycleNode::create_publisher<T>(topic,qos);
  }
  template<class T> T declare_or_get_parameter(const std::string & name, T value) {
    if (!this->has_parameter(name)) { return this->declare_parameter<T>(name, value); }
    T result{};
    this->get_parameter(name, result);
    return result;
  }
};
}
