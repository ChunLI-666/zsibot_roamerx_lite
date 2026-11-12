#pragma once
// Native fixture only: the newer TransformBuffer wrapper is not a Jazzy upgrade.
#include <tf2_ros/buffer.h>
namespace nav2 {
class TransformBuffer : public tf2_ros::Buffer {
public:
  using SharedPtr = std::shared_ptr<TransformBuffer>;
  using tf2_ros::Buffer::Buffer;
};
}
