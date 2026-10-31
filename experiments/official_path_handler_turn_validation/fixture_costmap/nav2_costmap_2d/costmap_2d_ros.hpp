#pragma once
// Test-only wrapper replacing ROS lifecycle integration, NOT a deployed Costmap2DROS.
// Uses real Jazzy Costmap2D storage and geometry. No node or DDS is constructed.
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_ros_common/tf2_factories.hpp>
namespace nav2_costmap_2d {
class Costmap2DROS {
public:
  explicit Costmap2DROS(nav2::TransformBuffer::SharedPtr tf)
  : grid_(2000,2000,0.01,-10.0,-10.0,0), tf_(std::move(tf)) {}
  Costmap2D * getCostmap() { return &grid_; }
  const std::string & getGlobalFrameID() const { return frame_; }
  const std::string & getBaseFrameID() const { return base_; }
  double getTransformTolerance() const { return 0.0; }
  nav2::TransformBuffer::SharedPtr getTfBuffer() { return tf_; }
private:
  Costmap2D grid_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::string frame_{"map"}, base_{"base_link"};
};
}
