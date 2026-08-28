#!/usr/bin/env python3
"""Test-only Matrix GT adapter for the planner/controller baseline.

This node is intentionally limited to the GT baseline mode. It must never be
started by the Lightning formal runner or by a real-robot launch.
"""

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, UInt8
from tf2_ros import TransformBroadcaster


def load_map_to_ground_truth(path):
    source = Path(path).expanduser().resolve()
    document = json.loads(source.read_text(encoding='utf-8'))
    value = document.get('map_T_ground_truth')
    if not isinstance(value, dict):
        raise ValueError(f'alignment has no map_T_ground_truth object: {source}')
    result = tuple(float(value[key]) for key in ('x', 'y', 'yaw'))
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f'alignment contains non-finite planar values: {source}')
    return result


def compose_planar(transform, pose):
    tx, ty, tyaw = transform
    x, y, yaw = pose
    cosine, sine = math.cos(tyaw), math.sin(tyaw)
    return (
        tx + cosine * x - sine * y,
        ty + sine * x + cosine * y,
        math.atan2(math.sin(tyaw + yaw), math.cos(tyaw + yaw)),
    )


def publish_due(last_stamp_ns, stamp_ns, minimum_period_ns):
    if last_stamp_ns is None or stamp_ns <= last_stamp_ns:
        return True
    return stamp_ns - last_stamp_ns >= minimum_period_ns


class MatrixGroundTruthBaseline(Node):
    def __init__(self):
        super().__init__('matrix_ground_truth_baseline')
        ground_truth_topic = self.declare_parameter(
            'ground_truth_topic', '/odom/mujoco_odom').value
        pose_topic = self.declare_parameter(
            'pose_topic', '/odom/current_pose').value
        status_topic = self.declare_parameter(
            'status_topic', '/lightning/loc_status').value
        pose_valid_topic = self.declare_parameter(
            'pose_valid_topic', '/lightning/pose_valid').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        alignment_file = self.declare_parameter('alignment_file', '').value
        if not alignment_file:
            raise RuntimeError('GT baseline requires an explicit alignment_file')
        self.map_to_ground_truth = load_map_to_ground_truth(alignment_file)
        max_publish_rate_hz = float(self.declare_parameter(
            'max_publish_rate_hz', 50.0).value)
        if not math.isfinite(max_publish_rate_hz) or max_publish_rate_hz <= 0.0:
            raise RuntimeError('max_publish_rate_hz must be finite and positive')
        self.minimum_publish_period_ns = int(1.0e9 / max_publish_rate_hz)
        self.last_published_stamp_ns = None

        self.pose_publisher = self.create_publisher(Odometry, pose_topic, 10)
        self.status_publisher = self.create_publisher(UInt8, status_topic, 10)
        self.valid_publisher = self.create_publisher(Bool, pose_valid_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, ground_truth_topic, self._ground_truth_callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'GT baseline adapter: {ground_truth_topic} -> {pose_topic}, '
            f'{self.map_frame}->{self.odom_frame}->{self.base_frame}, '
            f'map_T_ground_truth={self.map_to_ground_truth}, '
            f'max_publish_rate_hz={max_publish_rate_hz}')

    def _ground_truth_callback(self, message):
        stamp_ns = (int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec))
        if not publish_due(
                self.last_published_stamp_ns, stamp_ns, self.minimum_publish_period_ns):
            return
        self.last_published_stamp_ns = stamp_ns

        pose = Odometry()
        pose.header = message.header
        pose.header.frame_id = self.odom_frame
        pose.child_frame_id = self.base_frame
        source_orientation = message.pose.pose.orientation
        source_yaw = math.atan2(
            2.0 * (source_orientation.w * source_orientation.z
                   + source_orientation.x * source_orientation.y),
            1.0 - 2.0 * (source_orientation.y * source_orientation.y
                         + source_orientation.z * source_orientation.z),
        )
        map_x, map_y, map_yaw = compose_planar(
            self.map_to_ground_truth,
            (message.pose.pose.position.x, message.pose.pose.position.y, source_yaw),
        )
        pose.pose.pose.position.x = map_x
        pose.pose.pose.position.y = map_y
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.z = math.sin(0.5 * map_yaw)
        pose.pose.pose.orientation.w = math.cos(0.5 * map_yaw)
        pose.pose.covariance = message.pose.covariance
        pose.twist = message.twist
        self.pose_publisher.publish(pose)

        status = UInt8()
        status.data = 1
        self.status_publisher.publish(status)
        valid = Bool()
        valid.data = True
        self.valid_publisher.publish(valid)

        identity = TransformStamped()
        identity.header.stamp = message.header.stamp
        identity.header.frame_id = self.map_frame
        identity.child_frame_id = self.odom_frame
        identity.transform.rotation.w = 1.0

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = message.header.stamp
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        odom_to_base.transform.translation.x = map_x
        odom_to_base.transform.translation.y = map_y
        odom_to_base.transform.translation.z = 0.0
        odom_to_base.transform.rotation = pose.pose.pose.orientation
        self.tf_broadcaster.sendTransform([identity, odom_to_base])


def main(args=None):
    rclpy.init(args=args)
    node = MatrixGroundTruthBaseline()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
