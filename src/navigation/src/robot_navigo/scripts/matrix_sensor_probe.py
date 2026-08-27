#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class MatrixSensorProbe(Node):
    def __init__(self):
        super().__init__('matrix_sensor_probe')
        self.counts = {'lidar': 0, 'imu': 0, 'ground_truth': 0}
        self.advancing_stamps = {'lidar': 0, 'imu': 0, 'ground_truth': 0}
        self.maximum_stamp_ns = {'lidar': None, 'imu': None, 'ground_truth': None}
        self.invalid_clouds = 0
        self.last_lidar_time = None
        self.max_lidar_gap = 0.0
        self.create_subscription(
            PointCloud2, '/livox/lidar', self._lidar_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu/data_raw', self._imu_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odom/mujoco_odom', self._odom_callback,
            qos_profile_sensor_data)

    def _lidar_callback(self, message):
        expected_size = message.row_step * message.height
        if message.point_step <= 0 or len(message.data) != expected_size:
            self.invalid_clouds += 1
            return
        now = time.monotonic()
        if self.last_lidar_time is not None:
            self.max_lidar_gap = max(
                self.max_lidar_gap, now - self.last_lidar_time)
        self.last_lidar_time = now
        self.counts['lidar'] += 1
        self._observe_stamp('lidar', message)

    def _imu_callback(self, message):
        self.counts['imu'] += 1
        self._observe_stamp('imu', message)

    def _odom_callback(self, message):
        self.counts['ground_truth'] += 1
        self._observe_stamp('ground_truth', message)

    def _observe_stamp(self, stream, message):
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        maximum = self.maximum_stamp_ns[stream]
        if stamp_ns > 0 and (maximum is None or stamp_ns > maximum):
            self.maximum_stamp_ns[stream] = stamp_ns
            self.advancing_stamps[stream] += 1

    def ready(self, minimum_samples):
        return (self.invalid_clouds == 0 and
                self.max_lidar_gap <= 0.5 and
                all(count >= minimum_samples for count in self.counts.values()) and
                all(count >= minimum_samples
                    for count in self.advancing_stamps.values()))

    def lidar_stalled(self):
        return (self.last_lidar_time is not None and
                time.monotonic() - self.last_lidar_time > 2.0)


def main():
    parser = argparse.ArgumentParser(
        description='Wait for structurally valid Matrix simulation sensors')
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--minimum-samples', type=int, default=10)
    args, ros_args = parser.parse_known_args()
    if args.timeout <= 0.0 or args.minimum_samples <= 0:
        parser.error('timeout and minimum-samples must be positive')

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = MatrixSensorProbe()
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.ready(args.minimum_samples):
            print(
                f'[PASS] Matrix sensors ready: {node.counts}, '
                f'advancing_stamps={node.advancing_stamps}, '
                f'max_lidar_gap={node.max_lidar_gap:.3f}s')
            node.destroy_node()
            rclpy.shutdown()
            return 0
        if node.lidar_stalled():
            break

    print(
        f'[FAIL] Matrix sensor timeout: counts={node.counts}, '
        f'advancing_stamps={node.advancing_stamps}, '
        f'invalid_clouds={node.invalid_clouds}, '
        f'max_lidar_gap={node.max_lidar_gap:.3f}s', file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
