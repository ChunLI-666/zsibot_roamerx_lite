#!/usr/bin/python3.12

"""
Lightning-LM Bridge Node - Unified sensor bridge for Nav2 integration

This node consolidates multiple bridge functionalities:
1. TF Bridge: Complements Lightning-LM's map->odom TF with odom->base_link and base_link->livox_frame
2. Odometry Publisher: Converts TF to nav_msgs/Odometry for /odom/current_pose
3. Livox Converter: Converts Livox CustomMsg to PointCloud2
4. LaserScan Generator: Converts PointCloud2 to LaserScan for 2D navigation

Author: Claude Code
"""

import math
import struct
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from livox_ros_driver2.msg import CustomMsg


def quaternion_to_euler_yaw(q: Quaternion) -> float:
    """Extract yaw angle from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert RPY angles (radians) to quaternion (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float):
    """Compute 3x3 rotation matrix from RPY (ZYX extrinsic convention).

    Returns a flat tuple (r00, r01, r02, r10, r11, r12, r20, r21, r22)
    representing R = Rz(yaw) * Ry(pitch) * Rx(roll).

    Usage: to transform a point from child frame (livox_frame) to parent
    frame (base_link), given that RPY describes the child's orientation
    in the parent frame:
        p_parent = R @ p_child + t
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return (
        cy * cp,                    cy * sp * sr - sy * cr,     cy * sp * cr + sy * sr,
        sy * cp,                    sy * sp * sr + cy * cr,     sy * sp * cr - cy * sr,
        -sp,                        cp * sr,                    cp * cr,
    )


def _fit_plane_from_points(p1, p2, p3):
    """Fit normalized plane ax + by + cz + d = 0 from three points."""
    ux, uy, uz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
    vx, vy, vz = p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]

    a = uy * vz - uz * vy
    b = uz * vx - ux * vz
    c = ux * vy - uy * vx
    norm = math.sqrt(a * a + b * b + c * c)
    if norm < 1e-6:
        return None

    a, b, c = a / norm, b / norm, c / norm
    if c < 0.0:
        a, b, c = -a, -b, -c
    d = -(a * p1[0] + b * p1[1] + c * p1[2])
    return a, b, c, d


class LightningBridge(Node):
    """
    Unified bridge node for Lightning-LM and Nav2 integration.

    Lightning-LM publishes: map -> odom (TF)
    Nav2 expects:
      - TF: map -> odom -> base_link -> livox_frame
      - Topic: /odom/current_pose (Odometry)
      - Topic: /laser_scan (LaserScan)

    This node provides all the missing pieces.
    """

    def __init__(self):
        super().__init__('lightning_bridge')

        # Declare parameters
        self.declare_parameter('enable_tf_bridge', False)
        self.declare_parameter('enable_odom_publisher', False)
        self.declare_parameter('enable_livox_converter', True)
        self.declare_parameter('enable_laserscan', True)

                # LiDAR mounting extrinsics: base_link -> livox_frame (XYZRPY)
        # Positive pitch = LiDAR tilts forward (前倾). Unit: radians.
        # Example: Mid360 mounted with 15° forward tilt => lidar_pitch = 0.2618
        self.declare_parameter('lidar_x', 0.0)      # forward offset (m)
        self.declare_parameter('lidar_y', 0.0)      # left offset (m)
        self.declare_parameter('lidar_z', 0.0)      # up offset (m)
        self.declare_parameter('lidar_roll', 0.0)   # rotation around X (rad)
        self.declare_parameter('lidar_pitch', -0.2618)  # rotation around Y (rad), -15° = forward tilt
        self.declare_parameter('lidar_yaw', 0.0)    # rotation around Z (rad)

        # LaserScan parameters
        self.declare_parameter('target_frame', 'base_link')
        # Generate a virtual 2D obstacle scan from a horizontal height band.
        # Keep this above the ground plane; otherwise ground returns become
        # the closest range in each angle bin and pollute the local costmap.
        self.declare_parameter('min_height', 0.0)
        self.declare_parameter('max_height', 0.5)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.0087)  # ~0.5 degrees
        self.declare_parameter('scan_time', 0.1)
        self.declare_parameter('range_min', 0.5)
        self.declare_parameter('range_max', 50.0)
        self.declare_parameter('use_inf', True)

        # Optional ground removal before projecting 3D Livox points to a
        # virtual 2D scan. Keep it disabled in the hot path; correct TF plus a
        # stable height band should be enough for normal navigation.
        self.declare_parameter('enable_ground_filter', False)
        self.declare_parameter('ground_filter_distance', 0.12)
        self.declare_parameter('ground_filter_max_tilt_deg', 45.0)
        self.declare_parameter('ground_filter_candidate_min_range', 0.5)
        self.declare_parameter('ground_filter_candidate_max_range', 12.0)
        self.declare_parameter('ground_filter_candidate_min_z', -2.0)
        self.declare_parameter('ground_filter_candidate_max_z', 2.0)
        self.declare_parameter('ground_filter_max_samples', 2500)
        self.declare_parameter('ground_filter_min_inliers', 120)
        self.declare_parameter('ground_filter_min_inlier_ratio', 0.12)
        self.declare_parameter('ground_filter_iterations', 48)

        # Topic parameters
        self.declare_parameter('livox_input_topic', '/livox/lidar')
        self.declare_parameter('pointcloud_output_topic', '/livox/lidar/pointcloud2')
        self.declare_parameter('laserscan_output_topic', '/laser_scan')
        self.declare_parameter('odom_output_topic', '/odom/current_pose')

        # Get parameters
        self.enable_tf_bridge = self.get_parameter('enable_tf_bridge').value
        self.enable_odom_publisher = self.get_parameter('enable_odom_publisher').value
        self.enable_livox_converter = self.get_parameter('enable_livox_converter').value
        self.enable_laserscan = self.get_parameter('enable_laserscan').value

                # LiDAR extrinsics
        self.lidar_x = self.get_parameter('lidar_x').value
        self.lidar_y = self.get_parameter('lidar_y').value
        self.lidar_z = self.get_parameter('lidar_z').value
        self.lidar_roll = self.get_parameter('lidar_roll').value
        self.lidar_pitch = self.get_parameter('lidar_pitch').value
        self.lidar_yaw = self.get_parameter('lidar_yaw').value

        # lidar_* describes the physical mounting direction. The TF / point
        # transform uses the inverse rotation so raw Livox points become
        # horizontal in base_link, matching Foxglove visualization.
        self.tf_lidar_roll = -self.lidar_roll
        self.tf_lidar_pitch = -self.lidar_pitch
        self.tf_lidar_yaw = -self.lidar_yaw

        # Precompute quaternion and rotation matrix for LiDAR->base projection.
        self.lidar_quat = euler_to_quaternion(
            self.tf_lidar_roll, self.tf_lidar_pitch, self.tf_lidar_yaw)  # (x, y, z, w)
        self.lidar_rot = rpy_to_rotation_matrix(
            self.tf_lidar_roll, self.tf_lidar_pitch, self.tf_lidar_yaw)  # flat 9-tuple

        self.target_frame = self.get_parameter('target_frame').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.scan_time = self.get_parameter('scan_time').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.use_inf = self.get_parameter('use_inf').value
        self.enable_ground_filter = self.get_parameter('enable_ground_filter').value
        self.ground_filter_distance = self.get_parameter('ground_filter_distance').value
        self.ground_filter_max_tilt_deg = self.get_parameter('ground_filter_max_tilt_deg').value
        self.ground_filter_candidate_min_range = self.get_parameter(
            'ground_filter_candidate_min_range').value
        self.ground_filter_candidate_max_range = self.get_parameter(
            'ground_filter_candidate_max_range').value
        self.ground_filter_candidate_min_z = self.get_parameter(
            'ground_filter_candidate_min_z').value
        self.ground_filter_candidate_max_z = self.get_parameter(
            'ground_filter_candidate_max_z').value
        self.ground_filter_max_samples = self.get_parameter('ground_filter_max_samples').value
        self.ground_filter_min_inliers = self.get_parameter('ground_filter_min_inliers').value
        self.ground_filter_min_inlier_ratio = self.get_parameter(
            'ground_filter_min_inlier_ratio').value
        self.ground_filter_iterations = self.get_parameter('ground_filter_iterations').value

        livox_input_topic = self.get_parameter('livox_input_topic').value
        pointcloud_output_topic = self.get_parameter('pointcloud_output_topic').value
        laserscan_output_topic = self.get_parameter('laserscan_output_topic').value
        odom_output_topic = self.get_parameter('odom_output_topic').value

        self.get_logger().info(f'use_sim_time: {self.get_parameter("use_sim_time").value}')
        self.get_logger().info(
            f'LiDAR extrinsics (base_link->livox_frame): '
            f'xyz=[{self.lidar_x:.4f}, {self.lidar_y:.4f}, {self.lidar_z:.4f}], '
            f'physical_rpy=[{math.degrees(self.lidar_roll):.2f}, {math.degrees(self.lidar_pitch):.2f}, '
            f'{math.degrees(self.lidar_yaw):.2f}] deg, '
            f'tf_rpy=[{math.degrees(self.tf_lidar_roll):.2f}, {math.degrees(self.tf_lidar_pitch):.2f}, '
            f'{math.degrees(self.tf_lidar_yaw):.2f}] deg')

        # TF2 buffer and listener for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Static TF: base_link -> livox_frame (LiDAR mounting extrinsics)
        # Published once on /tf_static, survives for the lifetime of the node
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_lidar_tf()

        # State tracking
        self.lightning_active = False
        self.transform_count = 0
        self.last_map_to_odom: Optional[TransformStamped] = None

        # Calculate number of laser scan ranges
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        # QoS profiles
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # === TF Bridge ===
        if self.enable_tf_bridge or self.enable_odom_publisher:
            self.tf_sub = self.create_subscription(
                TFMessage,
                '/tf',
                self.tf_callback,
                10
            )
            self.get_logger().info('TF Bridge: Subscribed to /tf for map->odom')

        # === Odometry Publisher ===
        if self.enable_odom_publisher:
            self.odom_pub = self.create_publisher(
                Odometry,
                odom_output_topic,
                10
            )
            self.get_logger().info(f'Odometry Publisher: Publishing to {odom_output_topic}')

        # === Livox Converter ===
        if self.enable_livox_converter:
            self.livox_sub = self.create_subscription(
                CustomMsg,
                livox_input_topic,
                self.livox_callback,
                qos_best_effort
            )
            self.pointcloud_pub = self.create_publisher(
                PointCloud2,
                pointcloud_output_topic,
                qos_best_effort
            )
            self.get_logger().info(f'Livox Converter: {livox_input_topic} -> {pointcloud_output_topic}')

        # === LaserScan Generator ===
        if self.enable_laserscan:
            # If livox converter is enabled, we convert directly from CustomMsg
            # Otherwise, subscribe to PointCloud2
            if not self.enable_livox_converter:
                self.pc2_sub = self.create_subscription(
                    PointCloud2,
                    pointcloud_output_topic,
                    self.pointcloud_callback,
                    qos_best_effort
                )
            self.laserscan_pub = self.create_publisher(
                LaserScan,
                laserscan_output_topic,
                qos_best_effort
            )
            self.get_logger().info(f'LaserScan Generator: Publishing to {laserscan_output_topic}')
            self.get_logger().info(f'  target_frame: {self.target_frame}')
            self.get_logger().info(f'  height range: [{self.min_height}, {self.max_height}]')
            self.get_logger().info(f'  angle range: [{math.degrees(self.angle_min):.1f}, {math.degrees(self.angle_max):.1f}] deg')
            self.get_logger().info(f'  range: [{self.range_min}, {self.range_max}] m')
            self.get_logger().info(
                f'  ground_filter: enabled={self.enable_ground_filter}, '
                f'distance={self.ground_filter_distance:.2f}m, '
                f'max_tilt={self.ground_filter_max_tilt_deg:.1f}deg')

        # Independent 50Hz timer for odom->base_link TF
        # Decoupled from map->odom so the TF chain never breaks
        # even when lightning-lm has momentary delays
        if self.enable_tf_bridge:
            self.create_timer(0.02, self._tf_timer_callback)  # 50 Hz
            self.get_logger().info('TF Bridge: odom->base_link publishing at 50 Hz (independent timer)')

        # Watchdog only applies to the legacy TF/Odometry bridge path.
        if self.enable_tf_bridge or self.enable_odom_publisher:
            self.create_timer(5.0, self.watchdog_callback)

        self.get_logger().info('Lightning Bridge node started')

    def _tf_timer_callback(self):
        """Publish odom->base_link at steady 50 Hz, independent of lightning-lm.

        lightning-lm outputs the LiDAR sensor pose in the gravity-aligned map frame,
        so odom inherits the physical sensor tilt (~15° pitch).
        We apply the inverse pitch here to make base_link horizontal (robot body frame).
        """
        stamp = self.get_clock().now().to_msg()

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_link'
        # Compensate sensor tilt: apply lidar_pitch to un-tilt base_link
        qx, qy, qz, qw = euler_to_quaternion(0.0, self.lidar_pitch, 0.0)
        odom_to_base.transform.rotation.x = qx
        odom_to_base.transform.rotation.y = qy
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(odom_to_base)

    def watchdog_callback(self):
        """Warn if no transforms received."""
        if not self.lightning_active:
            self.get_logger().warn(
                'No map->odom transform received yet. '
                'Check if Lightning-LM localization is running and publishing TF.',
                throttle_duration_sec=10.0
            )

    def tf_callback(self, msg: TFMessage):
        """Process incoming TF messages to find map->odom transform."""
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                if not self.lightning_active:
                    self.get_logger().info('Lightning-LM localization active, monitoring TF')
                    self.lightning_active = True

                self.last_map_to_odom = transform
                self.transform_count += 1
                if self.transform_count % 1000 == 0:
                    self.get_logger().info(f'Processed {self.transform_count} transforms')

                # Publish Odometry message
                if self.enable_odom_publisher:
                    self._publish_odometry(transform)

    def _publish_static_lidar_tf(self):
        """Publish base_link->livox_frame as a static TF (once, on /tf_static).

        This avoids conflicts with other nodes publishing on /tf and ensures
        the LiDAR extrinsics are always available in the TF tree.

        lidar_pitch describes the physical mounting direction. The published TF
        uses the inverse rotation so raw Livox points are level in base_link.
        LaserScan generation uses the same inverse rotation matrix.
        """
        qx, qy, qz, qw = euler_to_quaternion(
            self.tf_lidar_roll, self.tf_lidar_pitch, self.tf_lidar_yaw)

        base_to_livox = TransformStamped()
        base_to_livox.header.stamp = self.get_clock().now().to_msg()
        base_to_livox.header.frame_id = 'base_link'
        base_to_livox.child_frame_id = 'livox_frame'
        base_to_livox.transform.translation.x = self.lidar_x
        base_to_livox.transform.translation.y = self.lidar_y
        base_to_livox.transform.translation.z = self.lidar_z
        base_to_livox.transform.rotation.x = qx
        base_to_livox.transform.rotation.y = qy
        base_to_livox.transform.rotation.z = qz
        base_to_livox.transform.rotation.w = qw

        self.static_tf_broadcaster.sendTransform(base_to_livox)
        self.get_logger().info(
            f'Published static TF: base_link -> livox_frame '
            f'(pitch={math.degrees(self.tf_lidar_pitch):.1f} deg)')

    def _publish_odometry(self, map_to_odom: TransformStamped):
        """Convert map->odom TF to Odometry message for /odom/current_pose.

        The Odometry message represents the robot's pose in the odom frame.
        Since Lightning-LM publishes map->odom, and we assume odom->base_link is identity,
        the robot's pose in map frame equals the map->odom transform.

        For navigo, the /odom/current_pose topic should contain the robot's global pose.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = map_to_odom.header.stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # Position from map->odom transform
        odom_msg.pose.pose.position.x = map_to_odom.transform.translation.x
        odom_msg.pose.pose.position.y = map_to_odom.transform.translation.y
        odom_msg.pose.pose.position.z = map_to_odom.transform.translation.z

        # Orientation from map->odom transform
        odom_msg.pose.pose.orientation = map_to_odom.transform.rotation

        # Covariance - set moderate uncertainty
        # [x, y, z, roll, pitch, yaw]
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[14] = 0.01  # z
        odom_msg.pose.covariance[21] = 0.01  # roll
        odom_msg.pose.covariance[28] = 0.01  # pitch
        odom_msg.pose.covariance[35] = 0.01  # yaw

        # Velocity is not available from TF alone, set to zero
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

    def livox_callback(self, msg: CustomMsg):
        """Convert Livox CustomMsg to PointCloud2 and optionally LaserScan."""
        # Convert to PointCloud2
        cloud = PointCloud2()
        cloud.header = msg.header  # Preserve original timestamp and frame_id
        cloud.height = 1
        cloud.width = msg.point_num
        cloud.is_dense = False
        cloud.is_bigendian = False

        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width

        # Convert points to binary data
        data = []
        for point in msg.points:
            data.append(struct.pack('ffff', point.x, point.y, point.z, float(point.reflectivity)))

        cloud.data = b''.join(data)

        # Publish PointCloud2
        if self.enable_livox_converter:
            self.pointcloud_pub.publish(cloud)

        # Generate LaserScan directly from Livox points (more efficient)
        if self.enable_laserscan:
            self._generate_laserscan_from_livox(msg)

    def pointcloud_callback(self, msg: PointCloud2):
        """Convert PointCloud2 to LaserScan (used if livox converter is disabled)."""
        if self.enable_laserscan:
            self._generate_laserscan_from_pointcloud2(msg)

    def _generate_laserscan_from_livox(self, msg: CustomMsg):
        """Generate LaserScan directly from Livox CustomMsg points.

        This is more efficient than going through PointCloud2 since we can
        directly access the point coordinates.
        """
        # Initialize ranges with inf or max range
        if self.use_inf:
            ranges = [float('inf')] * self.num_ranges
        else:
            ranges = [self.range_max] * self.num_ranges

        points = []
        for point in msg.points:
            transformed = self._transform_livox_point(point.x, point.y, point.z)
            if transformed is None:
                continue
            points.append(transformed)

        ground_plane = self._estimate_ground_plane(points)
        if self.enable_ground_filter and ground_plane is None:
            self.get_logger().warn(
                '地面剔除: 本帧没有可靠地面模型，退回仅按高度窗生成LaserScan',
                throttle_duration_sec=2.0)

        for x, y, z, range_val in points:
            if self._is_ground_point(x, y, z, ground_plane):
                continue

            height = self._height_for_filter(x, y, z, ground_plane)
            if height < self.min_height or height > self.max_height:
                continue

            # Range filter
            if range_val < self.range_min or range_val > self.range_max:
                continue

            angle = math.atan2(y, x)

            # Angle filter
            if angle < self.angle_min or angle > self.angle_max:
                continue

            # Calculate index
            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < self.num_ranges:
                # Keep the closest point for each angle bin
                if range_val < ranges[index]:
                    ranges[index] = range_val

        # Create LaserScan message
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.target_frame  # LaserScan should be in target frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        scan.intensities = []  # Not populated for simplicity

        self.laserscan_pub.publish(scan)

    def _generate_laserscan_from_pointcloud2(self, msg: PointCloud2):
        """Generate LaserScan from PointCloud2 message."""
        # Initialize ranges
        if self.use_inf:
            ranges = [float('inf')] * self.num_ranges
        else:
            ranges = [self.range_max] * self.num_ranges

        # Parse PointCloud2 fields
        x_offset = y_offset = z_offset = -1
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        if x_offset < 0 or y_offset < 0 or z_offset < 0:
            self.get_logger().warn('PointCloud2 missing x, y, or z fields')
            return

        point_step = msg.point_step
        data = msg.data

        points = []
        for i in range(msg.width * msg.height):
            offset = i * point_step
            lx = struct.unpack_from('f', data, offset + x_offset)[0]
            ly = struct.unpack_from('f', data, offset + y_offset)[0]
            lz = struct.unpack_from('f', data, offset + z_offset)[0]

            # Skip NaN points
            if math.isnan(lx) or math.isnan(ly) or math.isnan(lz):
                continue

            transformed = self._transform_livox_point(lx, ly, lz)
            if transformed is None:
                continue
            points.append(transformed)

        ground_plane = self._estimate_ground_plane(points)
        if self.enable_ground_filter and ground_plane is None:
            self.get_logger().warn(
                '地面剔除: 本帧没有可靠地面模型，退回仅按高度窗生成LaserScan',
                throttle_duration_sec=2.0)

        for x, y, z, range_val in points:
            if self._is_ground_point(x, y, z, ground_plane):
                continue

            height = self._height_for_filter(x, y, z, ground_plane)
            if height < self.min_height or height > self.max_height:
                continue

            # Range filter
            if range_val < self.range_min or range_val > self.range_max:
                continue

            angle = math.atan2(y, x)

            # Angle filter
            if angle < self.angle_min or angle > self.angle_max:
                continue

            # Calculate index
            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < self.num_ranges:
                if range_val < ranges[index]:
                    ranges[index] = range_val

        # Create LaserScan message
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.target_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        scan.intensities = []

        self.laserscan_pub.publish(scan)

    def _transform_livox_point(self, lx: float, ly: float, lz: float):
        """Transform one point from livox_frame to the scan target frame."""
        r00, r01, r02, r10, r11, r12, r20, r21, r22 = self.lidar_rot
        x = r00 * lx + r01 * ly + r02 * lz + self.lidar_x
        y = r10 * lx + r11 * ly + r12 * lz + self.lidar_y
        z = r20 * lx + r21 * ly + r22 * lz + self.lidar_z
        range_val = math.sqrt(x * x + y * y)
        if not math.isfinite(range_val) or not math.isfinite(z):
            return None
        return x, y, z, range_val

    def _estimate_ground_plane(self, points: Sequence[Tuple[float, float, float, float]]):
        """Estimate the dominant ground plane in the transformed point cloud.

        The transform can carry pitch/height calibration error. A per-frame
        plane model removes far ground returns before the height band is applied.
        """
        if not self.enable_ground_filter:
            return None

        candidates = [
            (x, y, z)
            for x, y, z, r in points
            if self.ground_filter_candidate_min_range <= r <= self.ground_filter_candidate_max_range
            and self.ground_filter_candidate_min_z <= z <= self.ground_filter_candidate_max_z
        ]
        if len(candidates) < self.ground_filter_min_inliers:
            return None

        if len(candidates) > self.ground_filter_max_samples:
            step = max(1, len(candidates) // self.ground_filter_max_samples)
            candidates = candidates[::step][:self.ground_filter_max_samples]

        min_normal_z = math.cos(math.radians(self.ground_filter_max_tilt_deg))
        best_plane = None
        best_inliers = 0
        n = len(candidates)

        for i in range(max(1, int(self.ground_filter_iterations))):
            i1 = (17 * i + 3) % n
            i2 = (37 * i + n // 3 + 11) % n
            i3 = (61 * i + 2 * n // 3 + 19) % n
            if i1 == i2 or i1 == i3 or i2 == i3:
                continue

            plane = _fit_plane_from_points(candidates[i1], candidates[i2], candidates[i3])
            if plane is None:
                continue

            a, b, c, d = plane
            if abs(c) < min_normal_z:
                continue

            inliers = 0
            for x, y, z in candidates:
                if abs(a * x + b * y + c * z + d) <= self.ground_filter_distance:
                    inliers += 1

            if inliers > best_inliers:
                best_inliers = inliers
                best_plane = plane

        if best_plane is None:
            return None

        inlier_ratio = best_inliers / float(n)
        if (best_inliers < self.ground_filter_min_inliers or
                inlier_ratio < self.ground_filter_min_inlier_ratio):
            return None

        return best_plane

    def _is_ground_point(self, x: float, y: float, z: float, ground_plane) -> bool:
        if ground_plane is None:
            return False

        a, b, c, d = ground_plane
        signed_distance = a * x + b * y + c * z + d
        return signed_distance <= self.ground_filter_distance

    def _height_for_filter(self, x: float, y: float, z: float, ground_plane) -> float:
        """Return obstacle height used by the LaserScan height band.

        When ground fitting succeeds, height is measured relative to the fitted
        ground plane. This prevents far ground from drifting into the band when
        LiDAR pitch or mounting height is slightly wrong.
        """
        if ground_plane is None:
            return z

        a, b, c, d = ground_plane
        return a * x + b * y + c * z + d


def main(args=None):
    rclpy.init(args=args)
    node = LightningBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception in Lightning Bridge: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
