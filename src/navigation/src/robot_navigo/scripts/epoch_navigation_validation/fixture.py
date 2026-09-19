"""Sensor/plant fixture and independent wire observer for real Nav2 processes."""
import copy
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from navigo_epoch_msgs.msg import LocalizationEpoch, EpochCommand, NavigationIntent, NavExecutionState
from rosgraph_msgs.msg import Clock as ClockMessage
from robots_dog_msgs.msg import NavigoControllerDebug
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt8, Bool
from tf2_ros import TransformBroadcaster
from rclpy.action import ActionClient
from rosidl_runtime_py.convert import message_to_ordereddict

from scene import Scene, integrate


def stamp(seconds):
    value = Time(); nanos = round(seconds*1e9)
    value.sec, value.nanosec = divmod(nanos, 1_000_000_000)
    return value


def fill_pose(value, position):
    value.position.x, value.position.y = float(position[0]), float(position[1])
    value.orientation.z = math.sin(position[2]/2); value.orientation.w = math.cos(position[2]/2)


class Fixture(Node):
    def __init__(self, map_yaml, initial, output):
        super().__init__('epoch_navigation_fixture')
        self.scene = Scene(map_yaml)
        self.pose = list(initial); self.map_offset = [0.,0.,0.]
        self.safe = [0.,0.,0.]; self.last_safe_receipt = 0.
        self.sim_time = 1000.; self.pause_clock = False; self.freeze_motion=False; self.typed_pose_bias=0.; self.source_poses={}
        self.last_tick = time.monotonic(); self.started = self.last_tick
        self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.localization_session = str(uuid.uuid4()); self.map_instance = 'fixture-map-'+str(uuid.uuid4())
        self.epoch = 1; self.commits = 1; self.health = LocalizationEpoch.NORMAL
        self.heartbeat = 0; self.commit_stamp = stamp(self.sim_time)
        self.events = []; self.latest = {}; self.latest_messages = {}; self.command_history=[]; self.scan_step = 0
        self.stream = (Path(output)/'wire_events.jsonl').open('w', buffering=1)
        self.clock_pub = self.create_publisher(ClockMessage,'/clock',10)
        self.odom_pub = self.create_publisher(Odometry,'/odom/current_pose',10)
        self.scan_pub = self.create_publisher(LaserScan,'/laser_scan',10)
        self.loc_pub = self.create_publisher(LocalizationEpoch,'/lightning/localization_epoch',10)
        self.status_pub = self.create_publisher(UInt8,'/lightning/loc_status',10)
        self.valid_pub = self.create_publisher(Bool,'/lightning/pose_valid',10)
        self.proxy_control = self.create_publisher(String,'/validation/proxy_control',10)
        self.command_injector = self.create_publisher(EpochCommand,'/cmd_vel_epoch',10)
        self.create_subscription(String,'/validation/proxy_events',lambda msg:self.record('proxy_event',json.loads(msg.data)),100)
        self.tf = TransformBroadcaster(self)
        self.navigator = ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.through_navigator = ActionClient(self,NavigateThroughPoses,'/navigate_through_poses')
        self.create_subscription(Twist,'/cmd_vel_safe',self.safe_callback,100)
        for topic, cls in [('/controller_server/FollowPath/motion_mode',String),('/controller_server/debug',NavigoControllerDebug),('/cmd_vel_nav',Twist),('/cmd_vel_epoch_raw',EpochCommand),('/cmd_vel_epoch',EpochCommand),
                           ('/nav_epoch/intent',NavigationIntent),('/nav_epoch/execution',NavExecutionState),
                           ('/plan',RosPath),('/nav_safety_gate/gate_status',UInt8)]:
            self.create_subscription(cls,topic,lambda msg, topic=topic:self.observe(topic,msg),100)
        durable = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(String,'/nav_epoch/gate_session',lambda msg:self.observe('/nav_epoch/gate_session',msg),durable)
        self.timer = self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.record('fixture_start',dict(pose=self.pose,boot_id=self.boot_id,localization_session=self.localization_session,map_instance=self.map_instance))

    def record(self, event, data):
        row = dict(monotonic_ns=time.monotonic_ns(),event=event,data=data)
        self.events.append(row); self.latest[event]=row
        self.stream.write(json.dumps(row,separators=(',',':'))+'\n')

    def observe(self, topic, msg):
        self.latest_messages[topic]=copy.deepcopy(msg)
        if topic=='/cmd_vel_epoch':
            self.command_history.append((time.monotonic_ns(),copy.deepcopy(msg)))
            self.command_history=self.command_history[-512:]
        data = message_to_ordereddict(msg)
        if topic == '/controller_server/debug':
            stamp_value=msg.robot_pose.header.stamp.sec*1_000_000_000+msg.robot_pose.header.stamp.nanosec
            expected=self.source_poses.get(stamp_value)
            data=dict(header=data['header'],robot_pose=data['robot_pose'],cmd_vel=data['cmd_vel'],path_update_count=msg.path_update_count,
                      expected_source_odom=expected,source_pose_found=expected is not None)
            if expected is not None:data['source_position_error_m']=math.hypot(msg.robot_pose.pose.position.x-expected[0],msg.robot_pose.pose.position.y-expected[1])
        if topic == '/plan':
            data = dict(header=data['header'],poses=len(msg.poses),start=message_to_ordereddict(msg.poses[0]) if msg.poses else None,end=message_to_ordereddict(msg.poses[-1]) if msg.poses else None)
        self.record(topic,data)

    def safe_callback(self, msg):
        self.safe = [msg.linear.x,msg.linear.y,msg.angular.z]
        self.last_safe_receipt = time.monotonic()
        self.record('/cmd_vel_safe',dict(velocity=self.safe.copy(),pose=self.pose.copy()))

    def current_map_pose(self):
        ox,oy,oa = self.map_offset; x,y,yaw = self.pose
        return [ox+math.cos(oa)*x-math.sin(oa)*y,oy+math.sin(oa)*x+math.cos(oa)*y,yaw+oa]

    def tick(self):
        current = time.monotonic(); dt = min(current-self.last_tick,.1); self.last_tick=current
        if not self.pause_clock:
            self.sim_time += dt
            executed = self.safe if current-self.last_safe_receipt < .35 else [0.,0.,0.]
            if not self.freeze_motion:self.pose = integrate(self.pose,executed,dt)
        source_stamp = stamp(self.sim_time)
        self.source_poses[source_stamp.sec*1_000_000_000+source_stamp.nanosec]=self.pose.copy()
        clock = ClockMessage(); clock.clock = source_stamp; self.clock_pub.publish(clock)
        odom = Odometry(); odom.header.stamp=source_stamp; odom.header.frame_id='odom'; odom.child_frame_id='base_link'
        fill_pose(odom.pose.pose,self.pose)
        odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z=self.safe
        self.odom_pub.publish(odom)
        transforms=[]
        for parent,child,pose in [('map','odom',self.map_offset),('odom','base_link',self.pose)]:
            transform=TransformStamped();transform.header.stamp=source_stamp;transform.header.frame_id=parent;transform.child_frame_id=child
            transform.transform.translation.x=float(pose[0]);transform.transform.translation.y=float(pose[1])
            transform.transform.rotation.z=math.sin(pose[2]/2);transform.transform.rotation.w=math.cos(pose[2]/2)
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        self.heartbeat += 1
        loc = LocalizationEpoch();loc.header.stamp=source_stamp;loc.header.frame_id='map';loc.schema_version=1
        loc.identity.process_session_id=self.localization_session;loc.identity.map_loaded_instance=self.map_instance
        loc.identity.epoch=self.epoch;loc.identity.commits=self.commits
        loc.heartbeat_sequence=self.heartbeat;loc.lifecycle_enabled=True;loc.phase='TRACKING'
        loc.health=self.health;loc.fusion_ready=self.health==LocalizationEpoch.NORMAL;loc.output_ready=loc.fusion_ready
        loc.commit_source_stamp=self.commit_stamp;loc.base_frame_id='base_link';loc.reason='test_fixture'
        loc.output_pose.header.stamp=source_stamp;loc.output_pose.header.frame_id='map';fill_pose(loc.output_pose.pose,self.current_map_pose())
        loc.output_pose.pose.position.x+=self.typed_pose_bias
        self.loc_pub.publish(loc);self.status_pub.publish(UInt8(data=self.health));self.valid_pub.publish(Bool(data=loc.output_ready))
        self.scan_step += 1
        if self.scan_step%2==0:
            scan=LaserScan();scan.header.stamp=source_stamp;scan.header.frame_id='base_link'
            scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.angle_max=scan.angle_min+359*scan.angle_increment
            scan.range_min=.01;scan.range_max=8.;scan.scan_time=.04
            # Physical pose stays continuous through an injected localization correction.
            scan.ranges=self.scene.scan(self.pose);self.scan_pub.publish(scan)
        if self.heartbeat%5==0:
            self.record('localization_publication',message_to_ordereddict(loc))

    def change_health(self, health, recover=False):
        self.health=health
        if recover:
            self.epoch+=1;self.commits+=1;self.commit_stamp=stamp(self.sim_time)
            self.map_offset[0]+=.30
        self.record('localization_transition',dict(health=self.health,epoch=self.epoch,commits=self.commits,map_offset=self.map_offset.copy()))
        return time.monotonic_ns()

    def close(self):
        self.stream.close();self.destroy_node()
