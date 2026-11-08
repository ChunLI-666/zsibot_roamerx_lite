"""Actual TF2 buffer/QoS checks with a fake node: no ROS context or DDS."""
from types import SimpleNamespace
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from matrix_navigation import make_health_tf_listener
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSCompatibility, ReliabilityPolicy, qos_check_compatible


class FakeNode:
    def __init__(self): self.subscriptions={}
    def create_subscription(self,typ,topic,callback,qos,**kwargs):
        value=SimpleNamespace(callback=callback,qos=qos)
        self.subscriptions[topic]=value
        return value
    def destroy_subscription(self,subscription): return True
    def get_clock(self): return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=2_500_000_000))


def test_latest_listener_preserves_actual_transform_and_reports_original_times():
    node=FakeNode();buffer=Buffer();listener,state=make_health_tf_listener(buffer,node,True)
    transform=TransformStamped();transform.header.frame_id='odom';transform.child_frame_id='base_link'
    transform.header.stamp.sec=2;transform.header.stamp.nanosec=450_000_000
    transform.transform.rotation.w=1.;transform.transform.translation.x=1.25
    message=TFMessage(transforms=[transform])
    info=dict(source_timestamp=2_460_000_000,received_timestamp=2_470_000_000)
    node.subscriptions['/tf'].callback(message,info)
    actual=buffer.lookup_transform('odom','base_link',Time())
    assert actual.header.stamp==transform.header.stamp and actual.transform==transform.transform
    assert state['last_by_edge']['odom<-base_link']==dict(received_ros_ns=2_500_000_000,
        original_stamp_ns=2_450_000_000,DDS_source_timestamp=2_460_000_000,DDS_received_timestamp=2_470_000_000)
    qos=node.subscriptions['/tf'].qos
    assert qos.depth==1 and qos.reliability==ReliabilityPolicy.BEST_EFFORT
    # Unresolved SYSTEM_DEFAULT liveliness may warn; it must not be incompatible.
    assert qos_check_compatible(QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE),qos)[0]!=QoSCompatibility.ERROR
    assert node.subscriptions['/tf_static'].qos.depth==100
    listener.unregister()


def test_default_retains_installed_tf2_qos_and_no_observation_hook():
    node=FakeNode();listener,state=make_health_tf_listener(Buffer(),node)
    assert node.subscriptions['/tf'].qos.depth==100
    assert state['mode']=='legacy_tf2_default' and state['callbacks']==0
    listener.unregister()
