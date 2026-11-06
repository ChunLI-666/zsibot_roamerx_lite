#!/usr/bin/env python3
"""Simulation-only frame/message adaptation, raw sensor topics untouched."""
import argparse
import copy
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu, CompressedImage, CameraInfo
from livox_ros_driver2.msg import CustomMsg
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from matrix_sensor_contract import lidar_to_custom_cdr, imu_to_local
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from imu_gate import ImuMonotonicGate

class Adapter(Node):
    def __init__(self, config):
        super().__init__('matrix_sensor_contract_adapter')
        self.config=config
        qos=QoSProfile(depth=1024,reliability=ReliabilityPolicy.RELIABLE)
        self.lidar_pub=self.create_publisher(CustomMsg,'/livox/lidar',30)
        self.imu_pub=self.create_publisher(Imu,'/livox/imu',qos)
        self.imu_gate=ImuMonotonicGate(expected_frame='map')
        self.imu_reject_pub=self.create_publisher(String,'/simulation/sensor_adapter/imu_reject',qos)
        self.imu_audit_fault=None
        self.image_pub=self.create_publisher(CompressedImage,'/camera/front/image/compressed',30)
        self.info_pub=self.create_publisher(CameraInfo,'/camera/front/camera_info',30)
        self.clock_pub=self.create_publisher(Clock,'/clock',10)
        self.diag_pub=self.create_publisher(DiagnosticArray,'/simulation/sensor_adapter/diagnostics',10)
        self.counts={'lidar':0,'imu':0,'camera':0,'errors':0}
        self.ages={};self.conversion_ms=0;self.last_clock_ns=0
        self.subs=[self.create_subscription(PointCloud2,'/front_lidar',self.lidar,30),
                   self.create_subscription(Imu,'/front_lidar/imu',self.imu_raw,qos,raw=True),
                   self.create_subscription(CompressedImage,'/front_camera/image/compressed',self.camera,30)]
        self.tf=StaticTransformBroadcaster(self)
        imu=TransformStamped();imu.header.frame_id='base_link';imu.child_frame_id='imu_link'
        imu.transform.translation.z=.1;imu.transform.rotation.w=1.
        c=config['robot']['sensors']['front_camera']
        camera=TransformStamped();camera.header.frame_id='base_link';camera.child_frame_id='front_camera_optical'
        for axis in ('x','y','z'):setattr(camera.transform.translation,axis,c['position'][axis])
        r,p,y=[math.radians(c['rotation'][k]) for k in ('roll','pitch','yaw')]
        cr,sr,cp,sp,cy,sy=math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2)
        q=[sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
        # FLU camera mount -> optical: right=-Y, down=-Z, forward=X.
        v=[-.5,.5,-.5,.5];x,y,z,w=q;a,b,c1,d=v
        qq=[w*a+x*d+y*c1-z*b,w*b-x*c1+y*d+z*a,w*c1+x*b-y*a+z*d,w*d-x*a-y*b-z*c1]
        for axis,val in zip(('x','y','z','w'),qq):setattr(camera.transform.rotation,axis,val)
        self.tf.sendTransform([imu,camera])
        self.timer=self.create_timer(1.,self.diagnostics)
        self.get_logger().info('Raw stamps preserved; snapshot lidar offsets zero; imu m/s^2; use_sim_time=false. /clock is source-IMU-derived for recording.')

    def observe(self,key,msg):
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.ages[key]=(time.time_ns()-ns)*1e-9
        self.counts[key]+=1
        return ns

    def lidar(self,msg):
        start=time.monotonic_ns()
        try:self.lidar_pub.publish(lidar_to_custom_cdr(msg));self.observe('lidar',msg)
        except ValueError as exc:
            self.counts['errors']+=1;self.get_logger().error(str(exc))
        self.conversion_ms=(time.monotonic_ns()-start)*1e-6

    def imu_raw(self,raw,info):
        # Capture actual callback receipt before decoding; never reserialize raw.
        receipt_unix_ns=time.time_ns();receipt_steady_ns=time.monotonic_ns()
        event=self.imu_gate.inspect(bytes(raw),receipt_unix_ns,receipt_steady_ns)
        if not event['accepted']:
            try:
                audit=String();audit.data=json.dumps(event,allow_nan=False,separators=(',',':'))
                self.imu_reject_pub.publish(audit)
            except Exception as exc:
                self.imu_audit_fault=str(exc);self.counts['errors']+=1
                raise RuntimeError('IMU reject audit failed; no silent continuation') from exc
            if event['fatal']:self.counts['errors']+=1
            return  # No observe(), accepted receipt, derived publish or clock update.
        self.imu(deserialize_message(bytes(raw),Imu))

    def imu(self,msg):
        self.imu_pub.publish(imu_to_local(msg));ns=self.observe('imu',msg)
        if ns-self.last_clock_ns >= 50_000_000:
            self.clock_pub.publish(Clock(clock=copy.deepcopy(msg.header.stamp)));self.last_clock_ns=ns

    def camera(self,msg):
        self.observe('camera',msg);out=copy.deepcopy(msg);out.header.frame_id='front_camera_optical'
        self.image_pub.publish(out)
        c=self.config['robot']['sensors']['front_camera'];info=CameraInfo();info.header=copy.deepcopy(out.header)
        info.width=c['width'];info.height=c['height'];info.distortion_model='plumb_bob'
        info.d=[float(c[k]) for k in ('K1','K2','P1','P2','K3')]
        info.k=[c['fx'],0.,c['cx'],0.,c['fy'],c['cy'],0.,0.,1.]
        info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        info.p=[c['fx'],0.,c['cx'],0.,0.,c['fy'],c['cy'],0.,0.,0.,1.,0.]
        self.info_pub.publish(info)

    def diagnostics(self):
        msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg()
        s=DiagnosticStatus();s.name='matrix_source_contract';s.hardware_id='simulation-only';s.level=DiagnosticStatus.OK if self.counts['errors']==0 else DiagnosticStatus.ERROR
        gate=self.imu_gate.diagnostics(time.time_ns(),time.monotonic_ns())
        if self.imu_audit_fault or not gate['accepted_input_fresh'] or self.counts['errors']:
            s.level=DiagnosticStatus.ERROR
        elif gate['rejected']:
            s.level=DiagnosticStatus.WARN
        s.message='source stamps unchanged; snapshot lidar; isolated simulation; SIM IMU acceptance gate'
        values={**self.counts,**{k+'_age_s':v for k,v in self.ages.items()},'conversion_ms':self.conversion_ms}
        values.update({'imu_gate_'+k:v for k,v in gate.items()})
        s.values=[KeyValue(key=k,value=str(v)) for k,v in values.items()];msg.status=[s];self.diag_pub.publish(msg)

def run(runtime_config):
    rclpy.init();node=Adapter(json.loads(Path(runtime_config).read_text()))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    print('ROOT_HOLD: use separately released sim_imu_adapter.py')

