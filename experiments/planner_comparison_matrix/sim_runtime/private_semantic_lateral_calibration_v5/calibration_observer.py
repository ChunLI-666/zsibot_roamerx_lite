"""Gate-aware calibration Observer only. No parent/backend/capability edits."""
import json
import math
import time
import lateral_contract as c
from imu_gate import ImuMonotonicGate

class Observer:
    def __init__(self,output):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2,Imu,CompressedImage,CameraInfo
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);self.rclpy=rclpy;self.decode=deserialize_message
        self.node=Node('lateral_calibration_observer',enable_rosout=False,start_parameter_services=False)
        self.executor=SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        self._callback_dispatches=0
        self.latest={};self.history=[];self.errors=[];self.counts={t:0 for t in c.TOPICS};self.stop=c.StableStop();self.stream=(output/'observations.jsonl').open('x')
        self.imu_gate=ImuMonotonicGate(expected_frame='map');self.imu_reject_stream=(output/'imu_rejects.jsonl').open('x')
        types=[Odometry,PointCloud2,Imu,CompressedImage,CompressedImage,CameraInfo];self.subscriptions=[]
        for topic,typ in zip(c.TOPICS,types):
            self.subscriptions.append(self.node.create_subscription(typ,topic,
                lambda raw,info,t=topic,cls=typ:self._dispatch(t,cls,raw),qos_profile_sensor_data,raw=True))
    def _dispatch(self,topic,typ,raw):
        # Budget all callbacks, including rejected IMU; accepted counts stay unchanged.
        self._callback_dispatches+=1
        self.capture(topic,typ,raw)
    def capture(self,topic,typ,raw):
        now,steady=time.time_ns(),time.monotonic_ns()
        try:
            if topic=='/front_lidar/imu':
                event=self.imu_gate.inspect(bytes(raw),now,steady)
                if not event['accepted']:
                    self.imu_reject_stream.write(json.dumps(event,allow_nan=False)+'\n');self.imu_reject_stream.flush()
                    if event['fatal']:self.errors.append('IMU gate fault: '+event['reason'])
                    return  # Reject never updates latest valid header/receipt or freshness lease.
            message=self.decode(raw,typ);h=message.header
            stamp=int(h.stamp.sec)*1_000_000_000+int(h.stamp.nanosec)
            if topic in self.latest and stamp<=self.latest[topic]['stamp_ns']:raise c.Hold('nonadvancing original header: '+topic)
            sample=dict(topic=topic,stamp_ns=stamp,received_unix_ns=now,received_steady_ns=steady,frame=h.frame_id,
                raw_cdr_bytes=len(raw),payload_valid=True)
            if topic=='/odom':
                p=message.pose.pose;t=message.twist.twist
                sample.update(child=message.child_frame_id,position=[p.position.x,p.position.y,p.position.z],
                    quaternion=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],
                    linear=[t.linear.x,t.linear.y,t.linear.z],angular=[t.angular.x,t.angular.y,t.angular.z])
                if self.history:
                    previous=self.history[-1];dt=(stamp-previous['stamp_ns'])/1e9
                    sample['position_speed_mps']=math.dist(sample['position'],previous['position'])/dt
                self.stop.update(sample);self.history.append(sample)
            elif topic=='/front_lidar':sample['payload_valid']=message.width>0 and len(message.data)==message.row_step*message.height
            elif topic.endswith('/compressed'):sample['payload_valid']=len(message.data)>100
            elif topic.endswith('/camera_info'):sample['payload_valid']=message.width>0 and message.height>0
            else:sample['payload_valid']=c.finite([message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z,
                message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z])
            self.latest[topic]=sample;self.counts[topic]+=1;self.stream.write(json.dumps(sample)+'\n')
        except Exception as exc:self.errors.append(str(exc))
    def spin(self):
        # Dispatch/wait budget only: synchronous callbacks and OS sleep are not
        # preemptible. Original sensor, IMU, lease and stop gates remain strict.
        deadline=time.monotonic_ns()+10_000_000
        callbacks=0;empty_dispatches=0
        while callbacks<32:
            remaining=deadline-time.monotonic_ns()
            if remaining<=0:break
            before=self._callback_dispatches
            # Fixed arguments retain Jazzy's ready iterator. A private executor
            # keeps this Node added across every callback and batch.
            self.executor.spin_once(timeout_sec=0.)
            after=self._callback_dispatches
            if self.errors:break
            if after<=before:
                # A no-handler result may only be an exhausted iterator cycle.
                # Retry it; after repeated empty cycles, yield at most1ms.
                empty_dispatches+=1
                if empty_dispatches>=2:
                    remaining=deadline-time.monotonic_ns()
                    if remaining>0:time.sleep(min(.001,remaining/1e9))
                continue
            callbacks+=after-before;empty_dispatches=0
    def close(self):
        self.imu_reject_stream.flush();self.imu_reject_stream.close()
        self.stream.flush();self.stream.close()
        try:
            self.executor.remove_node(self.node)
            if not self.executor.shutdown(timeout_sec=0.):raise RuntimeError('owned calibration executor did not stop')
        finally:
            try:self.node.destroy_node()
            finally:self.rclpy.shutdown()

if __name__=='__main__':print('ROOT_HOLD: Observer module only; no ROS context created')
