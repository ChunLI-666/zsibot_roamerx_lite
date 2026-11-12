"""Subscriber-only, standard-message observer. Default import creates no ROS context."""
import hashlib
import json
import time
from readiness_validator import Validator,GRAPH,TOPICS

class Observer:
    def __init__(self,output,sensor_config,release_id):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
        from rclpy.serialization import deserialize_message,serialize_message
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu,PointCloud2,CompressedImage,CameraInfo
        from tf2_msgs.msg import TFMessage
        from rosgraph_msgs.msg import Clock
        from imu_converter import imu_to_local
        self.rclpy=rclpy;self.decode=deserialize_message
        self.validator=Validator(sensor_config,lambda raw:bytes(serialize_message(imu_to_local(deserialize_message(raw,Imu)))))
        self.stream=(output/'sample_receipts.jsonl').open('x');self.graph_stream=(output/'graph_snapshots.jsonl').open('x')
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node=Node('read_only_imu_readiness_'+release_id,enable_rosout=False,start_parameter_services=False)
        self.executor=SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        classes=[Imu,Imu,Odometry,PointCloud2,CompressedImage,CompressedImage,CameraInfo,Clock,TFMessage,TFMessage]
        reliable=QoSProfile(depth=1024,reliability=ReliabilityPolicy.RELIABLE)
        static=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions={}
        for topic,typ in zip(TOPICS,classes):
            qos=static if topic=='/tf_static' else reliable if topic in ('/front_lidar/imu','/livox/imu','/clock') else qos_profile_sensor_data
            self.subscriptions[topic]=self.node.create_subscription(typ,topic,lambda raw,info,t=topic,cls=typ:self.capture(t,cls,raw,info),qos,raw=True)
    def capture(self,topic,typ,raw,info):
        wall,steady=time.time_ns(),time.monotonic_ns();raw=bytes(raw)
        try:
            # Actual Jazzy rclpy7.1.12 returns this dict, not a MessageInfo object;
            # it exposes no per-message publisher GID. Graph endpoint GIDs are separate.
            keys={'source_timestamp','received_timestamp','publication_sequence_number','reception_sequence_number'}
            if not isinstance(info,dict) or set(info)!=keys or any(type(v) is not int and v is not None for v in info.values()):raise ValueError('unexpected actual Jazzy callback info schema')
            evidence=dict(message_info=dict(info),matched_publisher_count=self.subscriptions[topic].get_publisher_count())
            row=self.validator.sample(topic,raw,self.decode(raw,typ),wall,steady,evidence)
        except Exception as exc:
            self.validator.errors.append(str(exc))
            row=dict(topic=topic,error=str(exc),cdr_sha256=hashlib.sha256(raw).hexdigest(),cdr_bytes=len(raw),publisher_gid=None,received_unix_ns=wall,received_steady_ns=steady)
        self.stream.write(json.dumps(row,allow_nan=False)+'\n');self.stream.flush()
    def graph(self):
        result={}
        for topic in GRAPH:
            rows=[]
            for endpoint in self.node.get_publishers_info_by_topic(topic):
                rows.append(dict(node_name=endpoint.node_name,node_namespace=endpoint.node_namespace,gid=bytes(endpoint.endpoint_gid).hex(),topic_type=endpoint.topic_type))
            result[topic]=sorted(rows,key=lambda row:row['gid'])
        self.graph_stream.write(json.dumps(dict(received_unix_ns=time.time_ns(),received_steady_ns=time.monotonic_ns(),publishers=result))+'\n');self.graph_stream.flush()
        return result
    def spin(self):
        # Amortize the outer full-history/source/graph checks over a small batch.
        # Each callback still executes capture() with its own actual receipt.
        # The deadline bounds scheduling/waits between callbacks; a callback
        # itself is not preemptible, and the parent's unchanged100ms gap gate
        # still rejects an overrun. No queue/history item is discarded.
        deadline=time.monotonic_ns()+10_000_000
        callbacks=0
        while callbacks<32:
            remaining=deadline-time.monotonic_ns()
            if remaining<=0:break
            before=sum(self.validator.received.values())
            # Actual Jazzy Executor resets its cached ready iterator whenever
            # timeout kwargs change. Keep it constant and keep this Node added
            # once so high-rate IMU cannot restart selection ahead of camera.
            self.executor.spin_once(timeout_sec=0.)
            after=sum(self.validator.received.values())
            if self.validator.errors:break
            if after<=before:
                # Amortize history checks and avoid a busy idle loop. The OS
                # can oversleep; original100ms outer gap still rejects that.
                remaining=deadline-time.monotonic_ns()
                if remaining>0:time.sleep(remaining/1e9)
                break
            callbacks+=after-before
    def close(self):
        self.stream.flush();self.stream.close();self.graph_stream.flush();self.graph_stream.close()
        try:
            self.executor.remove_node(self.node)
            if not self.executor.shutdown(timeout_sec=0.):raise RuntimeError('owned read-only executor did not stop')
        finally:
            try:self.node.destroy_node()
            finally:self.rclpy.shutdown()

if __name__=='__main__':print('ROOT_HOLD: subscriber module only; no ROS context created')
