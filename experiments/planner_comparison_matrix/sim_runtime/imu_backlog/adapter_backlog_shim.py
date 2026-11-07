#!/usr/bin/env python3
"""HOLD entry. Root's exact-release/PID-guarded launcher may call run()."""
import hashlib
import json
import os
import signal
from pathlib import Path
import sys
import time


FROZEN_SOURCE = Path('/home/charles/project/colcon_ws/analysis_outputs/planner_comparison_20261105/full_scene_private_sensor_bootstrap_v2/source')
FROZEN_SHA256 = {
    'imu_gate.py':'0b451821b94eb8182ff2f8df8d5e7faf359a24486455c90b629478f944023f66',
    'adapter_with_gate.py':'be6c4863a0616e2d2b861a9e31e559f0d60f4cde18fa85093042b2198a5df53c',
    'matrix_sensor_contract.py':'3159610e54ae74e50312979103cb37e6634b24a6c47dcf74a32685d130e9dc7a',
}


def make_adapter_class():
    # No ROS import at module import or the default HOLD entry.
    for name,digest in FROZEN_SHA256.items():
        if hashlib.sha256((FROZEN_SOURCE/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Frozen source SHA mismatch: '+name)
    sys.path.insert(0,str(FROZEN_SOURCE))
    import adapter_with_gate as base
    for name in ['adapter_with_gate','matrix_sensor_contract','imu_gate']:
        if Path(sys.modules[name].__file__).resolve() != (FROZEN_SOURCE/(name+'.py')).resolve():
            raise ValueError('Wrong imported frozen module: '+name)
    from imu_backlog_gate import BacklogPreservingImuGate,validate_imu_backlog_max_age_sec
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue

    class BacklogAdapter(base.Adapter):
        def __init__(self,config,imu_backlog_max_age_sec,backlog_audit_path):
            validate_imu_backlog_max_age_sec(imu_backlog_max_age_sec)
            self.backlog_audit = Path(backlog_audit_path).open('x',buffering=1)
            try:
                super().__init__(config)
                self.imu_gate = BacklogPreservingImuGate(expected_frame='map',imu_backlog_max_age_sec=imu_backlog_max_age_sec)
                self.imu_backlog_pub = self.create_publisher(String,'/simulation/sensor_adapter/imu_backlog',
                    QoSProfile(depth=1024,reliability=ReliabilityPolicy.RELIABLE))
            except BaseException:
                self.backlog_audit.close()
                raise

        def imu_raw(self,raw,info):
            # Same original wire bytes go to the gate and inherited conversion.
            receipt_unix_ns,receipt_steady_ns=time.time_ns(),time.monotonic_ns()
            event=self.imu_gate.inspect(bytes(raw),receipt_unix_ns,receipt_steady_ns)
            if not event['accepted'] or event.get('marked_backlog'):
                try:
                    audit=String();audit.data=json.dumps(event,allow_nan=False,separators=(',',':'))
                    if event.get('marked_backlog'):
                        self.backlog_audit.write(audit.data+'\n')
                        self.backlog_audit.flush()
                        self.imu_backlog_pub.publish(audit)
                    else:
                        self.imu_reject_pub.publish(audit)
                except Exception as exc:
                    self.imu_audit_fault=str(exc);self.counts['errors']+=1
                    raise RuntimeError('IMU backlog/reject audit failed; do not forward silently') from exc
            if not event['accepted']:
                if event['fatal']:self.counts['errors']+=1
                return
            self.imu(deserialize_message(bytes(raw),Imu))

        def diagnostics(self):
            # Only diagnostics severity is specialized. TF, lidar, camera,
            # IMU conversion and clock publication remain inherited methods.
            msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg()
            status=DiagnosticStatus();status.name='matrix_source_contract';status.hardware_id='simulation-only'
            gate=self.imu_gate.diagnostics(time.time_ns(),time.monotonic_ns())
            status.level=DiagnosticStatus.OK
            if self.imu_audit_fault or not gate['accepted_input_fresh'] or self.counts['errors']:
                status.level=DiagnosticStatus.ERROR
            elif gate['rejected'] or gate['marked_backlog_samples']:
                status.level=DiagnosticStatus.WARN
            status.message='source stamps unchanged; SIM IMU gate; strict timing FAIL retained after any marked backlog sample'
            values={**self.counts,**{k+'_age_s':v for k,v in self.ages.items()},'conversion_ms':self.conversion_ms}
            values.update({'imu_gate_'+k:v for k,v in gate.items()})
            status.values=[KeyValue(key=k,value=str(v)) for k,v in values.items()]
            msg.status=[status];self.diag_pub.publish(msg)

        def destroy_node(self):
            try:
                return super().destroy_node()
            finally:
                try:
                    self.backlog_audit.flush()
                    os.fsync(self.backlog_audit.fileno())
                finally:
                    self.backlog_audit.close()

    return BacklogAdapter


def run(runtime_config,*,imu_backlog_max_age_sec=.3,backlog_audit_path):
    """Called only after Root's separate release, owner/PID and domain checks."""
    if os.environ.get('ROS_DOMAIN_ID') != '185':
        raise ValueError('Only isolated SIM domain185 is supported')
    adapter=make_adapter_class()
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    stop=[]
    previous={number:signal.signal(number,lambda number,frame:stop.append(number))
              for number in (signal.SIGINT,signal.SIGTERM)}
    node=None
    initialized=False
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized=True
        if not stop:
            node=adapter(json.loads(Path(runtime_config).read_text()),imu_backlog_max_age_sec,backlog_audit_path)
            while rclpy.ok() and not stop:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        stop.append(signal.SIGINT)
    finally:
        try:
            if node is not None:node.destroy_node()
        finally:
            if initialized and rclpy.ok():rclpy.shutdown()
            for number,handler in previous.items():signal.signal(number,handler)


if __name__=='__main__':
    print(json.dumps(dict(state='ROOT_HOLD',no_ROS_init=True,no_Node=True,
        default_imu_backlog_max_age_sec=.3,maximum_imu_backlog_max_age_sec=1.,
        execution='Root exact-release and PID-guarded launcher must call run(); no direct execute entry')))
