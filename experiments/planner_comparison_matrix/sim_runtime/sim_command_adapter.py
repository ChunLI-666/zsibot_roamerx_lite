#!/usr/bin/env python3
"""Forward gated Nav2 policy velocities to the isolated Matrix simulation.

vx/2 and wz are native policy inputs, not proof of actual SI tracking.
The measured low-speed plant response is retained in motion_step_metrics.json.
"""
import argparse,json,math,pathlib,subprocess,sys,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8,Bool,String

def relay_command(mode):
    if mode=='local':
        return [sys.executable,str(pathlib.Path(__file__).resolve().with_name('sim_udp_relay.py'))]
    if mode=='ssh':
        return ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=2','charles@192.168.1.219','python3','/mnt/data/planner_comparison_20261105/sim_udp_relay.py']
    raise ValueError('unknown relay mode')

class Adapter(Node):
    def __init__(self,args):
        super().__init__('matrix_sim_command_adapter')
        self.maximum_vx=args.max_policy_vx;self.maximum_wz=args.max_policy_wz
        self.cmd=None;self.cmd_received=0.;self.loc=0;self.loc_received=0.;self.valid=False;self.valid_received=0.;self.odom=None;self.odom_received=0.
        self.debug=self.create_publisher(String,'/simulation/command_adapter/debug',50)
        self.sent=self.create_publisher(Twist,'/simulation/command_adapter/policy_cmd',50)
        self.subs=[self.create_subscription(Twist,'/cmd_vel_safe',self.command,1),
            self.create_subscription(UInt8,'/lightning/loc_status',self.status,10),
            self.create_subscription(Bool,'/lightning/pose_valid',self.pose_valid,10),
            self.create_subscription(Odometry,'/odom/current_pose',self.odometry,10)]
        self.relay=subprocess.Popen(relay_command(args.relay_mode),stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=None,text=True,bufsize=1)
        line=self.relay.stdout.readline().strip()
        if line!='SIM_RELAY_READY 127.0.0.1:7447':raise RuntimeError('relay startup failed: '+line)
        self.timer=self.create_timer(.05,self.tick)
        self.get_logger().info('SIM-only /cmd_vel_safe -> '+args.relay_mode+' age-checked relay -> localhost7447. No SDK, no auto route.')
    def command(self,m):self.cmd=m;self.cmd_received=time.monotonic()
    def status(self,m):self.loc=m.data;self.loc_received=time.monotonic()
    def pose_valid(self,m):self.valid=m.data;self.valid_received=time.monotonic()
    def odometry(self,m):self.odom=m;self.odom_received=time.monotonic()
    def tick(self):
        now=time.monotonic();stamp_ns=time.time_ns();reason='pass';vx=0.;wz=0.;age=None
        if self.cmd is None or now-self.cmd_received>.25:reason='command_timeout'
        elif self.loc!=1 or not self.valid or now-self.loc_received>.4 or now-self.valid_received>.4:reason='localization_not_ready'
        elif self.odom is None or now-self.odom_received>.2:reason='odometry_timeout'
        else:
            age=(stamp_ns-self.odom.header.stamp.sec*10**9-self.odom.header.stamp.nanosec)*1e-9
            x,y,z=self.cmd.linear.x,self.cmd.linear.y,self.cmd.angular.z
            if age<-.05 or age>.3:reason='odometry_stamp_age'
            elif self.odom.header.frame_id!='odom' or self.odom.child_frame_id!='base_link':reason='odometry_frame'
            elif not all(math.isfinite(v) for v in [x,y,z,self.cmd.linear.z,self.cmd.angular.x,self.cmd.angular.y]):reason='nonfinite_command'
            elif any(abs(v)>1e-6 for v in [self.cmd.linear.z,self.cmd.angular.x,self.cmd.angular.y]):reason='unsupported_command_axis'
            elif x<0 or abs(y)>1e-6:reason='unverified_reverse_or_lateral'
            elif x>self.maximum_vx+1e-6 or abs(z)>self.maximum_wz+1e-6:reason='command_exceeds_configured_bounds'
            else:vx=x;wz=z
        packet=dict(issued_unix_ns=stamp_ns,forward=vx/2.,yaw=wz)
        try:self.relay.stdin.write(json.dumps(packet)+'\n');self.relay.stdin.flush()
        except (BrokenPipeError,OSError):reason='relay_unavailable';vx=wz=0.
        out=Twist();out.linear.x=vx;out.angular.z=wz;self.sent.publish(out)
        self.debug.publish(String(data=json.dumps(dict(time_ns=stamp_ns,reason=reason,odom_age=age,command_age=now-self.cmd_received,nominal_policy_vx=vx,nominal_policy_wz=wz,udp_forward=packet['forward'],udp_yaw=packet['yaw'],below_measured_forward_motion_threshold=0<vx<.4))))
    def close(self):
        try:self.relay.stdin.close();self.relay.wait(timeout=3)
        except (OSError,subprocess.TimeoutExpired):self.relay.terminate()

def main():
    p=argparse.ArgumentParser();p.add_argument('--max-policy-vx',type=float,default=.4);p.add_argument('--max-policy-wz',type=float,default=.3)
    p.add_argument('--relay-mode',choices=['ssh','local'],default='ssh',help='local requires the Matrix simulator on this host at loopback UDP7447')
    a=p.parse_args()
    if not 0<a.max_policy_vx<=.5 or not 0<a.max_policy_wz<=.5:p.error('limits exceed validated relay bounds')
    rclpy.init();node=Adapter(a)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
