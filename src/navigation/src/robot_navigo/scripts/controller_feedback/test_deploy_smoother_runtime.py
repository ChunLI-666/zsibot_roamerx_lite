#!/usr/bin/env python3
"""Isolated real smoother check; no navigation, robot bridge or SDK is started."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve();out.mkdir(exist_ok=False)
    package=Path(__file__).resolve().parents[2]
    spec=importlib.util.spec_from_file_location('profile',package/'launch/rockdog_forward_test.launch.py')
    profile=importlib.util.module_from_spec(spec);spec.loader.exec_module(profile)
    params=profile.build_profile(package/'params/navigo_params.yaml',package)
    params['velocity_smoother']['ros__parameters']['use_sim_time']=False
    (out/'params.yaml').write_text(yaml.safe_dump(params))
    os.environ.update(ROS_DOMAIN_ID='189',ROS_LOCALHOST_ONLY='1')
    env=os.environ.copy();prefix=args.prefix.resolve()
    env['LD_LIBRARY_PATH']=str(prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    env['AMENT_PREFIX_PATH']=str(prefix)+':'+env.get('AMENT_PREFIX_PATH','')
    command=[str(prefix/'lib/navigo_velocity_optimizer/velocity_smoother'),'--ros-args','--params-file',str(out/'params.yaml'),
             '-r','cmd_vel:=/audit/input','-r','cmd_vel_smoothed:=/audit/output']
    import rclpy
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.srv import ChangeState
    rclpy.init();node=rclpy.create_node('forward_smoother_audit')
    samples=[];phase='startup'
    def receive(msg):
        samples.append(dict(phase=phase,time=time.monotonic(),vx=msg.linear.x,vy=msg.linear.y,wz=msg.angular.z))
    subscription=node.create_subscription(Twist,'/audit/output',receive,10)
    publisher=node.create_publisher(Twist,'/audit/input',10)
    log=(out/'process.log').open('w')
    process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
    try:
        client=node.create_client(ChangeState,'/velocity_smoother/change_state')
        if not client.wait_for_service(timeout_sec=15):raise RuntimeError('No smoother lifecycle service')
        for transition in (1,3):
            request=ChangeState.Request();request.transition.id=transition
            future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
            if not future.done() or not future.result().success:raise RuntimeError('Lifecycle activation failed')
        for phase,vx,vy,wz in [('forward',.1,.1,0.),('reverse_rejected',-.1,-.1,0.),('rotate',0.,0.,.08),('timeout',None,0.,0.)]:
            end=time.monotonic()+1.2
            while time.monotonic()<end:
                if vx is not None:
                    msg=Twist();msg.linear.x=vx;msg.linear.y=vy;msg.angular.z=wz;publisher.publish(msg)
                rclpy.spin_once(node,timeout_sec=.05)
        assert samples and all(s['vx']>=-1e-8 and abs(s['vy'])<1e-8 for s in samples)
        assert any(s['phase']=='forward' and s['vx']>.05 for s in samples)
        assert any(s['phase']=='rotate' and s['wz']>.02 for s in samples)
        assert abs(samples[-1]['vx'])<1e-8 and abs(samples[-1]['wz'])<1e-8
        result=dict(passed=True,samples=samples,argv=command,scope='Isolated smoother, no SDK; includes injected negative input and timeout')
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(f'PASS: {len(samples)} samples; no negative vx or lateral output, timeout stopped')
    finally:
        process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown();log.close()


if __name__=='__main__':main()
