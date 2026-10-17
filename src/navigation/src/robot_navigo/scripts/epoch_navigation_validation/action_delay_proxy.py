#!/usr/bin/env python3
"""Delay real planner/smoother results; never generate or modify navigation paths."""
import argparse
import hashlib
import json
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.convert import message_to_ordereddict

def content_hash(message):
    return hashlib.sha256(json.dumps(message_to_ordereddict(message),sort_keys=True,separators=(",",":")).encode()).hexdigest()
from rclpy.task import Future
from nav2_msgs.action import ComputePathToPose, SmoothPath
from std_msgs.msg import String


class DelayProxy(Node):
    def __init__(self, kind):
        super().__init__('validation_'+kind+'_delay_proxy')
        self.kind=kind;self.count=0;self.held=[];self.backend={};self.front_index={}
        action,backend,front=(ComputePathToPose,'/validation/real_compute_path_to_pose','/compute_path_to_pose') if kind=='planner' else (SmoothPath,'/validation/real_smooth_path','/smooth_path')
        self.callbacks=ReentrantCallbackGroup()
        self.client=ActionClient(self,action,backend,callback_group=self.callbacks)
        self.events=self.create_publisher(String,'/validation/proxy_events',10)
        self.create_subscription(String,'/validation/proxy_control',self.control,10,callback_group=self.callbacks)
        self.server=ActionServer(self,action,front,self.execute,goal_callback=lambda request:GoalResponse.ACCEPT,cancel_callback=self.cancel,callback_group=self.callbacks)
        self.ready_timer=self.create_timer(.2,lambda:self.emit('ready',dict(backend=backend,front=front)) if self.count==0 and self.client.server_is_ready() else None)

    def emit(self,event,data):
        self.events.publish(String(data=json.dumps(dict(kind=self.kind,event=event,monotonic_ns=time.monotonic_ns(),data=data))))

    def control(self,msg):
        if msg.data=='release':
            for future in self.held:
                if not future.done():future.set_result(True)
            self.emit('released',dict(held=len(self.held)))

    def cancel(self,handle):
        key=bytes(handle.goal_id.uuid);index=self.front_index.get(key)
        if index==1:
            # Deliberately inject an uncooperative cancellation response for the
            # held request; the BT must reject its late real result by lineage.
            self.emit('cancel_rejected_for_held_result',dict(request=index))
            return CancelResponse.REJECT
        if key in self.backend:self.backend[key].cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute(self,handle):
        self.count+=1;index=self.count;key=bytes(handle.goal_id.uuid);self.front_index[key]=index
        if not self.client.server_is_ready():
            handle.abort();raise RuntimeError('Real backend action was not available')
        backend=await self.client.send_goal_async(handle.request)
        if not backend.accepted:
            handle.abort();raise RuntimeError('Real backend rejected forwarded action')
        self.backend[key]=backend
        result=await backend.get_result_async()
        digest=content_hash(result.result)
        self.emit('backend_result',dict(request=index,status=result.status,result_sha256=digest,
                                      front_uuid=[int(v) for v in handle.goal_id.uuid],backend_uuid=[int(v) for v in backend.goal_id.uuid]))
        if index==1:
            wait=Future();self.held.append(wait);self.emit('held',dict(request=index,result_sha256=digest))
            await wait
        if result.status==4:handle.succeed()
        elif result.status==5:handle.canceled()
        else:handle.abort()
        self.emit('forwarded',dict(request=index,status=result.status,result_sha256=content_hash(result.result)))
        return result.result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--kind',required=True,choices=['planner','smoother']);args=parser.parse_args()
    rclpy.init(args=[]);node=DelayProxy(args.kind)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
