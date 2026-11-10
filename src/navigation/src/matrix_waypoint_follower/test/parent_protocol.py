#!/usr/bin/env python3
"""Bounded real parent/child DDS protocol tests. No plant, sensors or robot."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowWaypoints, NavigateToPose


def emit(kind, **values):
    print(json.dumps(dict(kind=kind, steady_ns=time.monotonic_ns(), **values)), flush=True)


def wait(future, seconds=5):
    end = time.monotonic()+seconds
    while not future.done():
        assert time.monotonic() < end, 'future deadline'
        time.sleep(.002)
    return future.result()


def until(predicate, seconds=5):
    end = time.monotonic()+seconds
    while not predicate():
        assert time.monotonic() < end, 'predicate deadline'
        time.sleep(.002)


assert os.environ.get('ROS_DOMAIN_ID') == '190'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
prefix = Path(sys.argv[1]).resolve()
out = Path(sys.argv[2]).resolve()
out.mkdir(exist_ok=False)
exe = prefix/'lib/matrix_waypoint_follower/waypoint_follower'
namespace = '/astra_b40_parent_fixture'
rclpy.init()
node = rclpy.create_node('synthetic_parent_client_and_child_server', namespace=namespace)
group = ReentrantCallbackGroup()
mode = 'success'
requests = []
canceled_children = []


def goal_callback(goal):
    requests.append(int(goal.pose.pose.position.x))
    emit('child_request', index=requests[-1], mode=mode)
    return GoalResponse.REJECT if mode == 'reject' else GoalResponse.ACCEPT


def execute(handle):
    if mode == 'hold':
        until(lambda: handle.is_cancel_requested)
        canceled_children.append(bytes(handle.goal_id.uuid).hex())
        handle.canceled()
    else:
        handle.succeed()
    return NavigateToPose.Result()


server = ActionServer(node, NavigateToPose, 'navigate_to_pose', execute,
                      goal_callback=goal_callback, cancel_callback=lambda _: CancelResponse.ACCEPT,
                      callback_group=group)
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
thread = threading.Thread(target=executor.spin)
thread.start()
environment = os.environ.copy()
environment['AMENT_PREFIX_PATH'] = str(prefix)+':/opt/ros/jazzy'
environment['LD_LIBRARY_PATH'] = str(prefix/'lib')+':/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu'
environment['ROS_LOG_DIR'] = str(out/'ros_logs')
log = (out/'follower.log').open('w')
process = subprocess.Popen([str(exe), '--ros-args', '-r', '__ns:='+namespace,
                           '-p', 'wait_at_waypoint.enabled:=false', '-p', 'bond_heartbeat_period:=0.0'],
                          env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    lifecycle = node.create_client(ChangeState, 'waypoint_follower/change_state', callback_group=group)
    assert lifecycle.wait_for_service(timeout_sec=5)
    for transition in (1, 3):
        request = ChangeState.Request()
        request.transition.id = transition
        assert wait(lifecycle.call_async(request)).success
    client = ActionClient(node, FollowWaypoints, 'follow_waypoints', callback_group=group)
    assert client.wait_for_server(timeout_sec=5)
    cancel = node.create_client(CancelGoal, 'follow_waypoints/_action/cancel_goal', callback_group=group)
    assert cancel.wait_for_service(timeout_sec=3)
    cases = []

    def send(index):
        goal = FollowWaypoints.Goal()
        goal.goal_index = index
        goal.number_of_loops = 0
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(i)
            pose.pose.orientation.w = 1.0
            goal.poses.append(pose)
        feedback = []
        handle = wait(client.send_goal_async(goal, feedback_callback=lambda m: feedback.append(m.feedback.current_waypoint)))
        assert handle.accepted
        emit('parent_accepted', index=index, uuid=bytes(handle.goal_id.uuid).hex())
        return handle, handle.get_result_async(), feedback

    for index in (2, 3):
        start = len(requests)
        handle, result_future, feedback = send(index)
        result = wait(result_future)
        assert result.status == 4 and not result.result.missed_waypoints
        assert requests[start:] == list(range(index, 5))
        assert feedback and feedback[0] == index and set(feedback) == set(range(index, 5))
        cases.append(dict(case='nonzero_success', index=index, status=result.status, requests=requests[start:], feedback=feedback))
    mode = 'reject'
    start = len(requests)
    handle, result_future, feedback = send(3)
    result = wait(result_future)
    assert result.status == 6
    assert requests[start:] == [3] and [m.index for m in result.result.missed_waypoints] == [3]
    cases.append(dict(case='new_child_rejected', status=result.status, missed_indices=[3]))
    mode = 'hold'
    start = len(requests)
    handle, result_future, feedback = send(2)
    until(lambda: len(requests) > start)
    unrelated = CancelGoal.Request()
    unrelated.goal_info.goal_id.uuid = list(bytes.fromhex('123456789abcdef0123456789abcdef0'))
    response = wait(cancel.call_async(unrelated))
    assert not response.goals_canceling and not result_future.done()
    response = wait(handle.cancel_goal_async())
    assert len(response.goals_canceling) == 1
    assert response.goals_canceling[0].goal_id == handle.goal_id
    result = wait(result_future)
    assert result.status == 5
    until(lambda: len(canceled_children) == 1)
    assert requests[start:] == [2]
    cases.append(dict(case='exact_parent_cancel', parent_uuid=bytes(handle.goal_id.uuid).hex(),
                      status=result.status, child_cancel_count=len(canceled_children), unrelated_cancel_ignored=True))
    mode = 'success'
    start = len(requests)
    handle, result_future, feedback = send(2)
    result = wait(result_future)
    assert result.status == 4 and not result.result.missed_waypoints
    assert requests[start:] == [2, 3, 4] and feedback[0] == 2
    cases.append(dict(case='resume_after_cancel', index=2, status=result.status, requests=requests[start:]))
    maps = Path('/proc')/str(process.pid)/'maps'
    mapping = maps.read_text()
    (out/'follower.maps').write_text(mapping)
    assert str(prefix/'lib/libmatrix_waypoint_follower_core.so') in mapping
    assert '/lib/libwaypoint_follower_core.so' not in mapping
    evidence = dict(passed=True, cases=cases, executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),
                    core_sha256=hashlib.sha256((prefix/'lib/libmatrix_waypoint_follower_core.so').read_bytes()).hexdigest(),
                    boundary='Synthetic real parent/child DDS; no simulator or physical acceptance')
    (out/'RESULT.json').write_text(json.dumps(evidence, indent=2)+'\n')
    emit('PARENT_PROTOCOL_PASS', cases=len(cases), core_sha256=evidence['core_sha256'])
finally:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
    log.close()
    executor.shutdown(timeout_sec=3)
    thread.join(timeout=3)
    server.destroy()
    node.destroy_node()
    rclpy.shutdown()
