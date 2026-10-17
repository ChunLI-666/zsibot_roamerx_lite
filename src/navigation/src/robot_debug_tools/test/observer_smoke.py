#!/usr/bin/env python3
"""Real local DDS smoke: native UUID + typed checker + actual producer binding."""
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from robot_debug_msgs.msg import BuildInfo, BuildStamp, CheckerSample, DebugView, StatusObservation
from robot_debug_tools.observer import DebugObserver


def main():
    rclpy.init(args=['--ros-args', '-p', "action_endpoints:=['/test/follow_path']"])
    source = Node('test_action_producer')
    observer = DebugObserver()
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(observer)
    qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status_pub = source.create_publisher(GoalStatusArray, '/test/follow_path/_action/status', qos)
    meta_pub = source.create_publisher(BuildInfo, '/debug/build_info', qos)
    obs_pub = source.create_publisher(StatusObservation, '/debug/observations', 64)
    views = []
    source.create_subscription(DebugView, '/debug/view', views.append, 10)
    build = BuildStamp(repository_id='test', artifact_id='test', git_commit='a' * 40,
                       build_id='test', dirty_state=1, known=True)
    info = BuildInfo(build=build, component='test', source_instance='test-instance',
                     node_fqn='/test_action_producer', endpoints=['/test/follow_path'])
    status = GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    status.goal_info.goal_id.uuid = list(range(16))
    observation = StatusObservation(component='test', source_instance='test-instance',
        source_build=build, reporter_build=build, endpoint='/test/follow_path',
        goal_id_valid=True, goal_id=status.goal_info.goal_id, path_revision=1,
        checker=[CheckerSample(evaluated=True, result_valid=True, result=False)])
    try:
        deadline = time.monotonic() + 6
        next_send = 0
        while time.monotonic() < deadline:
            if time.monotonic() > next_send:
                meta_pub.publish(info)
                status_pub.publish(GoalStatusArray(status_list=[status]))
                obs_pub.publish(observation)
                next_send = time.monotonic() + 0.1
            executor.spin_once(timeout_sec=0.01)
            if views and views[-1].current_checker_valid:
                break
        assert views and views[-1].current_checker_valid, 'typed checker observation not delivered'
        action = views[-1].actions[0]
        # Jazzy MessageInfo lacks the publisher GID. Do not assert a build binding
        # from node names alone; each typed observation still carries its own build.
        assert not action.source_binding_valid
        assert views[-1].current_checker[0].source_build.git_commit == 'a' * 40
        assert action.label == '/test/follow_path/' + bytes(range(16)).hex()
        assert len(list(observer.subscriptions)) == 3, 'observer entity ownership broken'
        status.status = GoalStatus.STATUS_CANCELED
        status_pub.publish(GoalStatusArray(status_list=[status]))
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and views[-1].current_checker_valid:
            executor.spin_once(timeout_sec=0.01)
        assert not views[-1].current_checker_valid
        print('PASS: native status, checker, UUID, conservative unknown native source and terminal invalidation')
    finally:
        executor.remove_node(observer)
        executor.remove_node(source)
        observer.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
