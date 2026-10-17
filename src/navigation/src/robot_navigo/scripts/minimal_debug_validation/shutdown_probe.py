#!/usr/bin/env python3
"""Diagnostic-only native composition startup/shutdown; no navigation goals."""
import sys
import json
import yaml

from common import (arguments, bootstrap, prepare_navigation, runtime_manifest,
                    Processes, write_json, sha256, start_stack)
from world import write_open_map


def main():
    parser = arguments(__doc__)
    parser.add_argument('--mode', choices=['cpp-off', 'normal'], default='cpp-off')
    parser.add_argument('--direct-wrapper-sigint', action='store_true',
        help='Actual default wrapper active -> SIGINT; no prior lifecycle shutdown service')
    args = parser.parse_args()
    if args.direct_wrapper_sigint:
        args.mode = 'normal'
    args.closed_loop, args.observer_off = True, True
    args.observations_off = args.mode == 'cpp-off'
    bootstrap(args)
    if args.map is None:
        args.map = write_open_map(args.output / 'test_map')
    params, data = prepare_navigation(args)
    if args.mode == 'cpp-off':
        for name in ('map_server', 'planner_server', 'controller_server',
                     'behavior_server', 'bt_navigator', 'velocity_smoother'):
            data[name]['ros__parameters']['enable_debug_observations'] = False
            data[name]['ros__parameters']['enable_debug_build_info'] = False
        params.write_text(yaml.safe_dump(data, sort_keys=False))
        profile = json.loads((args.output / 'profile.json').read_text())
        profile['effective_sha256'] = sha256(params)
        profile['modifications'].append('diagnostic-only C++ metadata and observations disabled')
        write_json(args.output / 'profile.json', profile)
    runtime_manifest(args, 'closed_loop')
    import rclpy
    from observer import Monitor
    rclpy.init()
    monitor, processes = Monitor(args, plant=True), Processes(args.output)
    processes.pump = lambda: rclpy.spin_once(monitor, timeout_sec=.01)
    result = dict(success=False, scope='DIAGNOSTIC ONLY: not Debug acceptance',
        mode=args.mode, goals_sent=0, hardware_io=False, python_observer=False,
        python_gate=False, recorder=False,
        configuration='ordinary production navigation parameters; synthetic SE2 sensors only')
    result['direct_wrapper_sigint'] = args.direct_wrapper_sigint
    result['python_gate'] = args.direct_wrapper_sigint
    try:
        if args.direct_wrapper_sigint:
            start_stack(processes, args, params, scan=False)
        else:
            processes.start('navigation', ['ros2', 'launch', 'robot_navigo', 'bringup_launch.py',
                'map:=' + str(args.map), 'params_file:=' + str(params), 'use_sim_time:=true',
                'autostart:=true', 'use_composition:=' + str(not args.no_composition)])
        monitor.readiness(processes)
        processes.loaded_libraries(args.install_prefix)
        monitor.spin_for(1., processes)
        graph = {}
        for topic in ('/debug/observations', '/debug/build_info'):
            graph[topic] = [dict(node=entry.node_name, namespace=entry.node_namespace)
                           for entry in monitor.get_publishers_info_by_topic(topic)]
        write_json(args.output / 'debug_publisher_graph.json', graph)
        result['debug_publishers'] = graph
        if args.mode == 'cpp-off' and any(graph.values()):
            raise AssertionError('C++ metadata/observations disabled but publishers still exist')
        if args.mode == 'normal' and not graph['/debug/build_info']:
            raise AssertionError('Normal probe did not create the expected metadata publishers')
        if args.direct_wrapper_sigint:
            result.update(success=True, lifecycle_shutdown_service_called=False)
        else:
            monitor.lifecycle_shutdown(processes)
            result.update(success=True, lifecycle_finalized=True)
    except Exception as error:
        result['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        processes.close()
        if args.direct_wrapper_sigint:
            log = (args.output / 'navigation.log').read_text(errors='replace')
            result['lifecycle_preshutdown_logged'] = 'Running Nav2 LifecycleManager rcl preshutdown' in log
            result['success'] = result['success'] and result['lifecycle_preshutdown_logged']
        monitor.finish()
        rclpy.shutdown()
        result['cleanup_valid'] = processes.cleanup_valid()
        exit_audit = processes.stages.get('navigation', {})
        result['native_failures'] = exit_audit.get('child_failures', [])
        result['normal_log_process_signal_exits'] = exit_audit.get('normal_log_process_signal_exits', [])
        result['success'] = result['success'] and result['cleanup_valid']
        write_json(args.output / 'result.json', result)
    return 0 if result['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
