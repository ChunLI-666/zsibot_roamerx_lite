#!/usr/bin/env python3
"""Actual ordinary Nav stack drives a sensor-only SE2 plant; never hardware IO."""
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from common import (arguments, bootstrap, prepare_navigation, runtime_manifest,
                    Processes, start_stack, start_recording, audit_bag, write_json)
from world import write_open_map, error_to_goal


def main():
    parser = arguments(__doc__)
    parser.add_argument('--initial', nargs=3, type=float, default=[2., 4., 0.])
    parser.add_argument('--scenario', choices=['all', 'arrive', 'cancel', 'tf_loss', 'scan_loss'], default='all')
    parser.add_argument('--fault-duration', type=float, default=5.)
    args = parser.parse_args()
    args.closed_loop = True
    bootstrap(args)
    if args.map is None:
        args.map = write_open_map(args.output / 'test_map')
    if args.goal is None:
        args.goal = [args.initial[0] + 1.2, args.initial[1], args.initial[2]]
    if args.scenario == 'all':
        rows = []
        for index, scenario in enumerate(('arrive', 'cancel', 'tf_loss', 'scan_loss')):
            command = ['/usr/bin/python3', str(Path(__file__).resolve()),
                '--install-prefix', str(args.install_prefix), '--output', str(args.output / scenario),
                '--map', str(args.map), '--nav-params', str(args.nav_params.resolve()),
                '--scenario', scenario, '--initial', *map(str, args.initial), '--goal', *map(str, args.goal),
                '--domain', str(100 + (args.domain - 100 + index + 1) % 121),
                '--ready-timeout', str(args.ready_timeout), '--timeout', str(args.timeout),
                '--tail', str(args.tail), '--fault-duration', str(args.fault_duration)]
            command += ['--goal-input', args.goal_input]
            if args.config:
                command += ['--config', str(args.config.resolve())]
            if args.no_composition:
                command += ['--no-composition']
            if args.component_executor_threads is not None:
                command += ['--component-executor-threads', str(args.component_executor_threads)]
            if args.observer_off:
                command += ['--observer-off']
            if args.observations_off:
                command += ['--observations-off']
            if args.cleanup_diagnostics:
                command += ['--cleanup-diagnostics']
            with (args.output / (scenario + '.log')).open('w') as log:
                try:
                    # Inner stages have bounded waits and own their ROS process
                    # groups. A parent subprocess timeout would SIGKILL the
                    # runner before it could clean those groups.
                    code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT).returncode
                    import json
                    payload = json.loads((args.output / scenario / 'result.json').read_text())
                except Exception as error:
                    code, payload = 1, dict(success=False, error=str(error))
            rows.append(dict(scenario=scenario, returncode=code, result=payload))
            write_json(args.output / 'result.json', dict(success=all(row['result']['success'] for row in rows),
                cases=rows, hardware_io=False, model='SE2 planar kinematic plant; no SDK/dynamics validation'))
        return 0 if all(row['result']['success'] for row in rows) else 1
    params, profile = prepare_navigation(args)
    runtime_manifest(args, 'closed_loop')
    import rclpy
    from observer import Monitor
    rclpy.init()
    monitor, processes = Monitor(args, plant=True), Processes(args.output)
    processes.pump = lambda: rclpy.spin_once(monitor, timeout_sec=.01)
    result = dict(success=False, scenario=args.scenario, hardware_io=False,
        default_environment=os.environ.get('LOCNAV_PROFILE', '0') != '1',
        profiling_enabled=os.environ.get('LOCNAV_PROFILE', '0') == '1',
        composition_enabled=not args.no_composition,
        cleanup_diagnostics=bool(args.cleanup_diagnostics),
        plant='SE2 constant body twist; 350ms command watchdog; scan is raycast map geometry',
        tested='actual ordinary planner/smoother/BT/controller/velocity_smoother/gate',
        not_tested=['Lightning algorithm', 'real physics', 'SDK/mux', 'Matrix'])
    handle = action_result = None
    try:
        others = [name for name, _ in monitor.get_node_names_and_namespaces()
                  if name != monitor.get_name()]
        if others:
            raise RuntimeError('Selected ROS domain is occupied: ' + str(others))
        monitor.spin_for(.25, processes)
        start_recording(processes, args)
        monitor.recorder_ready(processes)
        start_stack(processes, args, params)
        monitor.readiness(processes)
        processes.loaded_libraries(args.install_prefix)
        processes.stage('readiness', 'PASS')
        handle, action_result = monitor.send_goal(args.goal, processes)
        monitor.spin_until(lambda: max(map(abs, monitor.safe)) > 1e-5, 30.,
                           'actual nonzero output from safe command chain', processes)
        if args.scenario == 'arrive':
            monitor.spin_until(action_result.done, args.timeout, 'actual successful navigation result', processes)
            terminal = action_result.result().status
            if terminal != 4:
                raise AssertionError('Actual navigation did not succeed: status=' + str(terminal))
        elif args.scenario == 'cancel':
            monitor.spin_for(1., processes)
            terminal = monitor.cancel(handle, action_result, processes)
            if terminal != 5:
                raise AssertionError('Controlled cancellation did not return CANCELED: ' + str(terminal))
        else:
            monitor.spin_for(1., processes)
            boundary = time.monotonic_ns()
            if args.scenario == 'tf_loss':
                monitor.publish_tf = False
            else:
                monitor.publish_scan = False
            monitor.record('sensor_fault_injected', dict(kind=args.scenario,
                duration=args.fault_duration, navigation_logic_unchanged=True))
            monitor.spin_for(args.fault_duration, processes)
            observations = [row['data'] for row in monitor.events if row['event'] == '/debug/observations'
                            and row['monotonic_ns'] >= boundary]
            evidence = [sample for row in observations for sample in row.get('branch', [])
                        if (args.scenario == 'scan_loss' and
                            (sample.get('branch') in ('stale', 'timeout') or sample.get('site') == 'costmap_wait'))
                        or (args.scenario == 'tf_loss' and
                            (sample.get('branch') in ('pose_unavailable', 'goal_transform_failed')
                             or 'transform' in sample.get('original_error', '').lower()))]
            result['fault_evidence'] = evidence
            if not args.observations_off and not evidence:
                raise AssertionError('No direct existing branch observation for injected ' + args.scenario)
            monitor.publish_tf = monitor.publish_scan = True
            terminal = monitor.cancel(handle, action_result, processes)
            if terminal not in (5, 6):
                raise AssertionError('Fault trial expected canceled/aborted, got ' + str(terminal))
        monitor.record('action_terminal', dict(status=terminal))
        monitor.stopped_tail(args.tail, processes)
        geometry = error_to_goal(monitor.pose, args.goal)
        checker = profile['controller_server']['ros__parameters']
        plugin_names = checker['goal_checker_plugins']
        if len(plugin_names) != 1:
            raise ValueError('Independent arrival oracle requires one explicit goal checker')
        selected = checker[plugin_names[0]]
        if 'SimpleGoalChecker' not in selected['plugin']:
            raise ValueError('This oracle is for the approved SimpleGoalChecker profile')
        tolerances = dict(xy_m=float(selected['xy_goal_tolerance']), yaw_rad=float(selected['yaw_goal_tolerance']))
        geometry['within_tolerances'] = geometry['xy_m'] <= tolerances['xy_m'] and geometry['yaw_rad'] <= tolerances['yaw_rad']
        geometry['tolerances'] = tolerances
        geometry['source'] = 'independent plant state after settling; not controller debug estimate'
        result['geometry'] = geometry
        write_json(args.output / 'geometry_report.json', geometry)
        report = monitor.report()
        truth = report['algorithm_truth']
        if not all(truth[key] for key in ('compute_action_observed', 'smoother_action_observed', 'follow_action_observed')):
            raise AssertionError('Actual planner/smoother/controller Action chain was not observed')
        if not report['nonzero_safe_commands']:
            raise AssertionError('No actual nonzero safe commands')
        if args.scenario == 'arrive':
            if not geometry['within_tolerances']:
                raise AssertionError('Action claimed success outside independent goal tolerances')
            if math.hypot(monitor.pose[0] - monitor.initial[0], monitor.pose[1] - monitor.initial[1]) < .5:
                raise AssertionError('Arrival must involve real modeled motion')
            if not args.observations_off and not truth['raw_true_checker_returns']:
                raise AssertionError('No real true checker return was recorded')
        if not args.observer_off:
            missing = [topic for topic in ('/debug/view', '/debug/build_info')
                       if not report['counts'].get(topic)]
            if missing:
                raise AssertionError('Required new debug outputs missing: ' + str(missing))
        if not args.observations_off and not report['counts'].get('/debug/observations'):
            raise AssertionError('No source observations were received')
        if args.observations_off and report['counts'].get('/debug/observations'):
            raise AssertionError('Source observations disabled but records were still published')
        if not report['build_metadata']['valid']:
            raise AssertionError('Own typed/build metadata lacks valid actual build commits')
        if not report['view_contract']['valid']:
            raise AssertionError('Displayed checker crossed UUID/path/terminal validity boundaries')
        result.update(success=True, terminal_action_status=terminal, algorithm_truth=truth)
        processes.stage('closed_loop', 'PASS', scenario=args.scenario)
    except Exception as error:
        result['error'] = type(error).__name__ + ': ' + str(error)
        processes.stage('validation', 'FAIL', error=result['error'])
        if handle and action_result and not action_result.done():
            try:
                monitor.publish_tf = monitor.publish_scan = True
                monitor.cancel(handle, action_result, processes)
                monitor.spin_for(args.tail)
            except Exception as cancel_error:
                result['cancel_cleanup_error'] = str(cancel_error)
        monitor.report()
    finally:
        try:
            monitor.lifecycle_shutdown(processes)
        except Exception as error:
            result.update(success=False, lifecycle_shutdown_error=str(error))
        processes.close()
        monitor.finish()
        final_report = monitor.report()
        result['observations'] = final_report['counts']
        result['algorithm_truth'] = final_report['algorithm_truth']
        result['build_metadata_valid'] = final_report['build_metadata']['valid']
        result['view_contract_valid'] = final_report['view_contract']['valid']
        rclpy.shutdown()
        schema = audit_bag(args.output)
        result['schema_valid'] = schema['success']
        result['delivered_bag'] = schema.get('self_contained', {}).get('mcap')
        result['cleanup_valid'] = processes.cleanup_valid()
        result['success'] = (result['success'] and schema['success'] and result['cleanup_valid']
                             and result['build_metadata_valid'] and result['view_contract_valid'])
        write_json(args.output / 'result.json', result)
    return 0 if result['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
