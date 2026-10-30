#!/usr/bin/env python3
"""Run the actual new localization/navigation on old sensors; no actuator adapter."""
import os
from pathlib import Path
import sys
import time

import yaml

from common import (arguments, bootstrap, prepare_navigation, runtime_manifest,
                    Processes, start_stack, start_recording, audit_bag, write_json, sha256,
                    isolate_recovery_seed)
from replay_support import (validate_playback_options, player_command, full_schedule,
                            source_evidence, finish_original_source, coverage_report,
                            raw_sensor_qos, localization_command)


def write_raw_playback_qos(output, sensors):
    path = output / 'raw_playback_qos.yaml'
    path.write_text(yaml.safe_dump(raw_sensor_qos(sensors)))
    return path


def localization_environment(args):
    env = os.environ.copy()
    pangolin = Path(__file__).resolve().parents[8] / 'lightning-lm/thirdparty/Pangolin-0.9.3/build'
    # Preserve the ordinary legacy fallback; headless ARM validation must use
    # only the selected overlay's runtime dependencies.
    if not getattr(args, 'arm_runtime', False) and pangolin.is_dir():
        env['LD_LIBRARY_PATH'] = str(pangolin) + ':' + env.get('LD_LIBRARY_PATH', '')
    return env


def shutdown_navigation_once(monitor, processes, navigation_finalized=False):
    """Do not send SHUTDOWN again after the full-source tail has failed.

    finish_original_source finalizes navigation before waiting for source EOF
    and coverage. Its later failure must not repeat an already successful
    lifecycle transition (some lifecycle managers cannot handle that request).
    """
    if navigation_finalized or monitor.counts.get('lifecycle_finalized', 0):
        return False
    monitor.lifecycle_shutdown(processes)
    return True


def full_source_snapshot(monitor, player, schedule, expected, *,
                         drain_completed=False, validation_error=None):
    """Keep raw count/hash evidence even when the strict drain check fails."""
    actual = monitor.raw_source_report() if hasattr(monitor, 'raw_sources') else {}
    report = coverage_report(expected, actual)
    code = player.poll() if player is not None else None
    finalized = [row['data'] for row in monitor.events
                 if row['event'] == 'full_source_navigation_finalized']
    report.update(player_returncode=code, natural_eof=code == 0,
        original_start_ns=schedule['start_ns'], original_end_ns=schedule['end_ns'],
        last_clock_ns=monitor.get_clock().now().nanoseconds,
        navigation_finalized_before_eof=any(row.get('player_still_running') is True
                                          for row in finalized),
        timestamps_edited=False, synthetic_clock=False,
        drain_completed=drain_completed, validation_error=validation_error)
    return report


def main():
    parser = arguments(__doc__)
    parser.set_defaults(goal_input='topic')
    parser.add_argument('--record-raw-sensors', action='store_true',
                        help='Also copy original LiDAR/IMU into the output (large; disabled by default)')
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--duration', type=float, help='Source playback cap in bag seconds (short mode default: 120)')
    parser.add_argument('--observe', type=float, default=15., help='Wall seconds before controlled cancel')
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--start-offset', type=float, default=0.)
    parser.add_argument('--full-bag-mode', action='store_true',
                        help='Play every original raw sample to natural EOF; forbids duration/offset')
    parser.add_argument('--goal-at-source-time', type=float, default=0.,
                        help='Full mode: bag seconds from original metadata start before publishing goal')
    parser.add_argument('--teardown-before-eof', type=float, default=20.,
                        help='Full mode: shut navigation down while final original sensors still play')
    args = parser.parse_args()
    if not args.config or not args.map:
        parser.error('--config (Lightning YAML) and --map are required')
    try:
        validate_playback_options(args)
    except ValueError as error:
        parser.error(str(error))
    bootstrap(args)
    args.bag, args.config = args.bag.resolve(), args.config.resolve()
    if not (args.bag / 'metadata.yaml').is_file():
        raise ValueError('Expected a ROS bag directory containing metadata.yaml')
    if not (args.map.parent / 'index.txt').is_file():
        raise ValueError('Replay requires the matching tiled localization map')
    config = yaml.safe_load(args.config.read_text())
    common = config.get('common', {})
    if int(config.get('fasterlio', {}).get('lidar_type', 1)) != 1:
        raise ValueError('This historical replay fixture expects Livox CustomMsg input')
    sensors = [common.get('livox_lidar_topic', '/livox/lidar'), common.get('imu_topic', '/livox/imu')]
    args.raw_playback_qos = write_raw_playback_qos(args.output, sensors)
    args.record_topics_extra = sensors if args.record_raw_sensors else []
    metadata = yaml.safe_load((args.bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    types = {item['topic_metadata']['name']: item['topic_metadata']['type']
             for item in metadata['topics_with_message_count']}
    if types.get(sensors[0]) != 'livox_ros_driver2/msg/CustomMsg' or types.get(sensors[1]) != 'sensor_msgs/msg/Imu':
        raise ValueError('Source bag/config LiDAR and IMU topics or datatypes do not match')
    schedule = original = None
    if args.full_bag_mode:
        schedule = full_schedule(metadata, args.goal_at_source_time, args.teardown_before_eof, args.tail)
        original = source_evidence(args.bag, sensors)
        write_json(args.output / 'source_expected.json', dict(schedule=schedule, topics=original))
        # Permit cached/latched headers anywhere inside this exact source time
        # domain, rather than an arbitrary 300-second short-trial difference.
        write_json(args.output / 'source_time_domain.json', dict(
            source_window_ns=[schedule['start_ns']-2_000_000_000,
                              schedule['end_ns']+2_000_000_000],
            tolerance_seconds=2, source='original ROSbag metadata start/end',
            source_timestamps_edited=False, log_timestamps_edited=False))
    config.setdefault('system', {}).update(map_path=str(args.map.parent), with_ui=False, with_2dui=False)
    recovery = isolate_recovery_seed(config, args.output)
    effective = args.output / 'lightning_config.yaml'
    effective.write_text(yaml.safe_dump(config, sort_keys=False))
    localization_launch = localization_command(
        ['ros2', 'run', 'lightning', 'run_loc_online', '--config', str(effective),
         '--ros-args', '-p', 'use_sim_time:=true', '-p',
         'enable_debug_observations:=' + str(not args.observations_off).lower()], effective)
    write_json(args.output / 'replay_inputs.json', dict(bag=str(args.bag),
        arm_runtime=args.arm_runtime,
        metadata_sha256=sha256(args.bag / 'metadata.yaml'), sensors=sensors,
        map=str(args.map), map_sha256=sha256(args.map), source_config=str(args.config),
        source_config_sha256=sha256(args.config), effective_config_sha256=sha256(effective),
        localization_command=localization_launch,
        raw_playback_qos=str(args.raw_playback_qos),
        raw_playback_qos_sha256=sha256(args.raw_playback_qos),
        recovery_seed=recovery,
        original_sensors_copied_to_output=args.record_raw_sensors,
        rate=args.rate, source_duration=schedule['duration_seconds'] if schedule else
            (args.duration if args.duration is not None else 120.), start_offset=args.start_offset,
        full_bag_mode=args.full_bag_mode, schedule=schedule,
        arrival='N/A: historical sensors do not respond to new commands',
        excluded_source_topics='All topics except the two selected raw sensors'))
    params, _ = prepare_navigation(args)
    runtime_manifest(args, 'replay')
    import rclpy
    from observer import Monitor, yaw
    from rclpy.time import Time
    rclpy.init()
    monitor, processes = Monitor(args), Processes(args.output)
    processes.pump = lambda: rclpy.spin_once(monitor, timeout_sec=.01)
    result = dict(success=False, mode='sensor_only_open_loop', arrival='N/A', hardware_io=False,
        arm_runtime=args.arm_runtime,
        goal_input=args.goal_input, default_environment=os.environ.get('LOCNAV_PROFILE', '0') != '1',
        profiling_enabled=os.environ.get('LOCNAV_PROFILE', '0') == '1',
        composition_enabled=not args.no_composition)
    handle = action_result = player = None
    navigation_finalized = False
    try:
        others = [name for name, _ in monitor.get_node_names_and_namespaces()
                  if name != monitor.get_name()]
        if others:
            raise RuntimeError('Selected ROS domain is occupied: ' + str(others))
        if args.full_bag_mode:
            monitor.watch_raw_sources(sensors, types)
        processes.start('localization', localization_launch, env=localization_environment(args))
        player = processes.start('player', player_command(args, sensors))
        monitor.spin_until(lambda: monitor.get_clock().now().nanoseconds > 0,
            15., 'historical bag /clock before recorder startup', processes)
        start_recording(processes, args)
        monitor.recorder_ready(processes)
        start_stack(processes, args, params, scan=True)
        if args.full_bag_mode:
            if getattr(args, 'require_preplay_consumers', False):
                processes.stage('preplay_consumers', 'RUNNING')
                try:
                    monitor.preplay_consumers_ready(processes, sensors, types, schedule['start_ns'])
                except Exception as error:
                    processes.stage('preplay_consumers', 'FAIL', error=repr(error))
                    raise
                processes.stage('preplay_consumers', 'PASS')
            monitor.resume_source(sensors, processes)
        processes.stage('readiness', 'RUNNING')
        monitor.readiness(processes)
        monitor.record('provenance_audit_start', dict(sim_time_ns=monitor.get_clock().now().nanoseconds))
        processes.loaded_libraries(args.install_prefix)
        monitor.record('provenance_audit_complete', dict(sim_time_ns=monitor.get_clock().now().nanoseconds))
        processes.stage('readiness', 'PASS')
        if args.full_bag_mode:
            monitor.until_source_time(schedule['goal_ns'], args.rate, processes)
            monitor.readiness(processes)
        goal = args.goal
        if goal is None:
            transform = monitor.tf_buffer.lookup_transform('map', 'base_link', Time())
            heading = yaw(transform.transform.rotation)
            import math
            goal = [transform.transform.translation.x + math.cos(heading),
                    transform.transform.translation.y + math.sin(heading), heading]
        handle, action_result = monitor.send_goal(goal, processes)
        if args.full_bag_mode:
            monitor.until_source_time(schedule['teardown_ns'], args.rate, processes)
            monitor.record('full_source_navigation_teardown', dict(
                sim_time_ns=monitor.get_clock().now().nanoseconds,
                player_still_running=player.poll() is None,
                original_end_ns=schedule['end_ns']))
        else:
            monitor.spin_for(args.observe, processes)
        terminal = monitor.cancel(handle, action_result, processes)
        monitor.record('action_terminal', dict(status=terminal, mode='open_loop'))
        monitor.stopped_tail(args.tail, processes)
        if args.full_bag_mode:
            coverage = finish_original_source(monitor, processes, player, schedule, original, args.rate)
            navigation_finalized = True
            write_json(args.output / 'full_source_coverage.json', coverage)
            result['full_source_valid'] = coverage['valid']
            monitor.record('full_source_natural_eof', coverage)
            processes.stage('complete_original_source', 'PASS',
                duration_seconds=schedule['duration_seconds'], natural_eof=True)
        report = monitor.report()
        required = ['/odom/current_pose', '/laser_scan', '/cmd_vel_safe']
        if not args.observer_off:
            required += ['/debug/view', '/debug/build_info']
        if not args.observations_off:
            required += ['/debug/observations']
        missing = [topic for topic in required if not report['counts'].get(topic)]
        if missing:
            raise AssertionError('Required newly produced outputs missing: ' + str(missing))
        truth = report['algorithm_truth']
        if not all(truth[key] for key in ('compute_action_observed', 'smoother_action_observed',
                                         'follow_action_observed')):
            raise AssertionError('Actual planner/smoother/controller Action chain was not observed')
        if not report['build_metadata']['valid']:
            raise AssertionError('Own typed/build metadata does not contain valid actual build commits')
        if not report['view_contract']['valid']:
            raise AssertionError('Displayed checker crossed UUID/path/terminal validity boundaries')
        result.update(success=True, terminal_action_status=terminal,
            observations=report['counts'], algorithm_truth=report['algorithm_truth'])
        processes.stage('open_loop_observations', 'PASS', arrival='N/A')
    except Exception as error:
        result.update(error=type(error).__name__ + ': ' + str(error))
        processes.stage('validation', 'FAIL', error=result['error'])
        if args.full_bag_mode:
            write_json(args.output / 'full_source_coverage_failure.json',
                full_source_snapshot(monitor, player, schedule, original,
                                     validation_error=result['error']))
        if handle and action_result and not action_result.done():
            try:
                monitor.cancel(handle, action_result, processes, allow=('player',))
                monitor.spin_for(args.tail, allow=('player',))
            except Exception as cancel_error:
                result['cancel_cleanup_error'] = str(cancel_error)
        monitor.report()
    finally:
        try:
            shutdown_navigation_once(monitor, processes, navigation_finalized)
        except Exception as error:
            result.update(success=False, lifecycle_shutdown_error=str(error))
        processes.close()
        if args.full_bag_mode:
            # Cleanup pumps the monitor and can drain already queued samples.
            # A late match is evidence only: it never clears the prior failure
            # or turns a failed strict drain into full_source_valid=True.
            write_json(args.output / 'full_source_coverage.json',
                full_source_snapshot(monitor, player, schedule, original,
                    drain_completed=result.get('full_source_valid', False),
                    validation_error=result.get('error')))
        monitor.finish()
        final_report = monitor.report()
        result['observations'] = final_report['counts']
        result['algorithm_truth'] = final_report['algorithm_truth']
        result['build_metadata_valid'] = final_report['build_metadata']['valid']
        result['view_contract_valid'] = final_report['view_contract']['valid']
        rclpy.shutdown()
        schema = audit_bag(args.output)
        result['schema_valid'] = schema['success']
        required = ['/map', '/laser_scan', '/tf', '/tf_static', '/plan',
                    '/lightning/pose', '/odom/current_pose', '/cmd_vel_safe',
                    '/debug/view', '/debug/build_info']
        if args.goal_input == 'topic':
            required += ['/goal_pose']
        if not args.observations_off:
            required += ['/debug/observations']
        if args.full_bag_mode:
            required += ['/trajectories', '/transformed_global_plan']
        result['missing_delivery_topics'] = [topic for topic in required
            if not schema.get('topics', {}).get(topic, {}).get('count', 0)]
        result['delivery_valid'] = bool(schema.get('self_contained', {}).get('success')) \
            and not result['missing_delivery_topics']
        result['delivered_bag'] = schema.get('self_contained', {}).get('mcap')
        result['cleanup_valid'] = processes.cleanup_valid()
        result['success'] = (result['success'] and schema['success'] and result['delivery_valid'] and result['cleanup_valid']
                             and result['build_metadata_valid'] and result['view_contract_valid'])
        if args.full_bag_mode:
            result['success'] = result['success'] and result.get('full_source_valid', False)
        write_json(args.output / 'result.json', result)
    return 0 if result['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
