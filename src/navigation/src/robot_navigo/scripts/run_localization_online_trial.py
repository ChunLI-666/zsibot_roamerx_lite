#!/usr/bin/env python3
"""Isolated localization-only ROS replay. No navigation/actuator bridge is started."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml


def digest_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def binary_hashes(binary_dir):
    binary_dir = Path(binary_dir)
    required = ['run_loc_online', 'liblightning.libs.so']
    shared = sorted(path.name for path in binary_dir.glob('*.so*') if path.is_file())
    return {name: digest_file(binary_dir / name) for name in dict.fromkeys(required + shared)}


def input_files(path):
    """Freeze all source files, including tiled maps and bag metadata."""
    path = Path(path).resolve()
    files = [path] if path.is_file() else sorted(item for item in path.rglob('*') if item.is_file())
    if not files:
        raise ValueError(f'No input files: {path}')
    return [dict(path=str(item), size_bytes=item.stat().st_size, sha256=digest_file(item)) for item in files]


def observe_source_loss(node, spin_once, duration=4.5, clock=time.monotonic):
    deadline = clock() + duration
    while clock() < deadline:
        if node.poll() is not None:
            raise RuntimeError('Localization exited during post-playback source-loss observation')
        spin_once()
    # Also cover exit during the final spin or immediately after playback.
    if node.poll() is not None:
        raise RuntimeError('Localization exited during post-playback source-loss observation')


def group_exists(process):
    process.poll()  # Reap an exited leader before checking its group.
    try:
        os.killpg(process.pid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_process_group(process, grace_periods=(10., 5., 1.)):
    """Clean only the session created by this runner, even if its leader exited."""
    sent = []
    for sig, grace in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), grace_periods):
        if not group_exists(process):
            break
        try:
            os.killpg(process.pid, sig)
            sent.append(sig.name)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + grace
        while group_exists(process) and time.monotonic() < deadline:
            time.sleep(.02)
    process.poll()
    return dict(return_code=process.returncode, signals=sent,
                group_alive_after_cleanup=group_exists(process))


def assess_trial(timed_out, pre_cleanup_codes, failure, cleanup):
    if failure:
        return False, failure
    if timed_out:
        return False, 'Playback timed out'
    if pre_cleanup_codes.get('player') != 0:
        return False, 'Player did not finish cleanly'
    if 'node' not in pre_cleanup_codes or pre_cleanup_codes['node'] is not None:
        return False, 'Localization exited before runner-requested shutdown'
    if any(item.get('error') or item.get('group_alive_after_cleanup') for item in cleanup.values()):
        return False, 'Owned process group cleanup did not finish'
    if 'node' in cleanup and 'SIGINT' not in cleanup['node'].get('signals', []):
        return False, 'Localization exited before runner-requested shutdown signal'
    if any(set(item.get('signals', [])) & {'SIGTERM', 'SIGKILL'} or
           item.get('return_code', 0) not in (0, -signal.SIGINT) for item in cleanup.values()):
        return False, 'Owned process required forced shutdown or exited abnormally during cleanup'
    return True, None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary-dir', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=185)
    parser.add_argument('--timeout', type=float, default=90.)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output directory to preserve existing evidence')
    if not 0 <= args.domain <= 232 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Invalid isolated ROS domain or timeout')
    output = args.output.resolve()
    if any(output.is_relative_to(path.resolve()) for path in (args.bag, args.map, args.binary_dir)):
        parser.error('Output must be outside source bag, map and binary directories')
    provenance = {name: input_files(path) for name, path in (
        ('source_config', args.config), ('bag', args.bag), ('map', args.map))}
    binary_dir = args.binary_dir.resolve()
    hashes = binary_hashes(binary_dir)
    output.mkdir(parents=True)
    config = yaml.safe_load(args.config.read_text())
    config['system']['map_path'] = str(args.map.resolve())
    config['system']['with_ui'] = False
    config['system']['with_2dui'] = False
    config['system']['use_fp_init'] = True
    effective_config = output / 'config.yaml'
    effective_config.write_text(yaml.safe_dump(config))
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = str(binary_dir) + ':' + env.get('LD_LIBRARY_PATH', '')
    env['ROS_LOG_DIR'] = str(output / 'ros_logs')
    # Import only after establishing the isolated domain.
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import UInt8, Bool
    rclpy.init()
    monitor = Node('localization_trial_monitor')
    started = time.monotonic()
    events = []
    monitor.create_subscription(UInt8, '/lightning/loc_status',
        lambda message: events.append(dict(t=time.monotonic() - started, topic='loc_status', value=int(message.data))), 100)
    monitor.create_subscription(Bool, '/lightning/pose_valid',
        lambda message: events.append(dict(t=time.monotonic() - started, topic='pose_valid', value=bool(message.data))), 100)
    node_command = [str(binary_dir / 'run_loc_online'), '--config=' + str(effective_config),
                    '--ros-args', '-p', 'use_sim_time:=true']
    player_command = ['ros2', 'bag', 'play', str(args.bag.resolve()), '--clock', '100',
                      '--rate', '1', '--delay', '2', '--disable-keyboard-controls']
    node = player = None
    timed_out = False
    player_started = None
    failure = None
    pending_exception = None
    node_log = (output / 'node.log').open('w')
    player_log = (output / 'player.log').open('w')
    try:
        node = subprocess.Popen(node_command, cwd=output, env=env, stdout=node_log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        # Readiness is an actual publisher discovery, not a fixed startup sleep.
        deadline = time.monotonic() + 20
        while monitor.count_publishers('/lightning/loc_status') == 0:
            rclpy.spin_once(monitor, timeout_sec=.05)
            if node.poll() is not None:
                raise RuntimeError('Localization exited before status publisher discovery')
            if time.monotonic() > deadline:
                raise RuntimeError('No localization status publisher within 20 seconds')
        player_started = time.monotonic() - started
        player = subprocess.Popen(player_command, cwd=output, env=env, stdout=player_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + args.timeout
        while player.poll() is None:
            rclpy.spin_once(monitor, timeout_sec=.05)
            if node.poll() is not None:
                raise RuntimeError('Localization exited during playback')
            if time.monotonic() > deadline:
                timed_out = True
                break
        # Observe post-playback source loss using the real production heartbeat.
        if not timed_out and player.poll() == 0:
            observe_source_loss(node, lambda: rclpy.spin_once(monitor, timeout_sec=.05))
    except BaseException as error:
        failure = f'{type(error).__name__}: {error}'
        pending_exception = error
    finally:
        pre_cleanup_codes = {name: process.poll() for name, process in [('player', player), ('node', node)]
                             if process is not None}
        cleanup = {}
        codes = {}
        for name, process in [('player', player), ('node', node)]:
            if process is None:
                continue
            # Player cleanup can take time: sample each node immediately before its own stop.
            pre_cleanup_codes[name] = process.poll()
            try:
                cleanup[name] = stop_process_group(process)
            except Exception as error:
                cleanup[name] = dict(error=f'{type(error).__name__}: {error}')
            codes[name] = process.poll()
        passed, failure = assess_trial(timed_out, pre_cleanup_codes, failure, cleanup)
        node_log.close()
        player_log.close()
        monitor.destroy_node()
        rclpy.shutdown()
        result = dict(commands=dict(node=node_command, player=player_command),
                      ros_domain=args.domain, discovery='LOCALHOST', return_codes=codes,
                      passed=passed, failure=failure,
                      pass_scope='Runner execution only; localization lifecycle recovery requires a separate analyzer',
                      pre_cleanup_return_codes=pre_cleanup_codes,
                      cleanup=cleanup, input_provenance=provenance,
                      script_sha256=digest_file(Path(__file__)),
                      timed_out=timed_out, duration_wall_sec=time.monotonic() - started,
                      player_started_wall_sec=player_started, events=events, binary_sha256=hashes,
                      config_sha256=hashlib.sha256(effective_config.read_bytes()).hexdigest(),
                      limitations=['Localization node and sensor player only; no controller or actuator bridge',
                                   'Rate-one online replay on desktop, not RK3588 performance or independent ground truth'])
        (output / 'trial.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({key: value for key, value in result.items() if key != 'events'}, indent=2))
    if pending_exception is not None:
        raise pending_exception
    if not passed:
        raise SystemExit(f'Replay failed: {failure}; inspect preserved logs')


if __name__ == '__main__':
    main()
