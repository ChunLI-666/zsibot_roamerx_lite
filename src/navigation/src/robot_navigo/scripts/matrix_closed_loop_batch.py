#!/usr/bin/env python3
"""Serial, manifest-driven Matrix closed-loop regression orchestrator.

The orchestrator owns only the process group created for one case. It never
uses global process matching or global kill commands. GT and Lightning modes
are separate manifest entries and a failed Lightning case is never retried as
GT. Metrics are read from the existing E2E result JSON; this script does not
invent collision or trajectory metrics.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import fcntl
import hashlib
import html
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable, Mapping

import yaml


SCHEMA = 'robot_navigo.matrix_closed_loop_suite'
SCHEMA_VERSION = 1
FINAL_STATUSES = {'PASS', 'FAIL', 'SKIPPED', 'UNAVAILABLE', 'TIMEOUT', 'INFRA_FAIL'}
MODE_NAMES = {'gt_baseline', 'lightning_formal'}
COVERAGE_NAMES = {'central_smoke', 'full_map_loop'}
DEFAULT_TIMEOUT_SEC = 1800.0


class ManifestError(ValueError):
    pass


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        with path.open(encoding='utf-8') as stream:
            value = json.load(stream)
        return value if isinstance(value, dict) else None
    except (OSError, json.JSONDecodeError):
        return None


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def resolve_template(value: Any, context: Mapping[str, Any]) -> Any:
    if isinstance(value, str):
        try:
            return value.format_map(_MissingKey(context))
        except KeyError as error:
            raise ManifestError(f'unknown template variable: {error.args[0]}') from error
    if isinstance(value, list):
        return [resolve_template(item, context) for item in value]
    if isinstance(value, dict):
        return {key: resolve_template(item, context) for key, item in value.items()}
    return value


class _MissingKey(dict):
    def __missing__(self, key):
        raise KeyError(key)


def nested_get(value: Mapping[str, Any], path: str) -> Any:
    current: Any = value
    for key in path.split('.'):
        if not isinstance(current, Mapping) or key not in current:
            return None
        current = current[key]
    return current


def sha256_payload(payload: Any) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()
    return hashlib.sha256(encoded).hexdigest()


def resolve_variables(raw: Mapping[str, Any], extra: Mapping[str, Any] | None = None) -> dict[str, Any]:
    extra = dict(extra or {})
    resolved = dict(raw)
    for _ in range(len(raw) + 1):
        updated = resolve_template(dict(raw), {**extra, **resolved})
        if updated == resolved:
            return updated
        resolved = updated
    raise ManifestError('variables contain a cyclic or unresolved template')


def validate_route(path: Path, minimum_waypoints: int, coverage: str) -> list[str]:
    try:
        route = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        return [f'route is not readable JSON: {path}: {error}']
    failures = []
    if route.get('frame_id', 'map') != 'map':
        failures.append('route frame_id must be map')
    waypoints = route.get('waypoints')
    if not isinstance(waypoints, list) or len(waypoints) < minimum_waypoints:
        failures.append(
            f'route needs at least {minimum_waypoints} waypoints for full coverage')
    if route.get('return_to_start') is not True:
        failures.append('full-coverage route must return_to_start=true')
    if coverage == 'full_map_loop':
        metadata = route.get('metadata')
        if route.get('schema') != 'robot_navigo.map_coverage' or not isinstance(metadata, Mapping):
            failures.append('full_map_loop requires map_coverage route metadata')
        else:
            if float(metadata.get('planned_coverage_pct', 0.0)) < 99.0:
                failures.append('full_map_loop planned coverage must be at least 99%')
            if not metadata.get('map_fingerprint_sha256'):
                failures.append('full_map_loop requires a map fingerprint')
            if metadata.get('route_safety_check') != \
                    'all segments sampled inside the safe reachable domain':
                failures.append('full_map_loop requires the route segment safety check')
    for index, waypoint in enumerate(waypoints or []):
        if not isinstance(waypoint, Mapping):
            failures.append(f'route waypoint {index} is not an object')
            continue
        for key in ('x', 'y', 'yaw'):
            if key not in waypoint:
                failures.append(f'route waypoint {index} has no {key}')
    return failures


def validate_manifest(manifest: Mapping[str, Any]) -> list[str]:
    failures = []
    if manifest.get('schema') != SCHEMA:
        failures.append(f'unsupported schema: {manifest.get("schema")!r}')
    if manifest.get('schema_version') != SCHEMA_VERSION:
        failures.append(f'unsupported schema_version: {manifest.get("schema_version")!r}')
    scenes = manifest.get('scenes')
    if not isinstance(scenes, list) or not scenes:
        failures.append('scenes must be a non-empty list')
        return failures
    scene_names = set()
    for scene in scenes:
        if not isinstance(scene, Mapping):
            failures.append('scene must be an object')
            continue
        name = scene.get('name')
        if not name or name in scene_names:
            failures.append(f'duplicate or empty scene name: {name!r}')
        scene_names.add(name)
        if not isinstance(scene.get('scene_id'), int):
            failures.append(f'{name}: scene_id must be an integer')
        if scene.get('available', True) is False:
            if not scene.get('reason'):
                failures.append(f'{name}: unavailable scene needs reason')
            continue
        cases = scene.get('cases')
        if not isinstance(cases, list) or not cases:
            failures.append(f'{name}: available scene needs cases')
            continue
        case_names = set()
        for case in cases:
            if not isinstance(case, Mapping):
                failures.append(f'{name}: case must be an object')
                continue
            case_name = case.get('name')
            if not case_name or case_name in case_names:
                failures.append(f'{name}: duplicate or empty case name: {case_name!r}')
            case_names.add(case_name)
            if case.get('coverage') not in COVERAGE_NAMES:
                failures.append(
                    f'{name}/{case_name}: unsupported coverage {case.get("coverage")!r}')
            modes = case.get('modes')
            if not isinstance(modes, list) or not modes:
                failures.append(f'{name}/{case_name}: modes must be a non-empty list')
                continue
            mode_names = set()
            for mode in modes:
                if not isinstance(mode, Mapping):
                    failures.append(f'{name}/{case_name}: mode must be an object')
                    continue
                mode_name = mode.get('name')
                if mode_name not in MODE_NAMES:
                    failures.append(f'{name}/{case_name}: unsupported mode {mode_name!r}')
                if mode_name in mode_names:
                    failures.append(f'{name}/{case_name}: duplicate mode {mode_name!r}')
                mode_names.add(mode_name)
                command = mode.get('command')
                if not isinstance(command, list) or not command:
                    failures.append(f'{name}/{case_name}/{mode_name}: command is required')
    return failures


def mode_result_metrics(result: Mapping[str, Any] | None) -> dict[str, Any]:
    """Project only fields already emitted by matrix_closed_loop_e2e.py."""
    if not result:
        return {}
    acceptance = result.get('acceptance', {})
    trajectory = result.get('relative_trajectory', {})
    physical = result.get('ground_truth_physical_motion', {})
    route = result.get('route', {})
    return {
        'pass': result.get('pass'),
        'failures': result.get('failures', []),
        'action_status': nested_get(result, 'action.status'),
        'waypoints_completed': sum(
            1 for item in route.get('waypoint_results', [])
            if item.get('within_tolerance') is True),
        'waypoints_total': len(route.get('waypoint_results', [])),
        'position_rmse_m': trajectory.get('position_rmse_m'),
        'position_max_error_m': trajectory.get('position_max_error_m'),
        'yaw_rmse_rad': trajectory.get('yaw_rmse_rad'),
        'path_length_m': physical.get('path_length_m'),
        'cmd_vel_nav_hz': nested_get(result, 'topic_stats./cmd_vel_nav.frequency_hz'),
        'cmd_vel_nav_max_gap_sec': nested_get(
            result, 'topic_stats./cmd_vel_nav.max_interval_sec'),
        'localization_status_hz': nested_get(
            result, 'topic_stats./lightning/loc_status.frequency_hz'),
    }


def status_record(case: str, scene: Mapping[str, Any], mode: str, status: str,
                  reason: str, output: Path, signature: str,
                  started: str, elapsed: float, result: Mapping[str, Any] | None = None,
                  resumed: bool = False) -> dict[str, Any]:
    return {
        'schema': 'robot_navigo.matrix_closed_loop_case_status',
        'schema_version': 1,
        'scene': scene.get('name'),
        'scene_id': scene.get('scene_id'),
        'case': case,
        'mode': mode,
        'status': status,
        'reason': reason,
        'started_at': started,
        'finished_at': utc_now(),
        'elapsed_sec': round(elapsed, 3),
        'input_signature': signature,
        'result_file': str(output / 'result.json'),
        'metrics': mode_result_metrics(result),
        'contact_detection': {
            'status': 'UNAVAILABLE',
            'reason': 'Matrix batch contract has no ROS contact topic; '
                      'path/cost proxies are not collision truth',
        },
        'resumed': resumed,
    }


def terminate_process_group(process: subprocess.Popen[Any], grace_sec: float = 5.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + grace_sec
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.1)
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=grace_sec)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


def execute_case(scene: Mapping[str, Any], case: Mapping[str, Any], mode: Mapping[str, Any],
                 context: Mapping[str, Any], output: Path, timeout_sec: float,
                 resume: bool, dry_run: bool) -> dict[str, Any]:
    case_name = str(case['name'])
    mode_name = str(mode['name'])
    signature_payload = {
        'scene': scene,
        'case': case,
        'mode': mode,
        'context': {key: context[key] for key in sorted(context)},
    }
    signature = sha256_payload(signature_payload)
    status_path = output / 'case_status.json'
    previous = read_json(status_path) if resume else None
    if previous and previous.get('status') in FINAL_STATUSES and \
            previous.get('input_signature') == signature:
        reused = dict(previous)
        reused['resumed'] = True
        atomic_write_json(status_path, reused)
        return reused

    output.mkdir(parents=True, exist_ok=True)
    started = utc_now()
    running = status_record(
        case_name, scene, mode_name, 'RUNNING', 'case started', output,
        signature, started, 0.0)
    atomic_write_json(status_path, running)
    if dry_run:
        record = status_record(
            case_name, scene, mode_name, 'SKIPPED', 'dry-run', output,
            signature, started, 0.0)
        atomic_write_json(status_path, record)
        return record

    command = resolve_template(mode['command'], context)
    if not all(isinstance(item, str) for item in command):
        record = status_record(
            case_name, scene, mode_name, 'INFRA_FAIL', 'command contains non-string argv',
            output, signature, started, 0.0)
        atomic_write_json(status_path, record)
        return record
    env = os.environ.copy()
    env.update({
        'MATRIX_CLOSED_LOOP_MODE': mode_name,
        'MATRIX_BATCH_SCENE': str(scene['name']),
        'MATRIX_BATCH_CASE': case_name,
        'MATRIX_BATCH_OUTPUT': str(output),
    })
    env.update({str(key): str(value) for key, value in mode.get('environment', {}).items()})
    stdout_path = output / 'orchestrator.stdout.log'
    stderr_path = output / 'orchestrator.stderr.log'
    start_monotonic = time.monotonic()
    return_code = None
    reason = 'command completed'
    result = None
    try:
        with stdout_path.open('w', encoding='utf-8') as stdout, \
                stderr_path.open('w', encoding='utf-8') as stderr:
            process = subprocess.Popen(
                command,
                cwd=context.get('workspace') or None,
                env=env,
                stdout=stdout,
                stderr=stderr,
                start_new_session=True,
                text=True,
            )
            try:
                return_code = process.wait(timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                reason = f'timeout after {timeout_sec:.1f}s'
                terminate_process_group(process)
                status = 'TIMEOUT'
            else:
                result = read_json(output / 'result.json')
                if return_code == 0 and result and result.get('pass') is True:
                    status = 'PASS'
                elif result and result.get('pass') is False:
                    status = 'FAIL'
                    reason = 'E2E result reported failed acceptance checks'
                else:
                    status = 'INFRA_FAIL'
                    reason = f'runner exited with code {return_code}'
    except OSError as error:
        status = 'INFRA_FAIL'
        reason = f'could not start runner: {error}'
    elapsed = time.monotonic() - start_monotonic
    record = status_record(
        case_name, scene, mode_name, status, reason, output, signature,
        started, elapsed, result)
    record['command'] = command
    record['return_code'] = return_code
    atomic_write_json(status_path, record)
    return record


def scene_context(variables: Mapping[str, Any], scene: Mapping[str, Any], case: Mapping[str, Any],
                  mode: Mapping[str, Any], run_dir: Path, output: Path,
                  package_root: Path) -> dict[str, Any]:
    context = dict(variables)
    context.update({
        'scene': scene.get('name'),
        'scene_id': scene.get('scene_id'),
        'case': case.get('name'),
        'mode': mode.get('name'),
        'run_dir': str(run_dir),
        'scene_dir': str(run_dir / str(scene.get('name'))),
        'case_dir': str(output.parent),
        'case_output': str(output),
        'package_root': str(package_root),
    })
    return context


def collect_cases(manifest: Mapping[str, Any], run_dir: Path, package_root: Path,
                  scene_filter: set[str] | None, mode_filter: set[str] | None,
                  resume: bool, dry_run: bool, timeout_override: float | None) -> list[dict[str, Any]]:
    variables = resolve_variables(
        manifest.get('variables', {}), {'package_root': str(package_root)})
    if not isinstance(variables, dict):
        raise ManifestError('variables must be a mapping')
    rows = []
    for scene in manifest['scenes']:
        if scene_filter and scene.get('name') not in scene_filter:
            continue
        scene_dir = run_dir / str(scene['name'])
        if scene.get('available', True) is False or scene.get('enabled', True) is False:
            for mode_name in sorted(mode_filter or MODE_NAMES):
                output = scene_dir / 'unavailable' / mode_name
                reason = scene.get('reason', 'scene disabled')
                record = status_record(
                    'unavailable', scene, mode_name, 'UNAVAILABLE', reason,
                    output, sha256_payload({'scene': scene, 'mode': mode_name}),
                    utc_now(), 0.0)
                atomic_write_json(output / 'case_status.json', record)
                rows.append(record)
            continue
        for case in scene['cases']:
            route_value = case.get('route')
            if not route_value:
                raise ManifestError(f'{scene["name"]}/{case["name"]}: route is required')
            route_context = dict(variables)
            route_context.update({'scene': scene.get('name'), 'scene_id': scene.get('scene_id'),
                                  'package_root': str(package_root)})
            route_path = Path(resolve_template(route_value, route_context)).expanduser()
            if not route_path.is_absolute():
                route_path = package_root / route_path
            route_failures = validate_route(
                route_path, int(case.get('min_waypoints', 8)), str(case.get('coverage')))
            if route_failures:
                raise ManifestError(f'{scene["name"]}/{case["name"]}: ' + '; '.join(route_failures))
            for mode in case['modes']:
                if mode_filter and mode.get('name') not in mode_filter:
                    continue
                output = scene_dir / str(case['name']) / str(mode['name'])
                context = scene_context(variables, scene, case, mode, run_dir, output, package_root)
                context['route'] = str(route_path)
                context['timeout_sec'] = timeout_override or mode.get(
                    'timeout_sec', case.get('timeout_sec', manifest.get('defaults', {}).get(
                        'timeout_sec', DEFAULT_TIMEOUT_SEC)))
                rows.append(execute_case(
                    scene, case, mode, context, output, float(context['timeout_sec']),
                    resume, dry_run))
    return rows


def aggregate(rows: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    rows = list(rows)
    counts = {status: sum(row.get('status') == status for row in rows)
              for status in sorted(FINAL_STATUSES | {'RUNNING'})}
    return {
        'schema': 'robot_navigo.matrix_closed_loop_batch_summary',
        'schema_version': 1,
        'generated_at': utc_now(),
        'case_count': len(rows),
        'counts': counts,
        'pass': bool(rows) and all(row.get('status') in {'PASS', 'SKIPPED'} for row in rows),
        'contact_detection': {
            'status': 'UNAVAILABLE',
            'reason': 'No Matrix ROS contact topic is part of the batch contract',
        },
        'cases': rows,
    }


def html_report(summary: Mapping[str, Any]) -> str:
    def cell(value: Any) -> str:
        if value is None:
            return '<span class="na">N/A</span>'
        return html.escape(str(value))

    rows = []
    for case in summary.get('cases', []):
        metrics = case.get('metrics', {})
        rows.append(
            '<tr>'
            f'<td>{cell(case.get("scene"))}</td>'
            f'<td>{cell(case.get("case"))}</td>'
            f'<td>{cell(case.get("mode"))}</td>'
            f'<td class="status-{cell(case.get("status"))}">{cell(case.get("status"))}</td>'
            f'<td>{cell(metrics.get("action_status"))}</td>'
            f'<td>{cell(metrics.get("position_rmse_m"))}</td>'
            f'<td>{cell(metrics.get("path_length_m"))}</td>'
            f'<td>{cell(case.get("contact_detection", {}).get("status"))}</td>'
            f'<td>{cell(case.get("reason"))}</td>'
            '</tr>')
    counts = ', '.join(f'{key}: {value}' for key, value in summary.get('counts', {}).items())
    return f'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>Matrix 闭环批测报告</title>
<style>
body{{font-family:system-ui,sans-serif;margin:2rem;color:#202124}}
table{{border-collapse:collapse;width:100%;font-size:.92rem}}
th,td{{border:1px solid #d9d9d9;padding:.45rem;text-align:left;vertical-align:top}}
th{{background:#f2f4f7}} .status-PASS{{color:#087f23;font-weight:700}}
.status-FAIL,.status-INFRA_FAIL,.status-TIMEOUT{{color:#b42318;font-weight:700}}
.status-UNAVAILABLE,.status-SKIPPED,.na{{color:#667085}}
code{{white-space:pre-wrap}}
</style></head><body>
<h1>Matrix 规划控制闭环批测</h1>
<p>总体结果：<strong>{cell(summary.get('pass'))}</strong>；Case 数：{cell(summary.get('case_count'))}</p>
<p>状态统计：{html.escape(counts)}</p>
<p>接触/碰撞真值：<strong>UNAVAILABLE</strong>。当前契约没有 Matrix ROS contact topic，路径或 costmap 代理不等价于真实碰撞检测。</p>
<table><thead><tr><th>场景</th><th>Case</th><th>模式</th><th>状态</th><th>Action</th><th>定位 RMSE(m)</th><th>实际路径(m)</th><th>接触检测</th><th>原因</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table>
</body></html>
'''


@contextlib.contextmanager
def batch_lock(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a+', encoding='utf-8') as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding='utf-8') as stream:
            manifest = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ManifestError(f'cannot load suite: {error}') from error
    if not isinstance(manifest, dict):
        raise ManifestError('suite root must be a mapping')
    failures = validate_manifest(manifest)
    if failures:
        raise ManifestError('\n'.join(failures))
    return manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', required=True, help='suite YAML')
    parser.add_argument('--output', help='run directory; default output_root/timestamp')
    parser.add_argument('--report', help='HTML report path; default run_dir/MATRIX_REPORT.html')
    parser.add_argument('--lock-file', help='override the suite lock path')
    parser.add_argument('--scene', action='append', help='restrict to one or more scene names')
    parser.add_argument('--mode', action='append', choices=sorted(MODE_NAMES))
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--timeout-sec', type=float)
    parser.add_argument('--package-root', default=str(Path(__file__).resolve().parents[1]))
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        suite_path = Path(args.suite).expanduser().resolve()
        manifest = load_manifest(suite_path)
        package_root = Path(args.package_root).expanduser().resolve()
        output_root = Path(manifest.get('output_root', './matrix_closed_loop_batch')).expanduser()
        run_dir = Path(args.output).expanduser() if args.output else \
            output_root / dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        lock_path = Path(args.lock_file or manifest.get(
            'lock_file', str(output_root / '.matrix_closed_loop.lock')))
        if not lock_path.is_absolute():
            lock_path = suite_path.parent / lock_path
        with batch_lock(lock_path):
            run_dir.mkdir(parents=True, exist_ok=True)
            atomic_write_json(run_dir / 'batch_status.json', {
                'schema': 'robot_navigo.matrix_closed_loop_batch_status',
                'schema_version': 1, 'status': 'RUNNING', 'started_at': utc_now(),
                'suite': str(suite_path), 'dry_run': args.dry_run,
            })
            rows = collect_cases(
                manifest, run_dir, package_root,
                set(args.scene or []), set(args.mode or []), args.resume,
                args.dry_run, args.timeout_sec)
            summary = aggregate(rows)
            atomic_write_json(run_dir / 'summary.json', summary)
            report_path = Path(args.report).expanduser() if args.report else run_dir / 'MATRIX_REPORT.html'
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(html_report(summary), encoding='utf-8')
            atomic_write_json(run_dir / 'batch_status.json', {
                'schema': 'robot_navigo.matrix_closed_loop_batch_status',
                'schema_version': 1,
                'status': 'PASS' if summary['pass'] else 'FAIL',
                'finished_at': utc_now(),
                'summary': str(run_dir / 'summary.json'),
                'report': str(report_path),
            })
            print(json.dumps({
                'run_dir': str(run_dir), 'report': str(report_path),
                'status': 'PASS' if summary['pass'] else 'FAIL',
                'counts': summary['counts'],
            }, indent=2))
            return 0 if summary['pass'] else 1
    except ManifestError as error:
        print(f'[FAIL] {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
