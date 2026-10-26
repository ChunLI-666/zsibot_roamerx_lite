#!/usr/bin/env python3
"""Small native correctness tests only: no ROS or production CPU benchmark."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

from build_helper import ROOT, sha

LIBRARY = None


def clean_environment():
    env = os.environ.copy()
    for key in list(env):
        if (key.startswith(('LOCNAV_HEAP_SAMPLE', 'HEAP_PROFILE')) or
                key in ['LD_PRELOAD', 'HEAPPROFILE', 'HEAPCHECK', 'CPUPROFILE', 'TCMALLOC_SAMPLE_PARAMETER']):
            env.pop(key)
    return env


def run_fixture(build, output, name, option=None, enabled=True, interval='3600', parameter='524288'):
    directory = output / name
    directory.mkdir()
    env = clean_environment()
    env['LD_PRELOAD'] = str(LIBRARY) + ':' + str(build / 'liblocnav_heap_sample.so')
    if enabled:
        env.update(LOCNAV_HEAP_SAMPLE='1', LOCNAV_HEAP_SAMPLE_DIR=str(directory),
            LOCNAV_HEAP_SAMPLE_EXPECT_EXE=str(build / 'fixture'),
            LOCNAV_HEAP_SAMPLE_INTERVAL_SECONDS=interval, TCMALLOC_SAMPLE_PARAMETER=parameter)
    started = time.monotonic()
    result = subprocess.run([str(build / 'fixture'), *([option] if option else [])],
        env=env, text=True, capture_output=True, timeout=15)
    (output / (name + '.stdout')).write_text(result.stdout)
    (output / (name + '.stderr')).write_text(result.stderr)
    assert result.returncode == 0, result.stderr
    manifests = list(directory.glob('*/sampler_manifest.json'))
    summaries = list(directory.glob('*/sampler_summary.json'))
    return dict(directory=directory, elapsed=time.monotonic()-started, stdout=result.stdout,
        manifests=manifests, summaries=summaries)


def parse_heap(path):
    text = path.read_text()
    header = re.match(r'heap profile:\s*(\d+):\s*(\d+)\s*\[\s*(\d+):\s*(\d+)\]\s*@ heap_v2/524288', text)
    assert header, text[:200]
    assert 'MAPPED_LIBRARIES:\n' in text
    buckets = re.findall(r'^\s*(\d+):\s*(\d+)\s*\[\s*(\d+):\s*(\d+)\]\s*@\s+0x', text, re.M)
    values = [tuple(map(int, row)) for row in buckets]
    totals = tuple(map(int, header.groups()))
    assert tuple(sum(row[i] for row in values) for i in range(4)) == totals, 'Truncated/incomplete raw buckets'
    assert totals[:2] == totals[2:], 'Expected a live-only sample, not cumulative allocations'
    return totals


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--pprof', type=Path, required=True, help='Explicit compatible gperftools google-pprof executable')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    build, output = args.build.resolve(), args.output.resolve()
    global LIBRARY
    LIBRARY = Path(json.loads((build / 'build_manifest.json').read_text())['library']).resolve()
    if not (build / 'fixture').is_file() or not args.pprof.is_file():
        parser.error('Build with --with-fixture and provide an existing --pprof; nothing is installed automatically')
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    report = dict(success=False, build_manifest_sha256=sha(build / 'build_manifest.json'), tests=[])
    try:
        disabled = run_fixture(build, output, 'disabled_default', '--disabled-probe', enabled=False)
        assert json.loads(disabled['stdout'])['tasks'] == 1
        assert not list(disabled['directory'].iterdir())
        report['tests'].append('default_disabled_no_worker_or_output')

        quick = run_fixture(build, output, 'exit_wakes_long_wait', '--quick-exit')
        assert quick['elapsed'] < 2.0, quick['elapsed']
        summary = json.loads(quick['summaries'][0].read_text())
        assert summary['worker_stopped'] and summary['completed'] == 0 and summary['errors'] == 0
        report['tests'].append('normal_exit_wakes_3600s_wait_and_joins')

        default_interval = run_fixture(build, output, 'default_interval', '--quick-exit', interval='10')
        assert json.loads(default_interval['manifests'][0].read_text())['interval_seconds'] == 10
        report['tests'].append('diagnostic_10_second_interval')

        invalid = run_fixture(build, output, 'invalid_sampling_config', '--quick-exit', parameter='0')
        assert not invalid['manifests']
        assert json.loads(invalid['summaries'][0].read_text())['errors'] == 1
        assert list(invalid['directory'].glob('*/sampler_error.json'))
        assert not list(invalid['directory'].glob('*/*.heap'))
        report['tests'].append('sampling_disabled_or_stub_config_fail_closed')

        periodic = run_fixture(build, output, 'periodic_worker', '--periodic-probe', interval='.05')
        assert len(list(periodic['directory'].glob('*/sample_*.heap'))) >= 2
        assert json.loads(periodic['summaries'][0].read_text())['errors'] == 0
        report['tests'].append('ordinary_worker_periodic_multiple_snapshots')

        retained = run_fixture(build, output, 'retain_release_concurrent')
        directory = retained['manifests'][0].parent
        manifest = json.loads(retained['manifests'][0].read_text())
        assert 'TCMallocImplementation' in manifest['malloc_extension_dynamic_type']
        assert Path(manifest['allocator_library']).resolve() == LIBRARY.resolve()
        assert not manifest['full_heap_profiler_running']
        summary = json.loads(retained['summaries'][0].read_text())
        assert summary['completed'] == 3 and summary['errors'] == 0 and summary['worker_stopped']
        metadata = [json.loads((directory / f'sample_{i}.json').read_text()) for i in range(1, 4)]
        totals = [parse_heap(directory / f'sample_{i}.heap') for i in range(1, 4)]
        assert totals[0][1] - totals[1][1] >= 24*1024*1024
        assert totals[1][1] - totals[2][1] >= 24*1024*1024
        assert metadata[0]['reason'] == 'retained_and_releasable\nquoted"label'
        for row in metadata:
            assert row['valid'] and row['kind'] == 'sampled_live_heap_estimate'
            assert row['worker_tid'] != row['pid'] and row['starttime'] > 0
            assert row['collection_wall_ns'] >= 0 and row['collection_thread_cpu_ns'] >= 0
            assert not row['cumulative_allocations_available'] and not row['full_heap_profiler_running']
        report['tests'] += ['real_virtual_allocator_implementation_not_base_stub',
            'retained_freed_live_semantics_and_concurrent_allocations', 'complete_raw_buckets_and_pid_maps',
            'json_label_and_collection_cost_metadata']

        pprof = args.pprof.resolve()
        renderer_path = ROOT.parent / 'report.py'
        spec = importlib.util.spec_from_file_location('existing_flame_renderer', renderer_path)
        renderer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(renderer)
        texts = []
        for i in range(1, 4):
            prefix = output / f'pprof_{i}'
            env = clean_environment()
            env['PPROF_TMPDIR'] = str(output / 'pprof_tmp')
            for mode in ['text', 'collapsed']:
                result = subprocess.run([str(pprof), '--' + mode, '--show_bytes', '--inuse_space',
                    str(build / 'fixture'), str(directory / f'sample_{i}.heap')],
                    env=env, text=True, capture_output=True, timeout=15)
                prefix.with_suffix('.' + mode).write_text(result.stdout)
                prefix.with_suffix('.' + mode + '.stderr').write_text(result.stderr)
                assert result.returncode == 0, result.stderr
                assert 'Heap version 2' in result.stderr and '1-in-524288' in result.stderr
                if mode == 'text':
                    texts.append(result.stdout)
                else:
                    if result.stdout.strip():
                        stacks = renderer.parse_collapsed(result.stdout)
                        svg = renderer.svg(stacks, 'Fixture sampled live heap estimate', 'estimated live bytes')
                        ET.fromstring(svg)
                        prefix.with_suffix('.svg').write_text(svg)
                    else:
                        assert totals[i-1][1] == 0
                        try:
                            renderer.parse_collapsed(result.stdout)
                        except ValueError:
                            pass
                        else:
                            raise AssertionError('Empty live sample must not become a fake flame')
        assert 'AllocateRetained' in texts[0] and 'AllocateReleasable' in texts[0]
        assert 'AllocateRetained' in texts[1] and 'AllocateReleasable' not in texts[1]
        assert 'AllocateRetained' not in texts[2] and 'AllocateReleasable' not in texts[2]
        report['tests'].append('pprof_real_v2_correction_collapsed_svg_and_known_stack_free')
        report['raw_sampled_live_header_totals'] = totals
        report['sample_metadata'] = metadata
        report['success'] = True
    finally:
        report['artifact_sha256'] = {str(path.relative_to(output)): sha(path)
            for path in sorted(output.rglob('*')) if path.is_file() and path.name != 'verification.json'}
        (output / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(dict(success=report['success'], tests=report['tests'], output=str(output)), indent=2))


if __name__ == '__main__':
    main()
