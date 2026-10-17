#!/usr/bin/env python3
"""Compare instrumented commands against unmodified evalControl on matching inputs."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    root = parser.parse_args().root.resolve()
    output = root / 'native_validation'
    output.mkdir(exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='184', ROS_LOCALHOST_ONLY='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    env['LD_LIBRARY_PATH'] = str(root / 'install/lib') + ':' + env.get('LD_LIBRARY_PATH', '')
    env['AMENT_PREFIX_PATH'] = str(root / 'install') + ':' + env.get('AMENT_PREFIX_PATH', '')
    results = []
    for run in json.loads((root / 'runs.json').read_text()):
        if run['fixed'] or run['seed'] != 42 or run['weight'] != 5:
            continue
        original = Path(run['directory'])
        directory = output / run['name']
        directory.mkdir()
        case = yaml.safe_load((original / 'case.yaml').read_text())
        case['native_only'] = True
        (directory / 'case.yaml').write_text(yaml.safe_dump(case))
        command = [str(root / 'probe_build/cost_probe'), str(directory / 'case.yaml'),
                   str(root / 'inputs/params_weight5.yaml'), str(directory / 'native')]
        with (directory / 'process.log').open('w') as log:
            subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=30, check=True)
        actual = yaml.safe_load((directory / 'native.yaml').read_text())['command']
        expected = yaml.safe_load((original / 'trace.yaml').read_text())['diagnostic_command']
        difference = max(abs(a - b) for a, b in zip(actual, expected))
        if difference > 1e-6:
            raise RuntimeError(f"Native command mismatch: {run['name']}: {difference}")
        results.append(dict(name=run['name'], command=actual, max_difference=difference, argv=command))
    if len(results) != 18:
        raise RuntimeError('Expected 18 native comparisons')
    report = dict(count=len(results), max_difference=max(r['max_difference'] for r in results), results=results)
    (output / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: report[k] for k in ('count', 'max_difference')}))


if __name__ == '__main__':
    main()
