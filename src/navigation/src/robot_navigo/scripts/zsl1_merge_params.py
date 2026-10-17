#!/usr/bin/env python3
"""Create a complete reviewable experiment config; never launch or deploy it."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import yaml


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def merge(base, patch):
    result = copy.deepcopy(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def merge_experiment(base, controller, model, envelope):
    result = merge(merge(base, controller), model)
    for name in ('global_costmap', 'local_costmap'):
        parameters = result[name][name]['ros__parameters']
        if json.loads(parameters['footprint']) != envelope['footprint'] or parameters['footprint_padding'] != envelope['footprint_padding']:
            raise ValueError(f'{name} differs from the authoritative model envelope')
        if parameters['inflation_layer']['inflation_radius'] != envelope['inflation_radius']:
            raise ValueError(f'{name} inflation differs from the model envelope')
        if not parameters.get('plugins'):
            raise ValueError(f'{name} is missing complete base configuration')
    if not result['controller_server']['ros__parameters']['FollowPath']['forward_alignment']['enabled']:
        raise ValueError('Expected the explicit forward-alignment experiment overlay')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('base', 'controller-overlay', 'model-overlay', 'envelope', 'output'):
        parser.add_argument('--' + name, required=True)
    args = parser.parse_args()
    paths = [Path(args.base), Path(args.controller_overlay), Path(args.model_overlay), Path(args.envelope)]
    output = Path(args.output)
    if output.resolve() in {path.resolve() for path in paths}:
        raise ValueError('Output must not overwrite an input configuration')
    result = merge_experiment(*(yaml.safe_load(path.read_text()) for path in paths))
    output.write_text('# Complete model-only experiment parameters; not approved for hardware deployment.\n' + yaml.safe_dump(result, sort_keys=False))
    manifest = dict(inputs=[dict(path=str(path.resolve()), sha256=digest(path)) for path in paths],
        output=dict(path=str(output.resolve()), sha256=digest(output)), hardware_deployment_ready=False)
    Path(str(output) + '.manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest['output']))
