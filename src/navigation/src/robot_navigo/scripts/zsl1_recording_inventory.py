#!/usr/bin/env python3
"""Read recorded topic metadata and prior lossless debug extracts for joint evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml


def file_info(path):
    return dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def inventory(metadata_root, extracts):
    bags, debug = [], []
    for path in sorted(Path(metadata_root).rglob('metadata.yaml')):
        meta = yaml.safe_load(path.read_text())['rosbag2_bagfile_information']
        topics = [dict(name=t['topic_metadata']['name'], type=t['topic_metadata']['type'], count=t['message_count']) for t in meta['topics_with_message_count']]
        candidates = [t for t in topics if any(k in (t['name'] + ' ' + t['type']).lower() for k in ('joint', 'motor', 'lowstate', 'highstate'))]
        bags.append(dict(**file_info(path), topic_count=len(topics), possible_joint_topics=candidates, topics=topics))
    for path in sorted(Path(extracts).glob('*_extracted.json')):
        rows = json.loads(path.read_text()).get('/yz_robot_ctrl/debug', [])
        keys, count = set(), 0
        for row in rows:
            payload = row.get('payload', '')
            count += any(k in payload.lower() for k in ('joint', 'motor_state', 'q_raw'))
            try:
                value = json.loads(payload)
                if isinstance(value, dict): keys.update(value)
            except (ValueError, TypeError):
                pass
        debug.append(dict(**file_info(path), debug_message_count=len(rows), payload_keys=sorted(keys), possible_joint_payloads=count))
    return dict(schema_version=1, bags=bags, existing_debug_extracts=debug,
        finding='No joint trajectory can be inferred from base odometry or body velocity commands. Topic inventory and extracted payload evidence are separate from raw bag re-deserialization.',
        joint_observations_available=any(x['possible_joint_topics'] for x in bags) or any(x['possible_joint_payloads'] for x in debug))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--metadata-root', required=True)
    p.add_argument('--debug-extracts', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    result = inventory(args.metadata_root, args.debug_extracts)
    Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(dict(bags=len(result['bags']), debug_extracts=len(result['existing_debug_extracts']), joint_observations_available=result['joint_observations_available'])))
