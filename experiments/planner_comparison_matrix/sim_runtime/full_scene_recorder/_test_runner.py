#!/usr/bin/python3
"""Fixture injection is confined to this test script, absent from production CLI."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import full_scene_recording_owner as owner

root = Path(sys.argv[1])
config_path = root/'config.json'
config = owner.read_json(config_path)
owner.TAIL_SEC = .2
owner.CLOSE_RESERVE_SEC = .1
owner.CLOSE_WAIT_SEC = 3.
owner.REQUIRED_TOPICS = {'/fixture'}
owner.recorder_command = lambda config, output: [sys.executable, str(HERE/'_test_standins.py'), 'recorder', str(output/'bag')]
owner.snapshot_repositories = lambda config, output: owner.write(output/'repositories_at_recording_start.json', {'fixture_only': True})
os.environ.update(config['expected_environment'])
real_popen = subprocess.Popen
def checked_popen(argv, *args, **kwargs):
    marker = root/(config['recording_run_id'] + '.recording_once.json')
    assert marker.exists(), 'first Popen happened before once marker'
    with (root/'popen_events.jsonl').open('a') as stream:
        stream.write(json.dumps(list(argv)) + '\n')
    return real_popen(argv, *args, **kwargs)
subprocess.Popen = checked_popen
if config.get('fixture_reject_identity_before_exec_ack'):
    actual_identity = owner.deadline.process_identity
    def identity_after_ack_only(pid):
        plan_path = root/'run/recording_deadline_recorder_plan.json'
        registration_path = root/'run/recording_deadline_recorder_registration.json'
        if plan_path.exists() and pid != os.getpid():
            # The independent helper runs in another interpreter. Only emulate
            # the parent's empty-cmdline race for its own recorder child.
            children_path = root/'run/recording_deadline_children_armed.json'
            registration = owner.read_json(registration_path) if registration_path.exists() else None
            recorder_pid = registration['identity']['pid'] if registration else None
            if recorder_pid is None:
                stat = Path('/proc')/str(pid)/'stat'
                if stat.exists() and int(stat.read_text().rsplit(')',1)[1].split()[1]) == os.getpid():
                    args = (Path('/proc')/str(pid)/'cmdline').read_bytes()
                    if b'--owned-exec' in args:
                        raise ProcessLookupError('fixture: transient empty recorder cmdline before ACK')
            elif pid == recorder_pid:
                acknowledged = owner.read_json(children_path) if children_path.exists() else {}
                if 'recorder' not in acknowledged.get('exec_verified', []):
                    raise ProcessLookupError('fixture: transient empty recorder cmdline before ACK')
        return actual_identity(pid)
    owner.deadline.process_identity = identity_after_ack_only
if len(sys.argv) > 2 and sys.argv[2] == 'exit':
    def exit_after_ready():
        while not (root/'run/recorder_ready.json').exists():
            time.sleep(.02)
        time.sleep(.1)
        os._exit(17)
    threading.Thread(target=exit_after_ready, daemon=True).start()
result = owner.run(config_path, root/'run', root/'release.json')
(root/'runner_return.json').write_text(json.dumps(result))
