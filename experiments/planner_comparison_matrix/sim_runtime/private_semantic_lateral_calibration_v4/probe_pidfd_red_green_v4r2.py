"""Same inherited pidfd-open counterexample, old and new, pure Popen stand-ins."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
from unittest import mock

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))

def probe(source):
    spec=importlib.util.spec_from_file_location('pure_'+source.parent.name,source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    argv=['/usr/bin/python3','-u','/pure_backend.py']
    birth=dict(pid=123,starttime_ticks=456,boot_id='pure-boot')
    process=SimpleNamespace(pid=123,args=argv,returncode=None,poll=mock.Mock(return_value=None),
        wait=mock.Mock(side_effect=subprocess.TimeoutExpired(argv,3.)))
    with tempfile.TemporaryDirectory() as temp,mock.patch.object(module.subprocess,'Popen',return_value=process),\
         mock.patch.object(module,'stat_birth',return_value=birth),\
         mock.patch.object(module.deadline,'command_argv',return_value=argv),\
         mock.patch.object(module.deadline,'process_identity',side_effect=ProcessLookupError('pure empty argv')),\
         mock.patch.object(module.os,'pidfd_open',side_effect=OSError('pure fd failure')),\
         mock.patch.object(module.signal,'pidfd_send_signal') as signal:
        try:module.child(argv,Path(temp)/'child.log',{})
        except OSError as error:
            result=dict(error=str(error),pid=123,wait_calls=process.wait.call_count,still_live=process.poll() is None,
                signals=signal.call_count,cleanup_receipts=[p.name for p in Path(temp).glob('*.spawn_cleanup.json')],
                retained_failure=getattr(error,'owned_spawn_cleanup',None))
        else:raise AssertionError('Expected pure inherited failure')
    return result

if __name__=='__main__':
    result=dict(mock_only=True,old_v4=probe(HERE.parent/'review_source_v4/lateral_once.py'),
        new_v4r2=probe(HERE/'lateral_once.py'))
    assert result['old_v4']['wait_calls']==0 and result['old_v4']['cleanup_receipts']==[]
    assert result['new_v4r2']['wait_calls']==1 and result['new_v4r2']['signals']==0
    record=result['new_v4r2']['retained_failure']
    assert record['pid']==123 and record['birth3']['starttime_ticks']==456 and record['state']=='HOLD_OWNED_CHILD_OUTSTANDING'
    with (HERE/'pidfd_red_green_v4r2.json').open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(old_wait=0,new_wait=1,new_state=record['state'],no_signal=True)))
