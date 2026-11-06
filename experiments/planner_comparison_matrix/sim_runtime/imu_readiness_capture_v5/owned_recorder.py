"""Exact reviewed cal-v2 owned-child functions, used only for recorder here."""
import os
from pathlib import Path
import select
import signal
import subprocess
import time
import readiness_contract as c
import recording_deadline as deadline


def stat_birth(pid):
    fields=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,starttime_ticks=int(fields[19]),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def child(argv,log,env,check=None):
    with log.open('x') as stream:
        process=subprocess.Popen(argv,cwd='/',stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,env=env,close_fds=True)
    descriptor=os.pidfd_open(process.pid)
    owned=dict(process=process,fd=descriptor,birth=None,birth3=None,argv_ready=False,signalled=False)
    try:
        # Stable across exec even when /proc/cmdline is momentarily empty.
        # Retain this before any watch callback; complete argv is a separate gate.
        owned['birth3']=stat_birth(process.pid)
        until=time.monotonic()+2.
        while time.monotonic()<until:
            if select.select([descriptor],[],[],0)[0]:return owned
            try:
                actual=deadline.process_identity(process.pid)
                if any(actual[k]!=owned['birth3'][k] for k in ['pid','starttime_ticks','boot_id']) or actual['argv'] not in [list(argv),deadline.command_argv(argv)]:raise c.Hold('unexpected owned child exec identity')
                owned['birth']=actual;owned['argv_ready']=True
            except ProcessLookupError:pass
            if check is not None:check()
            if owned['argv_ready']:return owned
            time.sleep(.005)
        raise c.Hold('owned child identity unavailable')
    except BaseException as original:
        # A guard failure during spawn/exec must not lose the newly owned tool.
        cleanup=dict(birth3=owned['birth3'],argv_ready=owned['argv_ready'],original_error=str(original))
        try:
            interrupt(owned,[list(argv),deadline.command_argv(argv)])
        except BaseException as exc:cleanup['signal_error']=str(exc)
        finally:
            try:process.wait(timeout=3.)
            except BaseException as exc:cleanup['wait_error']=str(exc)
            cleanup.update(sigint_sent=owned['signalled'],exited=process.poll() is not None)
            try:deadline.atomic_json(log.with_suffix('.spawn_cleanup.json'),cleanup)
            finally:os.close(descriptor)
        raise


def interrupt(owned,allowed_argv):
    if owned is None or owned['signalled'] or select.select([owned['fd']],[],[],0)[0]:return
    birth3=owned['birth3'];actual3=stat_birth(owned['process'].pid)
    if birth3 is None or actual3!=birth3:raise c.Hold('owned-child stat birth changed; no signal')
    try:actual=deadline.process_identity(owned['process'].pid)
    except ProcessLookupError:
        if owned['argv_ready']:raise c.Hold('admitted owned-child argv unavailable; no signal')
        # This is only the immediately retained Popen pidfd, before admission.
        # An exec-empty cmdline cannot turn it into another process instance.
    else:
        if any(actual[k]!=birth3[k] for k in ['pid','starttime_ticks','boot_id']) or actual['argv'] not in allowed_argv:raise c.Hold('owned-child argv changed; no signal')
    signal.pidfd_send_signal(owned['fd'],signal.SIGINT);owned['signalled']=True
