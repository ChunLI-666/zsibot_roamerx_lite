"""Single SQLite owner. Main-thread waits pump ROS; durability is never deferred."""
import copy
from concurrent.futures import Future, TimeoutError
import queue
import threading
import time

from mission_state import MissionStore, Hold, remaining_state, check_identity_state


class StorageWorker:
    METHODS=frozenset(('create','operation','finish_operation','reserve','acknowledge','observe',
        'finish_attempt','record_not_sent','export','import_closed','flush_state_read_timing'))

    def __init__(self,path,resource=None,*,factory=MissionStore):
        self.pump=None;self.busy=False;self._closed=False;self._state=None;self._sink=None
        self.timing_drops=0;self._jobs=queue.Queue(maxsize=1);self._timings=queue.Queue(maxsize=256)
        self._owner=threading.get_ident();self._sequence=0;self._ready=Future()
        self._closed_result=Future()
        self._thread=threading.Thread(target=self._run,args=(path,resource,factory),name='mission-sqlite-owner')
        self._thread.start()
        try:self._ready.result()
        except BaseException:self._thread.join();raise

    def _run(self,path,resource,factory):
        store=None;future=None
        try:
            store=factory(path,resource)
            self._ready.set_result(True)
            while True:
                job=self._jobs.get()
                if job is None:break
                sequence,name,args,kwargs,output,conditional,future=job
                result=None;error=None;started=time.monotonic_ns()
                try:
                    if name=='_timing':store.timing_sink=self._timings.put_nowait if args[0] else None
                    else:result=getattr(store,name)(*args,**kwargs)
                    if output is not None and (not conditional or result):store.export(output)
                except BaseException as exc:error=exc
                try:state=store.state()
                except Hold:state=None
                except BaseException as exc:state=None;error=error or exc
                # No ROS, caller-owned objects or caller callbacks run on this thread.
                future.set_result((copy.deepcopy(result),state,error,store.timing_drops,
                    dict(job_id=sequence,operation=name,owner_thread_id=threading.get_ident(),
                        started_steady_ns=started,finished_steady_ns=time.monotonic_ns())))
        except BaseException as exc:
            if not self._ready.done():self._ready.set_exception(exc)
            elif future is not None and not future.done():future.set_exception(exc)
        finally:
            try:
                if store is not None:store.close()
            except BaseException as exc:self._closed_result.set_exception(exc)
            else:self._closed_result.set_result(True)

    def _drain_timing(self):
        while True:
            try:sample=self._timings.get_nowait()
            except queue.Empty:break
            if self._sink is not None:
                try:self._sink(sample)
                except Exception:self.timing_drops+=1

    def call(self,name,*args,output=None,export_if_changed=False,**kwargs):
        if threading.get_ident()!=self._owner:raise Hold('Storage facade requires its main-thread owner')
        if self._closed or self.busy:raise Hold('Closed or reentrant storage call')
        if not self._thread.is_alive():raise Hold('Storage owner is not running')
        if name not in self.METHODS and name!='_timing':raise Hold('Unsupported storage operation')
        future=Future();self._sequence+=1;deferred=None;self.busy=True
        try:
            self._jobs.put_nowait((self._sequence,name,copy.deepcopy(args),copy.deepcopy(kwargs),output,export_if_changed,future))
            while not future.done():
                self._drain_timing()
                if self.pump is not None:
                    try:self.pump()
                    except BaseException as exc:deferred=deferred or exc
                try:future.result(timeout=.002)
                except TimeoutError:pass
            result,state,error,drops,observation=future.result()
            self._state=state;self.timing_drops=max(self.timing_drops,drops);self._drain_timing()
            if self._sink is not None:
                try:self._sink(dict(stage='storage_job',success=error is None,**observation))
                except Exception:self.timing_drops+=1
            if error is not None:raise error
            if deferred is not None:raise deferred
            return result
        finally:self.busy=False

    def __getattr__(self,name):
        if name in self.METHODS:return lambda *args,**kwargs:self.call(name,*args,**kwargs)
        raise AttributeError(name)

    def state(self):
        if self._state is None:
            # Only used before create or for opening an existing mission, before ROS.
            if self.busy:raise Hold('No committed state available during storage work')
            self.call('flush_state_read_timing','initial_snapshot')
        if self._state is None:raise Hold('Mission has not been created')
        return copy.deepcopy(self._state)

    def remaining(self):return remaining_state(self.state())
    def check_identity(self,identity):return check_identity_state(self.state(),identity)

    @property
    def timing_sink(self):return self._sink
    @timing_sink.setter
    def timing_sink(self,value):
        self.call('_timing',value is not None);self._sink=value

    def close(self):
        if self._closed:return
        if self.busy:raise Hold('Cannot release storage owner during an unfinished checkpoint')
        self._jobs.put(None);self._thread.join();self._closed=True;self._drain_timing();self._closed_result.result()
