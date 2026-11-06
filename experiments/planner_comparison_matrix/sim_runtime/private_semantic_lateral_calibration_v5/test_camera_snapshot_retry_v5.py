#!/usr/bin/env python3
"""Pure fakeclock/proof/socket tests;no ROS,real socket,process or signal."""
import copy,io,json,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));sys.dont_write_bytecode=True
import lateral_contract as c
import semantic_tool_timed as b
FLAG='allow_camera_stale_lease_retry_v1'
class Clock:
 def __init__(self):self.n=1_000_000_000;self.wall=1793994700000000000
 def time_ns(self):return self.wall+self.n
 def monotonic_ns(self):return self.n
 def clock_gettime(self,kind):return self.n/1e9
 def sleep(self,x):self.n+=round(x*1e9)
class Signals:
 SIGINT=2;SIGTERM=15
 def signal(self,*a):return None
class Socket:
 def __init__(self,clock):self.clock=clock;self.rows=[];self.closed=False
 def settimeout(self,x):pass
 def sendto(self,payload,target):self.rows.append((json.loads(payload),self.clock.n));return len(payload)
 def close(self):self.closed=True
class Fixture:
 def __init__(self,enabled=True):
  self.clock=Clock();self.case={FLAG:enabled};self.release={FLAG:enabled,'release_id':'f'*32};self.identities={1:{'pid':1},2:{'pid':2},3:{'pid':3}};self.ready=dict(parent_identity=self.identities[1],recorder_identity=self.identities[2],helper_identity=self.identities[3],deadline_boottime_sec=100.)
  self.reads=0;self.hostcalls=0;self.idcalls=0;self.mode='refresh';self.changed=False;self.events=[]
  self.contract=SimpleNamespace(Hold=c.Hold,CAMERA_RETRY_SCHEMA=c.CAMERA_RETRY_SCHEMA,camera_stale_lease_retry_enabled=c.camera_stale_lease_retry_enabled,camera_stale_lease_details=c.camera_stale_lease_details,check_live=c.check_live,host_proof=self.host,read=self.read)
  self.deadline=SimpleNamespace(process_identity=self.identity)
 def lease(self,stale=False,generation=1):
  now,steady=self.clock.time_ns(),self.clock.n;samples={}
  for t in c.TOPICS:samples[t]=dict(topic=t,stamp_ns=now-100_000_000,received_steady_ns=steady-10_000_000,payload_valid=True,frame='map')
  samples['/odom'].update(child='BASE_LINK',position=[0.,0.,.33],quaternion=[0.,0.,0.,1.],linear=[0.,0.,0.],angular=[0.,0.,0.],position_speed_mps=0.)
  if stale:samples[c.CAMERA_TOPICS[0]].update(stamp_ns=now-301_000_000)
  return dict(release_id=self.release['release_id'],parent_identity=self.ready['parent_identity'],written_steady_ns=steady,anchor=[0.,0.,0.],samples=samples,generation=generation)
 def read(self,path):
  self.reads+=1;old=self.mode!='fresh'and(self.mode!='refresh'or self.reads==1);lease=self.lease(old,1 if old else 2)
  if self.changed:self.changed(lease)
  return lease
 def host(self,*a,**kw):
  self.hostcalls+=1
  if self.mode=='host_bad':raise c.Hold('external host proof too old/future')
 def identity(self,pid):
  self.idcalls+=1
  return {'pid':999}if self.mode=='identity_bad'else self.identities[pid]
 def prove(self,end=None):return b.prove_parent(self.case,self.release,self.ready,Path('/purefixture'),self.contract,self.deadline,self.clock,self.events,end)
class RetryTests(unittest.TestCase):
 def test_strict_dual_authorization_and_default_original_failure(self):
  self.assertFalse(c.camera_stale_lease_retry_enabled({},{}))
  for a,v in [(True,False),(True,None),(1,True),(False,0),(None,None)]:
   with self.subTest(a=a,v=v),self.assertRaises(c.Hold):c.camera_stale_lease_retry_enabled({FLAG:a},{FLAG:v})
  f=Fixture(False)
  with self.assertRaises(c.Hold):f.prove()
  self.assertEqual(f.reads,1);self.assertEqual(f.clock.n,1_000_000_000);self.assertEqual(f.events,[])
 def test_fresh_reread_rechecks_fullproof_and_preserves_snapshots(self):
  f=Fixture();f.prove();self.assertEqual(f.reads,2);self.assertEqual(f.hostcalls,2);self.assertEqual(f.idcalls,6);self.assertEqual(f.clock.n,1_001_000_000)
  self.assertEqual([r['event']for r in f.events],['camera_stale_snapshot_retry_wait','camera_snapshot_retry_snapshot_guard_passed']);self.assertEqual(f.events[0]['lease_snapshot']['generation'],1);self.assertEqual(f.events[1]['lease_snapshot']['generation'],2)
 def test_timeout_during_active_yields_only_five_neutrals_and_close(self):
  f=Fixture();sock=Socket(f.clock);f.mode='never_refresh'
  result=b.run_probe(.4,1.5,lambda:sock,lambda:None,io.StringIO(),f.clock,Signals(),proof_with_deadline=lambda end:f.prove(end))
  self.assertFalse(result['completed']);self.assertEqual(result['motion_packets_sent'],0);self.assertEqual(result['final_zero_logged_successes'],5);self.assertTrue(sock.closed);self.assertTrue(all(x['lateral']==0. for x,t in sock.rows));self.assertLessEqual(f.reads,32)
  waits=[r for r in f.events if r['event']=='camera_stale_snapshot_retry_wait'];self.assertLessEqual(waits[-1]['now_steady_ns']-waits[0]['now_steady_ns'],30_000_000)
 def test_other_guards_cannot_be_repaired_by_camera_retry(self):
  mutations={
   'GTstale':lambda l:l['samples']['/odom'].update(stamp_ns=f.clock.time_ns()-301_000_000),
   'IMUstale':lambda l:l['samples']['/front_lidar/imu'].update(stamp_ns=f.clock.time_ns()-301_000_000),
   'lidarstale':lambda l:l['samples']['/front_lidar'].update(stamp_ns=f.clock.time_ns()-301_000_000),
   'futurecamera':lambda l:l['samples'][c.CAMERA_TOPICS[1]].update(stamp_ns=f.clock.time_ns()+51_000_000),
   'payloadcamera':lambda l:l['samples'][c.CAMERA_TOPICS[1]].update(payload_valid=False),
   'futureReceipt':lambda l:l['samples'][c.CAMERA_TOPICS[1]].update(received_steady_ns=f.clock.n+1),
   'GTspeed':lambda l:l['samples']['/odom'].update(linear=[1.01,0.,0.]),
   'GTlying':lambda l:l['samples']['/odom'].update(position=[0.,0.,.1]),
   'GTNaN':lambda l:l['samples']['/odom'].update(linear=[float('nan'),0.,0.]),
   'GTframe':lambda l:l['samples']['/odom'].update(frame='other'),
   'GTcross':lambda l:l['samples']['/odom'].update(position=[.11,0.,.33]),
   'leaseTTL':lambda l:l.update(written_steady_ns=f.clock.n-150_000_001)}
  for label,fn in mutations.items():
   f=Fixture();f.mode='never_refresh';f.changed=fn
   with self.subTest(label=label),self.assertRaises((c.Hold,b.CanaryHold,ValueError)):f.prove()
   self.assertEqual(f.clock.n,1_000_000_000);self.assertFalse(any(x['event']=='camera_stale_snapshot_retry_wait'for x in f.events))
  for mode in ['host_bad','identity_bad']:
   f=Fixture();f.mode=mode
   with self.subTest(mode=mode),self.assertRaises((c.Hold,b.CanaryHold)):f.prove()
   self.assertEqual(f.clock.n,1_000_000_000)
  f=Fixture();f.ready['deadline_boottime_sec']=10.
  with self.assertRaises(b.CanaryHold):f.prove()
  self.assertEqual(f.clock.n,1_000_000_000)
 def test_active_deadline_bounds_wait_no_late_nonzero(self):
  f=Fixture();f.mode='never_refresh'
  with self.assertRaises(b.CanaryHold):f.prove(f.clock.n+500_000)
  self.assertEqual(f.clock.n,1_000_500_000)
 def test_success_guard_crossing30ms_cannot_send_positive(self):
  f=Fixture();original=f.contract.check_live
  def slow(samples,*a):
   result=original(samples,*a)
   if f.reads>=2:f.clock.n+=31_000_000
   return result
  f.contract.check_live=slow;sock=Socket(f.clock)
  result=b.run_probe(.4,1.5,lambda:sock,lambda:None,io.StringIO(),f.clock,Signals(),proof_with_deadline=lambda end:f.prove(end))
  self.assertEqual(result['motion_packets_sent'],0);self.assertEqual(result['final_zero_logged_successes'],5);self.assertTrue(sock.closed)
 def test_success_evidence_copy_crossing30ms_cannot_send_positive(self):
  f=Fixture();sock=Socket(f.clock);original=b.json.dumps
  def slow(value,*a,**kw):
   text=original(value,*a,**kw)
   if isinstance(value,dict)and value.get('generation')==2:f.clock.n+=31_000_000
   return text
  with mock.patch.object(b.json,'dumps',slow):result=b.run_probe(.4,1.5,lambda:sock,lambda:None,io.StringIO(),f.clock,Signals(),proof_with_deadline=lambda end:f.prove(end))
  self.assertEqual(result['motion_packets_sent'],0);self.assertEqual(result['final_zero_logged_successes'],5);self.assertTrue(sock.closed)
 def test_retry_events_not_motion_errors_and_no_catchup(self):
  f=Fixture();sock=Socket(f.clock);result=b.run_probe(.4,1.5,lambda:sock,lambda:None,io.StringIO(),f.clock,Signals(),proof_with_deadline=lambda end:f.prove(end))
  self.assertTrue(result['completed']);self.assertEqual(result['errors'],[]);self.assertTrue(f.events)
  stamps=[t for command,t in sock.rows if command['lateral']]
  self.assertTrue(all(b-a>=50_000_000 for a,b in zip(stamps,stamps[1:])));self.assertTrue(all(t<result['active_deadline_steady_ns']for t in stamps));self.assertLessEqual(len(stamps),30)
if __name__=='__main__':unittest.main(verbosity=2)
