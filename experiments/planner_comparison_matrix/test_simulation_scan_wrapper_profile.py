import ast,copy,hashlib,importlib.util,json,math,sys,tempfile,time,types,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent;GOAL=ROOT/'goal_pilot_source'
spec=importlib.util.spec_from_file_location('waypoint_contract',GOAL/'waypoint_contract.py');c=importlib.util.module_from_spec(spec);sys.modules[spec.name]=c;spec.loader.exec_module(c)
TREE=ast.parse((GOAL/'waypoint_batch.py').read_text());FROZEN=ast.parse((GOAL/'pilot_scan_timing.py').read_text())
def scope():
 s=dict(json=json,hashlib=hashlib,math=math,time=time,Path=Path,__file__=str(GOAL/'waypoint_batch.py'),HERE=GOAL,FROZEN_PILOT_SHA256='a63bce9490f55438e7af5d16025e4dd20d86293cec12f1560d403ecb629cbe31')
 for node in TREE.body:
  if isinstance(node,ast.ImportFrom) and node.module=='waypoint_contract':
   for a in node.names:s[a.asname or a.name]=getattr(c,a.name)
 return s

def observation(age):
 end=10**12
 return dict(header_ns=end-int(age*1e9),validation_clock_begin_ns=end,validation_clock_end_ns=end,validation_steady_begin_ns=end,validation_steady_end_ns=end,receive_steady_seconds=1000-age,frame_id='base_link',ranges=723)
class Integration(unittest.TestCase):
 def test_new_helpers_are_explicitly_imported_and_freeze_runs_without_ROS(self):
  s=scope();names=['simulation_scan_latency_profile','validate_simulation_scan_latency_profile','operational_scan_age_issues','strict_simulation_scan_deviations','producer_scan_signed_age_bounds']
  for name in names:self.assertIn(name,s)
  s['frozen']=types.SimpleNamespace(__file__=str(GOAL/'pilot_scan_timing.py'));s['message_info_contract']=types.SimpleNamespace(__file__=str(GOAL/'message_info_contract.py'))
  fns=[n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name in ['file_sha','freeze_sources']];exec(compile(ast.fix_missing_locations(ast.Module(body=fns,type_ignores=[])),'<wrapper freeze integration>','exec'),s)
  with tempfile.TemporaryDirectory() as directory:
   manifest=s['freeze_sources'](Path(directory),.5);self.assertEqual(manifest['simulation_scan_latency_profile'],c.simulation_scan_latency_profile(.5))
 def run_check(self,own_age,producer_age=None,ready=True,cap=.5):
  s=scope();read=next(n for n in FROZEN.body if isinstance(n,ast.FunctionDef) and n.name=='read_ready');real=dict(json=json,time=time,math=math,hashlib=hashlib);exec(compile(ast.fix_missing_locations(ast.Module(body=[read],type_ignores=[])),'<unchanged frozen read_ready>','exec'),real);s['frozen']=types.SimpleNamespace(read_ready=real['read_ready'])
  s['waypoint_binding']=lambda *a:dict(waypoint_contract=dict(client_sources_sha256={}))
  s['ownership_metadata_lease_deviation']=lambda *a:None;s['validate_ownership']=lambda *a:None
  cls=next(n for n in TREE.body if isinstance(n,ast.ClassDef) and n.name=='WaypointPilot');fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='execution_ready');exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'<wrapper execution integration>','exec'),s)
  raw_scan=dict(stamp_ns=123,frame_id='base_link',ranges=723,confirmation_payload_valid=True,header_age_s=own_age,receive_age_s=own_age)
  snapshot=dict(last=dict(scan=raw_scan,pose=dict(raw_pose_values_finite=True),**{'/cmd_vel_safe':dict(vx=0.,vy=0.,wz=0.)}));strict=[] if own_age<=.3 else ['scan:missing_or_stale_receive','scan:invalid_header_age']
  obj=types.SimpleNamespace(simulation_scan_max_age_sec=cap,simulation_scan_sequence_fault=None,runtime_binding=None,native_binding=None,loaded_sources={},ownership_metadata_reported_check_ns=set(),owned_identity=None,goal_sent_ns=None,first_ready_read_failure=None,first_runtime_ready_read_failure=None,latest_ready={},latest_ready_sha=None,strict_scan_latency_first_fault=None,strict_scan_latency_count=0,strict_scan_latency_seen=set(),event_pub=types.SimpleNamespace(get_subscription_count=lambda:1),artifacts=[],events=[])
  obj.check_interruption=lambda:None;obj.input_ready=lambda **kw:(not strict,list(strict),snapshot);obj.emit=lambda kind,**kw:obj.events.append((kind,kw));obj.invalidate_artifact=lambda issues:obj.artifacts.extend(issues);obj.scan_timing_record_check=lambda stopped,issues,snap,*a:setattr(obj,'strict_record',(list(issues),snap))
  with tempfile.TemporaryDirectory() as directory:
   p=Path(directory)/'ready.json';now=time.time_ns();document=dict(ready=ready,reason=None if ready else 'Stale/future stamp /laser_scan_leveled',time_ns=now,valid_until_ns=now+500000000,variant='official',action_server='/navigate_to_pose',goal_tolerances=dict(xy=.15,yaw=.08,stateful=False),params_sha256='a'*64,build_receipt_sha256='b'*64,waypoint_ownership=dict(checked_unix_ns=now),simulation_scan_latency_profile=c.simulation_scan_latency_profile(cap),producer_scan_observation=observation(.1 if producer_age is None else producer_age));raw=json.dumps(document).encode();p.write_bytes(raw);obj.args=types.SimpleNamespace(execute=producer_age is not None,ready=p,variant='official',scan_aging_confirm_sec=0.);returned=s['execution_ready'](obj);self.assertEqual(p.read_bytes(),raw)
  return obj,returned,snapshot
 def test_wrapper_operational_499_retains_strict_snapshot_and_permanent_fault(self):
  obj,(valid,issues,snap),original=self.run_check(.499);self.assertTrue(valid);self.assertFalse(issues);self.assertIs(snap,original);self.assertIn('scan:invalid_header_age',obj.strict_record[0]);self.assertIn('scan:strict_300ms_latency_violation',obj.artifacts);self.assertEqual(obj.strict_scan_latency_first_fault['strict_timing_status'],'FAIL')
 def test_wrapper_producer399_consumer100_still_strict_failure(self):
  obj,(valid,issues,_),_=self.run_check(.1,.399);self.assertTrue(valid);self.assertFalse(issues);self.assertIn('scan:producer_strict_300ms_age_exceeded',obj.strict_record[0]);self.assertEqual(obj.strict_scan_latency_first_fault['deviations'][0]['source'],'producer_original_scan_bounds')
 def test_wrapper_never_accepts_false_producer_but_keeps_its_strict_bounds(self):
  obj,(valid,issues,_),_=self.run_check(.1,.399,ready=False);self.assertFalse(valid);self.assertTrue(any('launcher_not_ready' in x for x in issues));self.assertIn('strict_evidence_only',obj.strict_scan_latency_first_fault['producer_evidence_context'])
 def test_default_wrapper_does_not_filter_original300_issue(self):
  obj,(valid,issues,_),_=self.run_check(.399,cap=.3);self.assertFalse(valid);self.assertEqual(issues,['scan:missing_or_stale_receive','scan:invalid_header_age']);self.assertFalse(obj.artifacts)
 def passed_expression(self,valid=True,cancel=False,reason=None):
  cls=next(n for n in TREE.body if isinstance(n,ast.ClassDef) and n.name=='WaypointPilot');method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='run_batch');expr=next(n.value for n in ast.walk(method) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='passed' for t in n.targets));scope=dict(valid=valid,self=types.SimpleNamespace(cancel_requested=cancel,cancel_reason=reason),fields=dict(status=4,error_code=0,missed_waypoints=[]),ordered=dict(all_visited=True,violation=None),settled=True,final_within=True)
  return eval(compile(ast.fix_missing_locations(ast.Expression(expr)),'<actual functional pass expression>','eval'),scope)
 def test_postterminal_operational_fault_blocks_functional_pass(self):self.assertFalse(self.passed_expression(valid=False))
 def test_terminal4_cancel_race_blocks_functional_pass(self):
  self.assertFalse(self.passed_expression(cancel=True));self.assertFalse(self.passed_expression(reason='input_became_invalid'))
 def test_strict_scan_failure_alone_does_not_block_operational_success(self):self.assertTrue(self.passed_expression())
 def test_default_existing_confirmation_episode_reports_strict_fail(self):
  cls=next(n for n in TREE.body if isinstance(n,ast.ClassDef) and n.name=='WaypointPilot');method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='run_batch');kw=next(n for n in ast.walk(method) if isinstance(n,ast.keyword) and n.arg=='strict_scan_timing_status');self.assertEqual(eval(compile(ast.fix_missing_locations(ast.Expression(kw.value)),'<actual strict scan verdict>','eval'),dict(self=types.SimpleNamespace(strict_scan_latency_first_fault=None,scan_aging_episodes=[{}]))),'FAIL')

if __name__=='__main__':unittest.main()
