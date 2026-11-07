import ast,copy,hashlib,importlib.util,json,math,sys,time,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('candidate_scan_contract',ROOT/'goal_pilot_source/waypoint_contract.py')
c=importlib.util.module_from_spec(spec);sys.modules[spec.name]=c;spec.loader.exec_module(c)
def scan(age=.499,recv=.499):return {'last':{'scan':dict(stamp_ns=1000000000,frame_id='base_link',ranges=723,confirmation_payload_valid=True,header_age_s=age,receive_age_s=recv)}}
def producer(age=.399):
 end=10**12;stamp=end-int(age*1e9)
 return dict(simulation_scan_latency_profile=c.simulation_scan_latency_profile(.5),producer_scan_observation=dict(header_ns=stamp,validation_clock_begin_ns=end,validation_clock_end_ns=end,validation_steady_begin_ns=end,validation_steady_end_ns=end,receive_steady_seconds=1000-age,frame_id='base_link',ranges=723,topic='/laser_scan_leveled'))
class ProfileTests(unittest.TestCase):
 def test_exact_profile(self):
  self.assertEqual(c.simulation_scan_latency_profile(.5),dict(schema='matrix.simulation_scan_latency.v1',mode='simulation_500ms',operational_max_age_sec=.5,strict_max_age_sec=.3,future_tolerance_sec=.05,original_header_and_receive_timestamps=True,applies_only_to='scan_header_and_receive_age'))
 def test_legacy_default_unchanged(self):
  issues=['scan:invalid_header_age'];self.assertEqual(c.operational_scan_age_issues(issues,scan(),.3),issues);c.validate_simulation_scan_latency_profile({},.3)
 def test_499_scan_only_operational(self):
  s=scan();original=copy.deepcopy(s);self.assertEqual(c.operational_scan_age_issues(['scan:missing_or_stale_receive','scan:invalid_header_age'],s,.5),[]);self.assertEqual(s,original)
 def test_501_header_rejected(self):self.assertTrue(c.operational_scan_age_issues(['scan:invalid_header_age'],scan(.501,.1),.5))
 def test_501_receipt_rejected(self):self.assertTrue(c.operational_scan_age_issues(['scan:missing_or_stale_receive'],scan(.1,.501),.5))
 def test_future_rejected(self):self.assertTrue(c.operational_scan_age_issues(['scan:invalid_header_age'],scan(-.0501,.1),.5))
 def test_nan_and_payload_rejected(self):
  for age in [math.nan,math.inf]:self.assertTrue(c.operational_scan_age_issues([],scan(age,.1),.5))
  s=scan();s['last']['scan']['confirmation_payload_valid']=False;self.assertTrue(c.operational_scan_age_issues([],s,.5))
 def test_profile_mismatch_or_missing_rejected(self):
  for actual in [{},{'simulation_scan_latency_profile':c.simulation_scan_latency_profile(.3)}]:
   with self.assertRaises(ValueError):c.validate_simulation_scan_latency_profile(actual,.5)
 def test_other_faults_never_filtered(self):
  issues=['pose:invalid_header_age','readiness:launcher_lease_missing_expired_or_future','scan:source_stamp_not_strictly_increasing','scan:invalid_header_age'];self.assertEqual(c.operational_scan_age_issues(issues,scan(),.5),issues[:-1])
 def test_producer_strict_old_is_retained_even_consumer_fresh(self):
  r=producer();s=scan(.1,.1);r_before=copy.deepcopy(r);d=c.strict_simulation_scan_deviations(s,r);self.assertEqual(d[0]['source'],'producer_original_scan_bounds');self.assertGreater(d[0]['header_age_bounds_s'][0],.3);self.assertEqual(r,r_before)
 def test_consumer_strict_old_is_retained(self):self.assertEqual(c.strict_simulation_scan_deviations(scan(),producer(.1))[0]['source'],'consumer_original_scan')
 def test_false_producer_never_rewritten_or_accepted(self):
  tree=ast.parse((ROOT/'goal_pilot_source/pilot_scan_timing.py').read_text());function=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='read_ready');scope=dict(json=json,time=time,math=math,hashlib=hashlib);exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),'<unchanged frozen validator>','exec'),scope)
  r=producer();r.update(ready=False,reason='Stale/future stamp /laser_scan_leveled');raw=json.dumps(r).encode()
  class Raw:
   def read_bytes(self):return raw
  with self.assertRaisesRegex(ValueError,'launcher_not_ready'):scope['read_ready'](Raw(),'official',None)
  self.assertFalse(json.loads(raw)['ready']);self.assertEqual(c.operational_scan_age_issues(['readiness:launcher_not_ready:Stale/future stamp /laser_scan_leveled'],scan(),.5),['readiness:launcher_not_ready:Stale/future stamp /laser_scan_leveled'])
 def test_original_frozen_bytes_untouched(self):self.assertEqual(hashlib.sha256((ROOT/'goal_pilot_source/pilot_scan_timing.py').read_bytes()).hexdigest(),'a63bce9490f55438e7af5d16025e4dd20d86293cec12f1560d403ecb629cbe31')
if __name__=='__main__':unittest.main()
