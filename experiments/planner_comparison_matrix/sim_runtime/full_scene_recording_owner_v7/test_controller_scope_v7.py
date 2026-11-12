import copy,time,unittest
import mapping_result_adapter as adapter
class ScopeTests(unittest.TestCase):
 def fixture(self,scope,axes):
  now,steady=time.time_ns(),time.monotonic_ns();identity={'pid':1};c=dict(scope=scope,run_id='r',route_sha256='q',oracle_sha256='o',sources_sha256={'s':'h'},output_directory='/test',case_id='c',recording_run_id='b')
  bind={k:c[k] for k in ['route_sha256','oracle_sha256','sources_sha256','output_directory','case_id','recording_run_id']};bind.update(root_release_sha256='rel',recorder_ready_sha256='ready')
  v=dict(scope=scope,mode=adapter.MODE,run_id='r',owner_process_identity=identity,runtime_binding=bind,formal_navigation_acceptance=False,classification=adapter.SUCCESS,stop_verified=True,relay_closed=True,relay_close_errors=[],relay_returncode=0,writer=dict(evidence_complete=True,dropped=0,worker_error=None,writer_alive=False),final_stop_observation=dict(command=[0.]*axes,continuous_seconds=.3,checked_steady_ns=steady,checked_unix_ns=now,truth=dict(xyz=[0.,0.,.33],q=[0.,0.,0.,1.],twist=[0.]*6,stamp_ns=now-10_000_000,received_steady=steady/1e9-.01,frame_id='map',child_frame_id='BASE_LINK')))
  return v,(c,identity,'rel','ready',steady-1_000_000_000)
 def test_known_scope_exact_stop_vector(self):
  for s,a in adapter.SCOPE_AXES.items():
   v,args=self.fixture(s,a);self.assertTrue(adapter.assess(v,*args)[0]);v['final_stop_observation']['command'].append(0.);self.assertFalse(adapter.assess(v,*args)[0])
 def test_nonzero_lateral_rejected(self):
  v,args=self.fixture('MAPPING_ACQUISITION_LATERAL_V4_ONLY',3);v['final_stop_observation']['command'][2]=.4;self.assertFalse(adapter.assess(v,*args)[0])
 def test_bool_nonfinite_and_missing_axis_rejected(self):
  for cmd in [[False,0.,0.],[0.,float('nan'),0.],[0.,0.]]:
   v,args=self.fixture('MAPPING_ACQUISITION_LATERAL_RECT_V5_ONLY',3);v['final_stop_observation']['command']=cmd;self.assertFalse(adapter.assess(v,*args)[0])
 def test_unknown_scope_and_cross_scope_rejected(self):
  with self.assertRaises(ValueError):adapter.expected_contract({'scope':'NOT_MAPPING'})
  v,args=self.fixture('MAPPING_ACQUISITION_LATERAL_V4_ONLY',3);v['scope']='MAPPING_ACQUISITION_LATERAL_RECT_V5_ONLY';self.assertFalse(adapter.assess(v,*args)[0])
 def test_identity_and_provenance_still_required(self):
  v,args=self.fixture('MAPPING_ACQUISITION_LATERAL_V4_ONLY',3);v['runtime_binding']['route_sha256']='other';self.assertFalse(adapter.assess(v,*args)[0])
if __name__=='__main__':unittest.main()
