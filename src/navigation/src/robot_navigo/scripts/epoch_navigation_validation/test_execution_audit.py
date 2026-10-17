import unittest
from execution_audit import audit_execution

def message(v):return dict(token={'id':1},controller_session_id='c',command_sequence=1,velocity=dict(linear=dict(x=v[0],y=v[1]),angular=dict(z=v[2])))
def row(event,data,t=100000000):return dict(event=event,data=data,monotonic_ns=t)
class ExecutionTests(unittest.TestCase):
 def test_empty_trace_not_pass(self):self.assertFalse(audit_execution([])['execution_pass'])
 def test_zero_stop_trace_is_valid(self):
  rows=[row('/cmd_vel_epoch_raw',message([0,0,0])),row('/cmd_vel_safe',dict(velocity=[0,0,0]))]
  self.assertTrue(audit_execution(rows)['execution_pass'])
 def test_mixed_axis_from_smoother_detected(self):
  rows=[row('/cmd_vel_epoch_raw',message([.1,0,0])),row('/cmd_vel_epoch',message([.075,0,.025])),row('/cmd_vel_safe',dict(velocity=[.075,0,.025]))]
  r=audit_execution(rows);self.assertEqual(r['raw_zero_axis_leak_count'],1);self.assertFalse(r['execution_pass'])
 def test_regular_acceleration_is_not_axis_leak(self):
  rows=[row('/cmd_vel_epoch_raw',message([.1,0,0])),row('/cmd_vel_epoch',message([.075,0,0])),row('/cmd_vel_safe',dict(velocity=[.075,0,0]))]
  r=audit_execution(rows);self.assertEqual(r['changed_nonzero_relay_count'],1);self.assertTrue(r['execution_pass'])
 def test_missing_source_rejected(self):
  rows=[row('/cmd_vel_safe',dict(velocity=[.075,0,0]))];self.assertFalse(audit_execution(rows)['execution_pass'])
 def test_forged_braking_cannot_accelerate_old_axis(self):
  raw=message([.1,0,0]);raw['source_motion_phase']=2
  relay=message([0,0,.1]);relay.update(source_motion_phase=2,transition_braking=True)
  rows=[row('/cmd_vel_epoch_raw',raw),row('/cmd_vel_epoch',relay),row('/cmd_vel_safe',dict(velocity=[0,0,.1]))]
  self.assertFalse(audit_execution(rows)['execution_pass'])
 def test_source_phase_must_survive_relay(self):
  raw=message([.1,0,0]);raw['source_motion_phase']=2
  relay=message([.075,0,0]);relay['source_motion_phase']=0
  rows=[row('/cmd_vel_epoch_raw',raw),row('/cmd_vel_epoch',relay),row('/cmd_vel_safe',dict(velocity=[.075,0,0]))]
  self.assertFalse(audit_execution(rows)['execution_pass'])
if __name__=='__main__':unittest.main()
