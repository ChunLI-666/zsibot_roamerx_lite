"""Actual health/prearm/tick pure fixtures; no ROS/process/socket/runtime."""
import copy,unittest
from types import SimpleNamespace as N
from unittest import mock
import full_scene_mapping as o
import full_scene_mapping_contract as c
from test_lifecycle_v7 import health_fixture
from test_startup_v6 import actual_function,install
from test_prearm_v12 import cdr
from test_v4_core import tick_fixture

def fixture(missing='publisher',until=.3):
 state,health=health_fixture();k=state['time'];begin=k.monotonic();events=[]
 context=dict(command=[0.,0.,0.],request=[0.,0.,0.],latest_inputs={})
 q=o.PrearmImuQualification(lambda:context,k);state.update(raw_imu_gate=q,PrearmImuQualification=o.PrearmImuQualification,emit=lambda name,**fields:events.append((name,fields)))
 pub=dict(name='matrix_sensor_contract_adapter',namespace='/',type='sensor_msgs/msg/Imu')
 state['release']['expected_sensor_publishers']={'/livox/imu':[pub]}
 def waiting():return k.monotonic()-begin<until
 state['node'].get_publishers_info_by_topic=lambda topic:[] if topic!='/livox/imu' or (missing=='publisher' and waiting()) else [N(node_name=pub['name'],node_namespace='/',topic_type=pub['type'])]
 state['node'].get_node_names_and_namespaces=lambda:[('fixture_recorder','/')]+([] if missing=='node' and waiting() else [(pub['name'],'/')])
 state['node'].get_subscriptions_info_by_topic=lambda topic:[N(node_name='other_legitimate_reader',node_namespace='/')]+([] if missing=='subscription' and waiting() else [N(node_name='fixture_recorder',node_namespace='/')])
 previous_spin=state['callbacks'].spin
 def spin():
  previous_spin();q.inspect(cdr(k.time_ns()),k.time_ns(),k.monotonic_ns());state['audit']['checked_steady']=k.monotonic()
 state['callbacks'].spin=spin
 allowed=install(actual_function('prearm_waiting_allowed'),state)
 return state,health,q,k,context,events,allowed

def raised_health(health):
 try:health()
 except c.Hold as error:return error
 raise AssertionError('fixture must reject')

class StartupGraph(unittest.TestCase):
 def test_missing_publisher_node_or_recorder_subscription_requalifies_full10s(self):
  for missing in ['publisher','node','subscription']:
   state,health,q,k,ctx,events,allowed=fixture(missing);begin=k.monotonic();zero=[]
   relay=N(command=(0.,0.,0.),send=lambda:zero.append(k.monotonic()))
   def tick():k.sleep(.04);return health()[:2]
   o.qualify_prearm_zero(q,tick,relay,state['model'],lambda *a,**fields:None,allowed,k,arming_allowed=lambda:True)
   self.assertTrue(q.armed);self.assertGreaterEqual(k.monotonic()-begin,10.25);self.assertGreaterEqual(min(q.spans().values()),10_000_000_000)
   self.assertTrue(zero);self.assertEqual(q.epoch,0)
   snapshots=[v['graph'] for n,v in events if n=='graph_readiness_not_qualified'];self.assertTrue(snapshots);self.assertTrue(all(v['missing_only'] for v in snapshots))
   self.assertTrue(all('expected' in item and ('observed' in item or 'observed_count' in item) for v in snapshots for item in v['missing']))
 def test_nonempty_wrong_duplicate_source_or_duplicate_node_are_immediate(self):
  for kind in ['wrong','duplicate_publisher','duplicate_node','duplicate_subscription']:
   state,health,q,k,ctx,events,allowed=fixture(until=100.)
   good=N(node_name='matrix_sensor_contract_adapter',node_namespace='/',topic_type='sensor_msgs/msg/Imu')
   if kind=='wrong':state['node'].get_publishers_info_by_topic=lambda topic:[N(node_name='foreign',node_namespace='/',topic_type='sensor_msgs/msg/Imu')] if topic=='/livox/imu' else []
   if kind=='duplicate_publisher':state['node'].get_publishers_info_by_topic=lambda topic:[good,good] if topic=='/livox/imu' else []
   if kind=='duplicate_node':state['node'].get_node_names_and_namespaces=lambda:[('fixture_recorder','/'),('matrix_sensor_contract_adapter','/'),('matrix_sensor_contract_adapter','/')]
   if kind=='duplicate_subscription':state['node'].get_subscriptions_info_by_topic=lambda topic:[N(node_name='fixture_recorder',node_namespace='/')]*2
   exc=raised_health(health);self.assertFalse(exc.graph_missing_only);self.assertFalse(allowed(exc));self.assertFalse(q.armed);self.assertTrue(events)
 def test_missing_does_not_mask_later_wrong_or_forbidden_Nav(self):
  state,health,q,k,ctx,events,allowed=fixture(until=100.)
  state['release']['expected_sensor_publishers']['/front_lidar/imu']=[dict(name='ue_zenoh_bridge',namespace='/',type='sensor_msgs/msg/Imu')]
  state['node'].get_publishers_info_by_topic=lambda topic:[N(node_name='wrong',node_namespace='/',topic_type='sensor_msgs/msg/Imu')] if topic=='/front_lidar/imu' else []
  error=raised_health(health);self.assertFalse(error.graph_missing_only);self.assertFalse(allowed(error));self.assertTrue(error.graph_observation['missing'])
  for kind in ['publisher','action']:
   state,health,q,k,ctx,events,allowed=fixture(until=100.)
   if kind=='publisher':state['node'].get_publishers_info_by_topic=lambda topic:[N(node_name='nav',node_namespace='/',topic_type='geometry_msgs/msg/Twist')] if topic=='/cmd_vel' else []
   else:state['node'].get_service_names_and_types=lambda:[('/navigate_to_pose/_action/send_goal',['nav2_msgs/action/NavigateToPose_SendGoal'])]
   error=raised_health(health);self.assertFalse(error.graph_missing_only);self.assertFalse(allowed(error))
 def test_missing_after_arm_nonzero_context_or_deadline_never_waits(self):
  for bad in ['armed','ever_armed','requested','command','request','fault','deadline','writer','audit']:
   state,health,q,k,ctx,events,allowed=fixture(until=100.);error=raised_health(health);self.assertTrue(allowed(error))
   if bad=='armed':q.armed=True
   elif bad=='ever_armed':q.ever_armed=True
   elif bad=='requested':q.ever_nonzero_requested=True
   elif bad=='command':ctx['command']=[.4,0.,0.]
   elif bad=='request':ctx['request']=[.4,0.,0.]
   elif bad=='fault':q.fault={'reason':'fixture'}
   elif bad=='deadline':k.sleep(40.)
   elif bad=='writer':state['writer'].summary=lambda:dict(dropped=1,worker_error=None)
   elif bad=='audit':state['audit']['first_violation']={'reason':'fixture'}
   self.assertFalse(allowed(error),bad)
 def test_actual_tick_cannot_send_positive_before_arm_or_with_missing_after_arm(self):
  for armed in [False,True]:
   state,health,q,k,ctx,events,allowed=fixture(until=100.);q.armed=q.ever_armed=armed
   ns,policy,rows=tick_fixture();ns['relay'].prearm_qualification=q;ns['health']=health
   with mock.patch.object(c,'guard_hits',return_value={}),self.assertRaises(c.Hold):ns['tick'](.4,0.)
   self.assertFalse(rows);self.assertTrue(q.ever_nonzero_requested)
 def test_missing_forever_does_not_extend40s_or_arm(self):
  state,health,q,k,ctx,events,allowed=fixture(until=100.);begin=k.monotonic();deadline=q.deadline
  def tick():k.sleep(.04);return health()[:2]
  with self.assertRaises(c.Hold):o.qualify_prearm_zero(q,tick,N(command=(0.,0.,0.),send=lambda:None),state['model'],lambda *a,**kw:None,allowed,k,arming_allowed=lambda:True)
  self.assertFalse(q.armed);self.assertEqual(q.deadline,deadline);self.assertLess(k.monotonic()-begin,40.1)
 def test_evidence_rejection_and_other_health_errors_are_not_graph_missing(self):
  state,health,q,k,ctx,events,allowed=fixture(until=100.)
  state['emit']=lambda *a,**kw:(_ for _ in ()).throw(c.Hold('bounded event writer rejected evidence'))
  error=raised_health(health);self.assertFalse(getattr(error,'graph_readiness',False));self.assertFalse(allowed(error))
  state,health,q,k,ctx,events,allowed=fixture(until=100.);state['imu_gate_fault']={'reason':'actual original fault'}
  error=raised_health(health);self.assertFalse(getattr(error,'graph_readiness',False));self.assertFalse(allowed(error))
if __name__=='__main__':unittest.main(verbosity=2)
