"""Exact RMW unresolved sentinels are zero-only discovery, never ownership proof."""
import unittest
from types import SimpleNamespace as N
from unittest import mock
import full_scene_mapping as o
import full_scene_mapping_contract as c
from test_graph_readiness_v19 import fixture,raised_health
from test_v4_core import tick_fixture
UNKNOWN=dict(name='_NODE_NAME_UNKNOWN_',namespace='_NODE_NAMESPACE_UNKNOWN_',type='sensor_msgs/msg/Imu')
def infos(rows):return [N(node_name=x['name'],node_namespace=x['namespace'],topic_type=x['type']) for x in rows]
def unresolved_fixture(rows=None):
 state,health,q,k,ctx,events,allowed=fixture(until=0.)
 state['node'].get_publishers_info_by_topic=lambda topic:infos([UNKNOWN] if rows is None else rows) if topic=='/livox/imu' else []
 return state,health,q,k,ctx,events,allowed
class Unresolved(unittest.TestCase):
 def test_actual_exact_unique_unknown_is_not_accepted_and_needs_new_full10s_known(self):
  state,health,q,k,ctx,events,allowed=unresolved_fixture();error=raised_health(health)
  self.assertTrue(error.graph_unresolved_identity);self.assertFalse(error.graph_missing_only);self.assertEqual(error.graph_observation['missing'],[]);self.assertTrue(allowed(error));self.assertFalse(q.armed)
  detail=error.graph_observation['unresolved_publishers'][0];self.assertEqual(detail['observed'],[UNKNOWN]);self.assertEqual(detail['expected'][0]['name'],'matrix_sensor_contract_adapter');self.assertIn('not_expected_identity',detail['resolution_status'])
  begin=k.monotonic();first_known=[];known=dict(name='matrix_sensor_contract_adapter',namespace='/',type='sensor_msgs/msg/Imu')
  def publishers(topic):
   if topic!='/livox/imu':return []
   if k.monotonic()-begin<.3:return infos([UNKNOWN])
   if not first_known:first_known.append(k.monotonic())
   return infos([known])
  state['node'].get_publishers_info_by_topic=publishers
  def tick():k.sleep(.04);return health()[:2]
  o.qualify_prearm_zero(q,tick,N(command=(0.,0.,0.),send=lambda:None),state['model'],lambda *a,**kw:None,allowed,k,arming_allowed=lambda:True)
  self.assertTrue(q.armed);self.assertGreaterEqual(k.monotonic()-first_known[0],10.);self.assertGreaterEqual(min(q.spans().values()),10_000_000_000)
 def test_partial_wrong_type_foreign_bare_or_duplicate_never_wait(self):
  cases=[dict(UNKNOWN,name='matrix_sensor_contract_adapter'),dict(UNKNOWN,namespace='/'),dict(UNKNOWN,type='sensor_msgs/msg/Image'),dict(UNKNOWN,name='foreign'),dict(UNKNOWN,name='_CREATED_BY_BARE_DDS_APP_',namespace='_CREATED_BY_BARE_DDS_APP_'),dict(UNKNOWN,name='_NODE_NAME_UNKNOWN_suffix_')]
  for rows in [[v] for v in cases]+[[UNKNOWN,UNKNOWN],[UNKNOWN,dict(name='matrix_sensor_contract_adapter',namespace='/',type='sensor_msgs/msg/Imu')]]:
   state,health,q,k,ctx,events,allowed=unresolved_fixture(rows);error=raised_health(health)
   self.assertFalse(error.graph_unresolved_identity);self.assertFalse(error.graph_missing_only);self.assertFalse(allowed(error));self.assertFalse(q.armed)
 def test_unknown_cannot_mask_later_foreign_or_duplicate_guarded_node(self):
  state,health,q,k,ctx,events,allowed=unresolved_fixture()
  state['release']['expected_sensor_publishers']['/front_lidar/imu']=[dict(name='ue_zenoh_bridge',namespace='/',type='sensor_msgs/msg/Imu')]
  state['node'].get_publishers_info_by_topic=lambda t:infos([UNKNOWN]) if t=='/livox/imu' else infos([dict(name='foreign',namespace='/',type='sensor_msgs/msg/Imu')]) if t=='/front_lidar/imu' else []
  error=raised_health(health);self.assertTrue(error.graph_observation['unresolved_publishers']);self.assertFalse(error.graph_unresolved_identity);self.assertFalse(allowed(error))
  state,health,q,k,ctx,events,allowed=unresolved_fixture();state['node'].get_node_names_and_namespaces=lambda:[('fixture_recorder','/'),('matrix_sensor_contract_adapter','/')]*2
  error=raised_health(health);self.assertFalse(error.graph_unresolved_identity);self.assertFalse(allowed(error))
 def test_armed_nonzero_or_fault_context_still_rejects_and_actual_tick_sends_none(self):
  for bad in ['armed','ever_armed','requested','command','request','fault','deadline']:
   state,health,q,k,ctx,events,allowed=unresolved_fixture();error=raised_health(health)
   if bad=='armed':q.armed=True
   elif bad=='ever_armed':q.ever_armed=True
   elif bad=='requested':q.ever_nonzero_requested=True
   elif bad=='command':ctx['command']=[.4,0.,0.]
   elif bad=='request':ctx['request']=[.4,0.,0.]
   elif bad=='fault':q.fault={'reason':'fixture'}
   else:k.sleep(40.)
   self.assertFalse(allowed(error),bad)
  for armed in [False,True]:
   state,health,q,k,ctx,events,allowed=unresolved_fixture();q.armed=q.ever_armed=armed
   ns,p,rows=tick_fixture();ns['relay'].prearm_qualification=q;ns['health']=health
   with mock.patch.object(c,'guard_hits',return_value={}),self.assertRaises(c.Hold):ns['tick'](.4,0.)
   self.assertFalse(rows)
 def test_permanent_unknown_cannot_extend40s_or_qualify(self):
  state,health,q,k,ctx,events,allowed=unresolved_fixture();begin=k.monotonic();deadline=q.deadline
  def tick():k.sleep(.04);return health()[:2]
  with self.assertRaises(c.Hold):o.qualify_prearm_zero(q,tick,N(command=(0.,0.,0.),send=lambda:None),state['model'],lambda *a,**kw:None,allowed,k,arming_allowed=lambda:True)
  self.assertFalse(q.armed);self.assertEqual(q.deadline,deadline);self.assertLess(k.monotonic()-begin,40.1)
if __name__=='__main__':unittest.main(verbosity=2)
