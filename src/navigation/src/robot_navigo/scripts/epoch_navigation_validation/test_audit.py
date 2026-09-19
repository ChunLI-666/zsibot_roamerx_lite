"""Independent observer negative controls, including the reviewed false-pass trace."""
import copy
import math
import unittest
from audit_events import audit,nonzero


def valid_trace():
    t=1_000_000_000
    token={'localization':{'epoch':1},'gate_session_id':'gate'}
    command={'token':token,'controller_session_id':'controller','command_sequence':7,
             'boot_id':'boot','source_steady_time_ns':t-10,'max_age_ms':300,
             'velocity':{'linear':{'x':.1,'y':0},'angular':{'z':0}}}
    permission={'active':True,'token':token,'boot_id':'boot','source_steady_time_ns':t-10,'controller_session_id':'controller'}
    events=[('fixture_start',{'boot_id':'boot'}),('localization_publication',{'identity':token['localization'],'health':1}),
            ('/nav_epoch/gate_session',{'data':'gate'}),('/nav_epoch/intent',copy.deepcopy(permission)),
            ('/nav_epoch/execution',copy.deepcopy(permission)),('/cmd_vel_epoch_raw',copy.deepcopy(command)),
            ('/cmd_vel_epoch',copy.deepcopy(command)),('/cmd_vel_safe',{'velocity':[.1,0,0]})]
    return [{'event':event,'monotonic_ns':t-8+i,'data':data} for i,(event,data) in enumerate(events)]


class AuditTest(unittest.TestCase):
    def test_valid_observation_passes(self):
        self.assertFalse(audit(valid_trace())['failures'])
    def test_empty_trace_is_not_integration_pass(self):
        self.assertTrue(audit([])['failures'])
    def test_nan_is_a_violation(self):
        row={'event':'/cmd_vel_safe','monotonic_ns':1,'data':{'velocity':[math.nan,0.,0.]}}
        self.assertTrue(nonzero(row['data']['velocity']))
        self.assertTrue(any(item['reason']=='nonfinite safe velocity' for item in audit([row])['failures']))
    def test_recent_mismatched_inactive_authority_cannot_bypass(self):
        rows=valid_trace()
        for row in rows:
            if row['event'] in ('/nav_epoch/intent','/nav_epoch/execution'):
                row['data'].update(active=False,token={'wrong':'token'},controller_session_id='wrong')
        rows=[r for r in rows if r['event'] not in ('/cmd_vel_epoch_raw','localization_publication','/nav_epoch/gate_session')]
        self.assertGreaterEqual(len(audit(rows)['failures']),5)
    def test_every_missing_authority_is_rejected(self):
        for topic in ('/cmd_vel_epoch_raw','localization_publication','/nav_epoch/gate_session','/nav_epoch/intent','/nav_epoch/execution'):
            with self.subTest(topic=topic):self.assertTrue(audit([r for r in valid_trace() if r['event']!=topic])['failures'])
    def test_far_future_raw_cannot_prove_present_command(self):
        rows=valid_trace();next(r for r in rows if r['event']=='/cmd_vel_epoch_raw')['monotonic_ns']+=100_000_000
        rows.sort(key=lambda r:r['monotonic_ns'])
        self.assertTrue(audit(rows)['failures'])
    def test_new_normal_publication_supersedes_old_loss(self):
        rows=valid_trace();rows.insert(1,{'event':'localization_transition','monotonic_ns':100_000_000,'data':{'health':3}})
        self.assertFalse(audit(rows)['failures'])
    def test_loss_transition_still_blocks_nonzero(self):
        rows=valid_trace();rows.insert(1,{'event':'localization_transition','monotonic_ns':100_000_000,'data':{'health':3}})
        next(r for r in rows if r['event']=='localization_publication')['data']['health']=3
        self.assertTrue(audit(rows)['failures'])
    def test_execution_expiry_is_stricter_than_intent(self):
        rows=valid_trace()
        next(r for r in rows if r['event']=='/nav_epoch/execution')['data']['source_steady_time_ns']-=350_000_000
        self.assertTrue(audit(rows)['failures'])
    def test_future_and_wrong_boot_authorities_are_rejected(self):
        for topic in ('/nav_epoch/intent','/nav_epoch/execution'):
            for field,value in [('source_steady_time_ns',11_000_000_000),('boot_id','wrong')]:
                rows=valid_trace();next(r for r in rows if r['event']==topic)['data'][field]=value
                with self.subTest(topic=topic,field=field):self.assertTrue(audit(rows)['failures'])
    def test_relay_cannot_refresh_raw_source(self):
        rows=valid_trace()
        next(r for r in rows if r['event']=='/cmd_vel_epoch')['data']['source_steady_time_ns']+=1
        self.assertTrue(audit(rows)['failures'])
    def test_execution_must_match_controller_session(self):
        rows=valid_trace()
        next(r for r in rows if r['event']=='/nav_epoch/execution')['data']['controller_session_id']='wrong'
        self.assertTrue(audit(rows)['failures'])


if __name__=='__main__':unittest.main()
