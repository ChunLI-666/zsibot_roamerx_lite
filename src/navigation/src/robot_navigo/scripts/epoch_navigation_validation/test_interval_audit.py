"""Boundary and authority negative controls for the new interval scorer."""
import copy
import math
import unittest
from audit_events import audit,audit_intervals
from execution_audit import audit_execution
from publication_clock import timeline
from test_publication_clock import observed_trace


class IntervalAuditTests(unittest.TestCase):
    def test_definite_revocation_cannot_be_authorized_by_future_active(self):
        rows=observed_trace()
        execution=next(r for r in rows if r['event']=='/nav_epoch/execution')
        execution['data']['active']=False
        later=copy.deepcopy(execution);later['data']['active']=True
        later['observation']['dds']['source_timestamp']+=20_000_000
        later['observation']['dds']['received_timestamp']+=20_000_000
        rows.append(later)
        self.assertEqual(audit(rows)['assessment'],'FAIL')
    def test_definite_localization_loss_fails(self):
        rows=observed_trace()
        next(r for r in rows if r['event']=='localization_publication')['data']['health']=3
        self.assertEqual(audit(rows)['assessment'],'FAIL')
    def test_loss_without_new_normal_publication_remains_fail(self):
        rows=observed_trace()
        # The last NORMAL publication predates a loss that is now >100ms old.
        loc=next(r for r in rows if r['event']=='localization_publication')
        loc['publication_bracket']=[700_000_000,700_100_000]
        rows.append(dict(event='localization_transition',monotonic_ns=800_000_000,data=dict(health=3)))
        self.assertTrue(any(x['reason']=='safe output persists after loss transition' for x in audit(rows)['failures']))
    def test_50ms_crossing_cannot_be_midpoint_pass(self):
        projected,_=timeline(observed_trace())
        safe=next(r for r in projected if r['event']=='/cmd_vel_safe')
        relay=next(r for r in projected if r['event']=='/cmd_vel_epoch')
        relay['_dds_wall_ns']=safe['_dds_wall_ns']-50_000_000
        safe['_offset_drift_ns']=relay['_offset_drift_ns']=100
        result=audit_intervals(projected)
        self.assertEqual(result['assessment'],'UNSCORABLE')
        self.assertEqual(result['observer_matching_window_ms'],50)
    def test_expired_execution_ttl_is_300ms_not_old_callback_margin(self):
        rows=observed_trace()
        next(r for r in rows if r['event']=='/nav_epoch/execution')['data']['source_steady_time_ns']-=305_000_000
        self.assertEqual(audit(rows)['assessment'],'FAIL')
    def test_phase_uses_publication_matching_and_preserves_callback_evidence(self):
        rows=observed_trace();original=copy.deepcopy(rows)
        result=audit_execution(rows)
        self.assertTrue(result['execution_pass'])
        self.assertIn('callback_audit',result)
        self.assertEqual(rows,original)
    def test_all_dds_timestamps_invalid_is_unscorable_not_crash(self):
        rows=observed_trace()
        for row in rows:
            if 'observation' in row:row['observation']['dds']['source_timestamp']=None
        result=audit(rows)
        self.assertEqual(result['assessment'],'UNSCORABLE')
        self.assertEqual(result['definite_failure_count'],0)
        self.assertIn('callback_audit',result)
    def test_definite_failure_takes_precedence_over_uncertainty(self):
        rows=observed_trace()
        for row in rows:
            if row['event'] in ('/cmd_vel_epoch_raw','/cmd_vel_epoch'):
                row['data']['source_steady_time_ns']-=301_000_000
        loc=next(r for r in rows if r['event']=='localization_publication')
        loc['publication_bracket']=[1_000_500_000,1_001_500_000]
        result=audit(rows)
        self.assertGreater(result['definite_failure_count'],0)
        self.assertGreater(result['unscorable_count'],0)
        self.assertEqual(result['assessment'],'FAIL')
    def test_all_nan_safe_is_definite_failure_even_with_invalid_clock(self):
        for invalid_clock in (False,True):
            rows=observed_trace()
            safe=next(r for r in rows if r['event']=='/cmd_vel_safe')
            safe['data']['velocity']=[math.nan]*3
            if invalid_clock:safe['observation']['dds']['source_timestamp']=None
            with self.subTest(invalid_clock=invalid_clock):
                result=audit(rows)
                self.assertEqual(result['assessment'],'FAIL')
                self.assertTrue(any(x['reason']=='nonfinite safe velocity' for x in result['failures']))
    def test_local_publish_width_cannot_hide_confirmed_loss(self):
        rows=observed_trace();loc=next(r for r in rows if r['event']=='localization_publication')
        loc['publication_bracket']=[500_000_000,900_000_000]
        loc['data']['health']=3
        self.assertEqual(audit(rows)['assessment'],'FAIL')


if __name__=='__main__':unittest.main()
