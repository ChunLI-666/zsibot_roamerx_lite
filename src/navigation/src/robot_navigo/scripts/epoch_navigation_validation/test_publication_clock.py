"""DDS clock evidence distinguishes observer backlog from expired publication."""
import copy
import unittest
from audit_events import audit
from publication_clock import WIRE_TOPICS,timeline,capture,bounds,difference,within
from test_audit import valid_trace


def observed_trace():
    rows=valid_trace();offset=10**15
    for row in rows:
        if row['event']=='localization_publication':row['publication_bracket']=[row['monotonic_ns'],row['monotonic_ns']]
        if row['event'] not in WIRE_TOPICS:continue
        source=row['monotonic_ns']+1_000_000;callback=source+350_000_000
        row['monotonic_ns']=callback+200
        row['observation']={'callback_clock':{'monotonic_before':callback,'wall':callback+offset+50,'monotonic_after':callback+100},
            'dds':{'source_timestamp':source+offset,'received_timestamp':source+offset+1000}}
    return rows


class PublicationClockTest(unittest.TestCase):
    def test_backlog_not_expiry_and_original_evidence_preserved(self):
        rows=observed_trace();original=copy.deepcopy(rows);result=audit(rows)
        self.assertFalse(result['failures']);self.assertTrue(result['callback_audit']['failures'])
        self.assertGreater(result['publication_clock']['max_observer_queue_ms'],349.)
        self.assertEqual(result['observer_matching_window_ms'],50)
        self.assertEqual(rows,original)
    def test_actual_expired_source_still_fails(self):
        rows=observed_trace()
        for row in rows:
            if row['event'] in ('/cmd_vel_epoch_raw','/cmd_vel_epoch'):
                row['data']['source_steady_time_ns']-=301_000_000
        self.assertTrue(any(f['reason']=='safe command source age invalid' for f in audit(rows)['failures']))
    def test_invalid_clock_is_unscorable(self):
        for fault in ('missing','zero','invalid','jump','reversed'):
            rows=observed_trace();row=next(r for r in rows if r['event']=='/cmd_vel_safe')
            if fault=='missing':del row['observation']
            elif fault=='zero':row['observation']['dds']['source_timestamp']=0
            elif fault=='invalid':row['observation']['callback_clock']['monotonic_after']=0
            elif fault=='jump':row['observation']['callback_clock']['wall']+=2_000_000
            else:row['observation']['dds']['received_timestamp']=row['observation']['dds']['source_timestamp']-1
            with self.subTest(fault=fault):
                result=audit(rows)
                self.assertFalse(result['publication_clock']['available'])
                self.assertTrue(any(f['reason']=='DDS publication time is unscorable' for f in result['failures']))
    def test_ttl_uncertainty_is_not_a_pass(self):
        rows=observed_trace()
        safe=next(r for r in rows if r['event']=='/cmd_vel_safe')
        publication=safe['observation']['dds']['source_timestamp']-10**15
        for row in rows:
            if row['event'] in ('/cmd_vel_epoch_raw','/cmd_vel_epoch'):
                row['data']['source_steady_time_ns']=publication-300_000_000+25
        self.assertTrue(any(f['reason']=='timestamp boundary is unscorable' for f in audit(rows)['failures']))
    def test_wide_local_publish_far_from_boundary_passes(self):
        rows=observed_trace()
        loc=next(r for r in rows if r['event']=='localization_publication')
        loc['publication_bracket']=[loc['monotonic_ns']-500_000,loc['monotonic_ns']+200_000]
        self.assertEqual(audit(rows)['assessment'],'PASS')
    def test_local_publish_crosses_identity_boundary(self):
        rows=observed_trace()
        loc=next(r for r in rows if r['event']=='localization_publication')
        new=copy.deepcopy(loc);new['data']['health']=3
        new['publication_bracket']=[1_000_500_000,1_001_500_000];new['monotonic_ns']=1_001_000_000
        rows.append(new)
        self.assertEqual(audit(rows)['assessment'],'UNSCORABLE')
    def test_shared_offset_cancels_even_if_wider_than_matching_window(self):
        rows=observed_trace()
        for row in rows:
            if 'observation' not in row:continue
            c=row['observation']['callback_clock'];c['monotonic_before']-=60_000_000;c['monotonic_after']+=60_000_000
        projected,clock=timeline(rows)
        safe=next(r for r in projected if r['event']=='/cmd_vel_safe');relay=next(r for r in projected if r['event']=='/cmd_vel_epoch')
        self.assertGreater(bounds(safe)[1]-bounds(safe)[0],100_000_000)
        self.assertEqual(difference(safe,relay),(1,1))
        self.assertEqual(within(difference(safe,relay),-50_000_000,50_000_000),'pass')
    def test_cross_matching_boundary_is_unscorable(self):
        # Small measured drift survives shared-offset cancellation.
        left=dict(monotonic_ns=0,_dds_wall_ns=50_000_000,_offset_drift_ns=100)
        right=dict(monotonic_ns=0,_dds_wall_ns=0,_offset_drift_ns=100)
        self.assertEqual(within(difference(left,right),-50_000_000,50_000_000),'unscorable')
        left['_dds_wall_ns']=50_000_101
        self.assertEqual(within(difference(left,right),-50_000_000,50_000_000),'fail')
    def test_optional_dds_sequence_none_preserved(self):
        value=capture(dict(source_timestamp=123,received_timestamp=456,publication_sequence_number=None,reception_sequence_number=None))
        self.assertIsNone(value['dds']['publication_sequence_number'])
        self.assertEqual(value['dds']['source_timestamp'],123)
        rows=observed_trace()
        next(r for r in rows if r['event']=='/cmd_vel_safe')['observation']['dds']['source_timestamp']=None
        self.assertFalse(audit(rows)['publication_clock']['available'])
    def test_legacy_data_remains_legacy(self):
        projected,meta=timeline(valid_trace())
        self.assertIsNone(projected);self.assertEqual(meta['basis'],'legacy_callback')
        self.assertFalse(audit(valid_trace())['failures'])


if __name__=='__main__':unittest.main()
