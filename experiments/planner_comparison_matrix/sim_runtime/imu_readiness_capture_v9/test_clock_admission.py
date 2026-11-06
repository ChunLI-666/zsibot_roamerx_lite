"""Only an old discovery clock may be rejected without accepted-state refresh."""
import unittest
from types import SimpleNamespace as N
from readiness_validator import Validator
from test_readiness import CONFIG,gid,cdr,message,c,RAW_IMU,GT

CLOCK='/clock'
def clock(ns):return N(clock=N(sec=ns//10**9,nanosec=ns%10**9))
class ClockAdmission(unittest.TestCase):
    def validator(self):return Validator(CONFIG,lambda x:x)
    def test_discovery_old_clock_retained_rejected_without_any_accepted_state(self):
        v=self.validator();ns=100*10**9;row=v.sample(CLOCK,b'original_clock',clock(ns),ns+322_278_722,123,gid(CLOCK))
        self.assertFalse(row['accepted']);self.assertTrue(row['admission_rejected']);self.assertEqual(row['stamp_ns'],ns)
        self.assertEqual(v.received[CLOCK],1);self.assertEqual(v.accepted[CLOCK],0);self.assertNotIn(CLOCK,v.latest)
        self.assertEqual(v.clocks,{});self.assertEqual(v.stats,{});self.assertEqual(v.discovery_clock_rejections,1)
        self.assertEqual(row['received_unix_ns'],ns+322_278_722);self.assertEqual(row['received_steady_ns'],123)
    def test_fresh_followup_is_accepted_unchanged(self):
        v=self.validator();ns=100*10**9
        v.sample(CLOCK,b'old',clock(ns),ns+400_000_000,123,gid(CLOCK))
        fresh=ns+500_000_000;row=v.sample(CLOCK,b'new',clock(fresh),fresh+1_000_000,456,gid(CLOCK))
        self.assertTrue(row['accepted']);self.assertEqual(v.latest[CLOCK]['stamp_ns'],fresh)
        self.assertEqual(v.accepted[CLOCK],1);self.assertEqual(v.discovery_clock_rejections,1)
    def test_any_stale_clock_in_core_remains_fatal_and_no_rejection_counter(self):
        v=self.validator();v.measure_start=0;ns=100*10**9
        with self.assertRaisesRegex(c.Hold,'stale/future'):v.sample(CLOCK,b'old',clock(ns),ns+300_000_001,123,gid(CLOCK))
        self.assertEqual(v.discovery_clock_rejections,0);self.assertEqual(v.accepted[CLOCK],0)
    def test_future_clock_remains_fatal_even_before_core(self):
        v=self.validator();ns=100*10**9
        with self.assertRaisesRegex(c.Hold,'stale/future'):v.sample(CLOCK,b'future',clock(ns),ns-50_000_001,123,gid(CLOCK))
        self.assertEqual(v.discovery_clock_rejections,0)
    def test_nonadvance_clock_and_other_topics_stale_remain_fatal(self):
        v=self.validator();ns=100*10**9;v.sample(CLOCK,b'new',clock(ns),ns,0,gid(CLOCK))
        with self.assertRaisesRegex(c.Hold,'nonadvancing'):v.sample(CLOCK,b'old',clock(ns),ns+400_000_000,123,gid(CLOCK))
        v=self.validator();v.measure_start=0
        with self.assertRaisesRegex(c.Hold,'stale/future'):v.sample(GT,b'old',message(GT,ns),ns+300_000_001,123,gid(GT))
    def test_age_boundaries_and_repeated_rejections_do_not_admit_or_reset_phase(self):
        ns=100*10**9
        for age in [299_000_000,300_000_000]:
            v=self.validator();self.assertTrue(v.sample(CLOCK,b'bound',clock(ns),ns+age,123,gid(CLOCK))['accepted'])
        v=self.validator()
        for age in [300_000_001,322_278_722,3_500_000_000]:
            self.assertFalse(v.sample(CLOCK,b'old',clock(ns),ns+age,age,gid(CLOCK))['accepted'])
        self.assertEqual(v.discovery_clock_rejections,3);self.assertIsNone(v.measure_start)
        self.assertEqual(v.latest,{});self.assertEqual(v.clocks,{});self.assertEqual(v.clock_matched,set())
    def test_increasing_but_old_discovery_clock_does_not_refresh_previous_fresh(self):
        v=self.validator();ns=100*10**9;v.sample(CLOCK,b'fresh',clock(ns),ns,123,gid(CLOCK))
        before=dict(v.latest[CLOCK]);stats=dict(v.stats[CLOCK])
        row=v.sample(CLOCK,b'queued',clock(ns+50_000_000),ns+1_000_000_000,456,gid(CLOCK))
        self.assertFalse(row['accepted']);self.assertEqual(v.latest[CLOCK],before);self.assertEqual(v.stats[CLOCK],stats)
        self.assertEqual(set(v.clocks),{ns});self.assertEqual(v.accepted[CLOCK],1)
if __name__=='__main__':unittest.main(verbosity=2)
