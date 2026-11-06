"""Initial reliable IMU cache rejection keeps the original gate and core strict."""
import unittest
from readiness_validator import Validator,RAW_IMU,IMU
from test_readiness import CONFIG,gid,cdr,message,converted,c
class ImuAdmission(unittest.TestCase):
    def validator(self):return Validator(CONFIG,converted)
    def frame(self,t):return 'map' if t==RAW_IMU else 'imu_link'
    def test_old_first_for_each_IMU_rejected_after_full_gate_without_acceptance(self):
        ns=100*10**9
        for topic in [RAW_IMU,IMU]:
            v=self.validator();gate=v.raw_gate if topic==RAW_IMU else v.derived_gate
            row=v.sample(topic,cdr(ns,self.frame(topic)),message(topic,ns),ns+400_000_000,123,gid(topic))
            self.assertFalse(row['accepted']);self.assertTrue(row['admission_rejected']);self.assertTrue(row['imu_gate']['fatal'])
            self.assertIn('original_cdr_base64',row['imu_gate']);self.assertEqual(v.discovery_imu_rejections[topic],1)
            self.assertEqual(gate.received,1);self.assertEqual(gate.rejected,1);self.assertIsNone(gate.last_accepted_ns)
            self.assertEqual(v.accepted[topic],0);self.assertEqual(v.latest,{});self.assertEqual(v.raw,{});self.assertEqual(v.derived,{})
            self.assertEqual(v.stats,{});self.assertEqual(v.paired,set());self.assertIsNone(v.measure_start)
    def test_followup_current_IMU_can_be_accepted_with_original_stamp(self):
        ns=100*10**9
        for topic in [RAW_IMU,IMU]:
            v=self.validator();v.sample(topic,cdr(ns,self.frame(topic)),message(topic,ns),ns+400_000_000,123,gid(topic))
            fresh=ns+500_000_000;row=v.sample(topic,cdr(fresh,self.frame(topic)),message(topic,fresh),fresh+1_000_000,456,gid(topic))
            self.assertTrue(row['accepted']);self.assertEqual(v.latest[topic]['stamp_ns'],fresh);self.assertEqual(v.accepted[topic],1)
    def test_core_and_drain_stale_or_future_are_still_fatal(self):
        ns=100*10**9
        for topic in [RAW_IMU,IMU]:
            for phase in [0,123]:
                for age in [300_000_001,-50_000_001]:
                    v=self.validator();v.measure_start=phase;v.measure_end=phase+1
                    with self.assertRaisesRegex(c.Hold,'original_header_stale_or_future'):v.sample(topic,cdr(ns,self.frame(topic)),message(topic,ns),ns+age,456,gid(topic))
                    self.assertEqual(v.discovery_imu_rejections[topic],0)
    def test_future_frame_nonfinite_and_malformed_never_become_discovery_rejections(self):
        ns=100*10**9
        for topic in [RAW_IMU,IMU]:
            cases=[(cdr(ns,self.frame(topic)),ns-50_000_001),(cdr(ns,'bad_frame'),ns+400_000_000),(b'bad',ns+400_000_000)]
            vals=[0.]*37;vals[0]=float('nan');cases.append((cdr(ns,self.frame(topic),vals),ns+400_000_000))
            for raw,wall in cases:
                v=self.validator()
                with self.assertRaises(c.Hold):v.sample(topic,raw,message(topic,ns),wall,123,gid(topic))
                self.assertEqual(v.discovery_imu_rejections[topic],0)
    def test_increasing_old_cache_never_refreshes_accepted_gate_or_validator(self):
        ns=100*10**9
        for topic in [RAW_IMU,IMU]:
            v=self.validator();v.sample(topic,cdr(ns,self.frame(topic)),message(topic,ns),ns,123,gid(topic))
            gate=v.raw_gate if topic==RAW_IMU else v.derived_gate;before=(gate.last_accepted_ns,gate.last_accepted_receipt_ns,gate.last_accepted_receipt_steady_ns,dict(v.latest[topic]),dict(v.stats[topic]))
            later=ns+50_000_000;v.sample(topic,cdr(later,self.frame(topic)),message(topic,later),ns+1_000_000_000,456,gid(topic))
            self.assertEqual(before,(gate.last_accepted_ns,gate.last_accepted_receipt_ns,gate.last_accepted_receipt_steady_ns,v.latest[topic],v.stats[topic]));self.assertEqual(v.accepted[topic],1)
    def test_derived_nonadvance_and_accepted_gap_faults_remain_fatal(self):
        ns=100*10**9;v=self.validator();v.sample(IMU,cdr(ns,'imu_link'),message(IMU,ns),ns,123,gid(IMU))
        with self.assertRaises(c.Hold):v.sample(IMU,cdr(ns,'imu_link'),message(IMU,ns),ns+400_000_000,456,gid(IMU))
        for topic in [RAW_IMU,IMU]:
            v=self.validator();v.sample(topic,cdr(ns,self.frame(topic)),message(topic,ns),ns,123,gid(topic));later=ns+100_000_001
            with self.assertRaisesRegex(c.Hold,'accepted_header_gap_exceeded'):v.sample(topic,cdr(later,self.frame(topic)),message(topic,later),later,456,gid(topic))
if __name__=='__main__':unittest.main(verbosity=2)
