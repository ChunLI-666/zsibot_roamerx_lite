"""Unified bounded warmup rejects old valid payloads; core and faults stay strict."""
import copy
import unittest
from unittest import mock
from types import SimpleNamespace as N
import readiness_capture as capture
from readiness_validator import Validator,GT,RAW_CAMERA,CAMERA,INFO
from test_readiness import CONFIG,gid,message,converted,c,Clock
TOPICS=[GT,'/front_lidar',RAW_CAMERA,CAMERA,INFO,'/clock']
class DiscoveryInputs(unittest.TestCase):
    def validator(self):return Validator(CONFIG,converted)
    def test_all_old_valid_nonIMU_inputs_are_audited_without_accepted_state(self):
        ns=100*10**9
        for topic in TOPICS:
            v=self.validator();row=v.sample(topic,b'original bytes',message(topic,ns),ns+400_000_000,123,gid(topic))
            self.assertFalse(row['accepted']);self.assertTrue(row['admission_rejected']);self.assertEqual(row['stamp_ns'],ns)
            self.assertEqual(v.accepted[topic],0);self.assertEqual(v.latest,{});self.assertEqual(v.stats,{})
            self.assertEqual(v.raw,{});self.assertEqual(v.derived,{});self.assertEqual(v.cameras,{});self.assertEqual(v.images,{})
            self.assertEqual(v.infos,{});self.assertEqual(v.clocks,{});self.assertIsNone(v.previous_gt);self.assertIsNone(v.stop.stamp)
            self.assertEqual(v.discovery_header_rejections[topic],1);self.assertIsNone(v.measure_start)
    def test_core_zero_positive_and_drain_keep_all_stale_future_HOLD(self):
        ns=100*10**9
        for topic in TOPICS:
            for phase in [0,1]:
                for age in [300_000_001,-50_000_001]:
                    v=self.validator();v.measure_start=phase;v.measure_end=phase+1
                    with self.assertRaisesRegex(c.Hold,'stale/future'):v.sample(topic,b'x',message(topic,ns),ns+age,123,gid(topic))
                    self.assertEqual(v.discovery_header_rejections[topic],0)
    def test_old_bad_payload_frame_standing_intrinsics_and_layout_remain_fatal(self):
        ns=100*10**9
        cases=[]
        for topic in [GT,'/front_lidar',RAW_CAMERA,CAMERA,INFO]:
            m=message(topic,ns);m.header.frame_id='wrong';cases.append((topic,m))
        m=message(GT,ns);m.pose.pose.position.z=.1;cases.append((GT,m))
        m=message(GT,ns);m.pose.pose.orientation.w=0.;cases.append((GT,m))
        m=message(GT,ns);m.twist.twist.linear.x=float('nan');cases.append((GT,m))
        m=message('/front_lidar',ns);m.data=b'';cases.append(('/front_lidar',m))
        for topic in [RAW_CAMERA,CAMERA]:
            m=message(topic,ns);m.data=b'';cases.append((topic,m))
        m=message(INFO,ns);m.k[0]+=1.;cases.append((INFO,m))
        for topic,m in cases:
            v=self.validator()
            with self.assertRaises(c.Hold):v.sample(topic,b'old',m,ns+400_000_000,123,gid(topic))
            self.assertEqual(v.discovery_header_rejections[topic],0)
    def test_increasing_old_does_not_refresh_existing_fresh_state_or_pairing(self):
        ns=100*10**9
        for topic in TOPICS:
            v=self.validator();v.sample(topic,b'fresh',message(topic,ns),ns,123,gid(topic))
            names=['latest','stats','accepted','cameras','images','infos','clocks','previous_gt']
            before=copy.deepcopy({n:getattr(v,n) for n in names});stop=copy.deepcopy(vars(v.stop))
            later=ns+50_000_000;row=v.sample(topic,b'old',message(topic,later),ns+1_000_000_000,456,gid(topic))
            self.assertFalse(row['accepted']);self.assertEqual(before,{n:getattr(v,n) for n in names});self.assertEqual(stop,vars(v.stop))
    def test_invalid_old_timestamp_is_not_waived(self):
        ns=100*10**9
        for topic in TOPICS:
            m=message(topic,ns);h=m.clock if topic=='/clock' else m.header.stamp;h.sec=-1
            with self.assertRaisesRegex(c.Hold,'timestamp'):self.validator().sample(topic,b'old',m,ns,123,gid(topic))
    def fake_discovery(self,healthy=True,fault=False):
        clk=Clock();calls=[];pins=[]
        class V:
            def graph(self,g,pin=False):
                if pin:pins.append(clk.t)
            def check(self,*args):
                calls.append(clk.t)
                if not healthy:raise c.Hold('not yet fresh')
        obs=N(validator=V(),graph=lambda:{})
        def watch():
            if fault:raise c.Hold('capture fault')
            clk.t+=.1
        return clk,calls,pins,obs,watch
    def test_discovery_uses_full_original3s_and_pins_only_at_current_end(self):
        clk,calls,pins,obs,watch=self.fake_discovery()
        with mock.patch.object(capture.time,'monotonic',clk.monotonic),mock.patch.object(capture.time,'monotonic_ns',clk.monotonic_ns),mock.patch.object(capture.time,'time_ns',clk.time_ns):capture.bounded_discovery(obs,watch)
        self.assertGreaterEqual(clk.t,3.);self.assertLess(clk.t,3.11);self.assertEqual(pins,[clk.t]);self.assertGreater(len(calls),20)
    def test_discovery_never_current_still_HOLD_at_original_deadline(self):
        clk,calls,pins,obs,watch=self.fake_discovery(healthy=False)
        with mock.patch.object(capture.time,'monotonic',clk.monotonic),mock.patch.object(capture.time,'monotonic_ns',clk.monotonic_ns),mock.patch.object(capture.time,'time_ns',clk.time_ns),self.assertRaisesRegex(c.Hold,'3s bounded discovery'):capture.bounded_discovery(obs,watch)
        self.assertGreaterEqual(clk.t,3.);self.assertLess(clk.t,3.11);self.assertFalse(pins)
    def test_discovery_never_swallows_original_watch_capture_fault(self):
        clk,calls,pins,obs,watch=self.fake_discovery(fault=True)
        with mock.patch.object(capture.time,'monotonic',clk.monotonic),self.assertRaisesRegex(c.Hold,'capture fault'):capture.bounded_discovery(obs,watch)
        self.assertFalse(pins);self.assertFalse(calls)
if __name__=='__main__':unittest.main(verbosity=2)
