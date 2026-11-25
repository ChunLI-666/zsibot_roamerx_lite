"""Warehouse-only scan age, terminal authority, and durable retry regression."""
import json
from pathlib import Path
import tempfile
import unittest
import uuid

from mission_resume import Session
from mission_state import Hold, MissionStore, atomic_json, failure_decision
from waypoint_contract import (
    operational_scan_age_issues, recoverable_warehouse_scan_pause,
    simulation_scan_latency_profile, validate_simulation_scan_latency_profile,
)
from test_mission_state import fixture

TOPIC = '/laser_scan_leveled_nav_floor_v1'
PROFILE = simulation_scan_latency_profile(.9)
ISSUE = 'scan:invalid_header_age'
NOW = 2_000_000_000


def scan_at(age):
    return dict(last=dict(scan=dict(stamp_ns=NOW-round(age*1e9),
        frame_id='base_link',ranges=723,confirmation_payload_valid=True,
        header_age_s=age,receive_age_s=age)))


def producer_at(age):
    return dict(warehouse_fullmap={'size_x':7904},simulation_scan_latency_profile=PROFILE,
        mission_deadline={'run_id':'original','manifest_sha256':'a'*64},
        producer_scan_observation=dict(header_ns=NOW-round(age*1e9),frame_id='base_link',ranges=723,
            validation_clock_begin_ns=NOW,validation_clock_end_ns=NOW,
            validation_steady_begin_ns=NOW,validation_steady_end_ns=NOW,receive_steady_seconds=2.-age))


class ScanResume(unittest.TestCase):
    def test_warehouse_operational_limit_and_legacy_scopes(self):
        ready=producer_at(.8)
        self.assertEqual(validate_simulation_scan_latency_profile(ready,.9),PROFILE)
        with self.assertRaisesRegex(ValueError,'mismatch'):
            validate_simulation_scan_latency_profile(ready,.3)
        with self.assertRaisesRegex(ValueError,'Warehouse'):
            validate_simulation_scan_latency_profile(dict(ready,warehouse_fullmap=None),.9)
        self.assertEqual(validate_simulation_scan_latency_profile({},.3)['mode'],'strict_300ms')
        for cap,age,ok in ((.3,.31,False),(.5,.51,False),(.9,.31,True),(.9,.9,True),(.9,.901,False)):
            with self.subTest(cap=cap,age=age):
                self.assertEqual(not operational_scan_age_issues([ISSUE],scan_at(age),cap),ok)

    def test_only_a_real_over_age_sample_can_pause(self):
        ready=producer_at(.901)
        scan=scan_at(.901)
        self.assertIsNone(recoverable_warehouse_scan_pause([ISSUE],scan_at(.9),producer_at(.9),TOPIC,NOW,NOW))
        self.assertIsNone(recoverable_warehouse_scan_pause([ISSUE,'odom:invalid_header_age'],scan,ready,TOPIC,NOW,NOW))
        self.assertIsNone(recoverable_warehouse_scan_pause([ISSUE],scan_at(-.1),ready,TOPIC,NOW,NOW))
        pause=recoverable_warehouse_scan_pause([ISSUE],scan,ready,TOPIC,NOW,NOW)
        self.assertEqual(pause['last_original_scan_ns'],scan['last']['scan']['stamp_ns'])
        result=dict(status=5,simulation_scan_latency_profile=PROFILE,
                    cancel_reason='input_became_invalid:'+ISSUE,warehouse_scan_pause=pause)
        for terminal,children,stopped in ((False,True,True),(True,False,True),(True,True,False)):
            self.assertEqual(failure_decision(result,terminal,children,stopped)[0],'HOLD')
        self.assertEqual(failure_decision(dict(result,warehouse_scan_pause=None),True,True,True)[0],'HOLD')
        self.assertEqual(failure_decision(result,True,True,True,operator_cancel=True)[0],'CANCELED')
        self.assertEqual(failure_decision(result,True,True,True)[0],'PAUSED_INPUT_HEALTH')

    def test_durable_resume_retains_mission_rejects_stale_scan_and_uses_new_action(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            store=MissionStore(root/'mission.sqlite')
            try:
                raw,identity,policy,_=fixture()
                route=json.loads(raw)
                route.update(scenario_profile='warehouse_fullmap',waypoints=route['waypoints'][:1])
                original=store.create(json.dumps(route).encode(),identity,policy)
                req=str(uuid.uuid4())
                first=store.reserve(req,root/'attempt0')
                store.acknowledge(first['parent_action_uuid'],True)
                pause=recoverable_warehouse_scan_pause([ISSUE],scan_at(.901),producer_at(.901),TOPIC,NOW,NOW)
                native=dict(action_uuid=first['parent_action_uuid'],status=5,error_code=0,missed_waypoints=[],
                    functional_status='FAIL',final_measured_and_commanded_stop=True,children_quiescent=True,
                    cancel_reason='input_became_invalid:'+ISSUE,simulation_scan_latency_profile=PROFILE,
                    warehouse_scan_pause=pause,finished_ns=NOW)
                result=root/'native.json';atomic_json(result,native)
                store.finish_attempt(native,True,result_path=result)
                state=store.state()
                self.assertEqual(state['lifecycle'],'PAUSED_INPUT_HEALTH')
                session=Session(store,req,root/'attempt1')
                with self.assertRaisesRegex(Hold,'newer original scan'):
                    session.validate_recovered_scan(producer_at(.901),scan_at(.901))
                session.validate_recovered_scan(producer_at(.05),scan_at(.05))
                second=store.reserve(req,root/'attempt1')
                self.assertNotEqual(second['parent_action_uuid'],first['parent_action_uuid'])
                self.assertEqual(second['start_index'],0)
                self.assertEqual(store.state()['mission_id'],original['mission_id'])
                self.assertEqual(store.state()['route'],original['route'])
            finally:
                store.close()


if __name__=='__main__':
    unittest.main()
