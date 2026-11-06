"""Pure exact-family/disabled-slot regressions. Synthetic metadata is not an actual calibration."""
import copy
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import lateral_capability as p
from test_private_pair_admission_v16 import PrivateFixture,BASE

FROZEN_V5=BASE/'sim_semantic_lateral_private_snapshot_retry_candidate_v5/review_source_v5'

def synthetic_eight():
    values={name:('# PURE METADATA FIXTURE '+name+'\n').encode() for name in p.CAL_SOURCES}
    return values,{name:hashlib.sha256(raw).hexdigest() for name,raw in values.items()}

def synthetic_private_fixture(root):
    fixture=PrivateFixture(root);raw,sources=synthetic_eight()
    for name,value in raw.items():fixture.put('source_'+name,value,raw=True)
    for key,field in [('source_manifest','runtime_package_sha256'),('result','source_package_sha256'),
                      ('model_measurements','source_package_sha256'),('calibration_root_release','package_sha256')]:
        value=copy.deepcopy(fixture.objects[key]);value[field]=sources;fixture.put(key,value)
    host=copy.deepcopy(fixture.objects['host_preflight']);host['collector_sha256']=sources['host_preflight.py'];fixture.put('host_preflight',host)
    fixture.proof['source_package_sha256']=sources
    return fixture,sources

class PendingFamily(unittest.TestCase):
    def test_final_exact8_and_empty_or_partial_replacement_cannot_admit(self):
        manifest=p.read(FROZEN_V5/'source_manifest_v5.json')
        self.assertEqual(p.PRIVATE_V5_CAL_SOURCE_SHA,manifest['runtime_package_sha256'])
        self.assertEqual(p.PRIVATE_V5_CAL_SOURCE_SHA,{name:p.sha(FROZEN_V5/name)for name in p.CAL_SOURCES})
        unused,full=synthetic_eight()
        for pending in [{},dict(list(full.items())[:-1]),dict(full,extra='f'*64),dict(full,**{'lateral_once.py':'UNKNOWN'}),None]:
            with mock.patch.object(p,'PRIVATE_V5_CAL_SOURCE_SHA',pending):
                for sources in [{},full,pending]:
                    with self.subTest(pending=pending),self.assertRaises(p.AdmissionError):p.calibration_family(sources)

    def test_original_whole_families_and_endpoint_roles_remain_exact(self):
        self.assertEqual(p.calibration_family(p.CAL_SOURCE_SHA),('legacy_shared_v4',p.LEGACY_ENDPOINT,{'new_adapter','bridge24','projector61'}))
        self.assertEqual(p.calibration_family(p.PRIVATE_CAL_SOURCE_SHA),('private_v4r2',p.PRIVATE_ENDPOINT,{'new_adapter','bridge','projector'}))
        unused,full=synthetic_eight()
        with mock.patch.object(p,'PRIVATE_V5_CAL_SOURCE_SHA',full):
            self.assertEqual(p.calibration_family(full),('private_v5_camera_refresh',p.PRIVATE_ENDPOINT,{'new_adapter','bridge','projector'}))
            for old in [p.CAL_SOURCE_SHA,p.PRIVATE_CAL_SOURCE_SHA]:
                mixed=dict(full);mixed['lateral_once.py']=old['lateral_once.py']
                with self.assertRaises(p.AdmissionError):p.calibration_family(mixed)

    def test_synthetic_complete_third_family_still_needs_all_actual_private_guards(self):
        with tempfile.TemporaryDirectory() as name:
            fixture,sources=synthetic_private_fixture(Path(name))
            with self.assertRaises(p.AdmissionError):fixture.admit()
            with mock.patch.object(p,'PRIVATE_V5_CAL_SOURCE_SHA',sources):
                actual=fixture.admit();self.assertEqual(actual['calibration_source_family'],'private_v5_camera_refresh')
                self.assertFalse(actual['calibration_diagnostic_passed'])
                fixture.put('adapter_identity',fixture.adapter)
                with self.assertRaises(p.AdmissionError):fixture.admit()

    def test_third_family_never_relabels_an_invalid_pair_or_bypasses_private_endpoint(self):
        for mutation in ['bad_endpoint','failed_positive','source_mix']:
            with self.subTest(mutation=mutation),tempfile.TemporaryDirectory() as name:
                fixture,sources=synthetic_private_fixture(Path(name))
                if mutation=='bad_endpoint':
                    value=copy.deepcopy(fixture.objects['case']);value['command_endpoint']=list(p.LEGACY_ENDPOINT);fixture.put('case',value)
                elif mutation=='failed_positive':
                    value=copy.deepcopy(fixture.objects['probe0_acceptance']);value['state']='FAILED_NOT_SUSTAINED_LATERAL';fixture.put('probe0_acceptance',value)
                else:
                    value=copy.deepcopy(fixture.objects['model_measurements']);value['source_package_sha256']=p.PRIVATE_CAL_SOURCE_SHA;fixture.put('model_measurements',value)
                with mock.patch.object(p,'PRIVATE_V5_CAL_SOURCE_SHA',sources),self.assertRaises(p.AdmissionError):fixture.admit()

    def test_frozen_real_v5_eight_selects_third_and_old_new_two_file_mix_rejects(self):
        for name in ['lateral_contract.py','semantic_tool_timed.py']:
            mixed=dict(p.PRIVATE_V5_CAL_SOURCE_SHA);mixed[name]=p.PRIVATE_CAL_SOURCE_SHA[name]
            with self.subTest(name=name),self.assertRaises(p.AdmissionError):p.calibration_family(mixed)
        with tempfile.TemporaryDirectory() as name:
            fixture=PrivateFixture(Path(name))
            for filename in p.CAL_SOURCES:fixture.put('source_'+filename,(FROZEN_V5/filename).read_bytes(),raw=True)
            fixture.put('source_manifest',p.read(FROZEN_V5/'source_manifest_v5.json'))
            for key,field in [('result','source_package_sha256'),('model_measurements','source_package_sha256'),('calibration_root_release','package_sha256')]:
                value=copy.deepcopy(fixture.objects[key]);value[field]=p.PRIVATE_V5_CAL_SOURCE_SHA;fixture.put(key,value)
            for key in ['case','calibration_root_release']:
                value=copy.deepcopy(fixture.objects[key]);value['allow_camera_stale_lease_retry_v1']=True;fixture.put(key,value)
            fixture.proof['source_package_sha256']=p.PRIVATE_V5_CAL_SOURCE_SHA
            actual=fixture.admit();self.assertEqual(actual['calibration_source_family'],'private_v5_camera_refresh')
            self.assertFalse(actual['calibration_diagnostic_passed']);self.assertEqual(actual['command_endpoint'],list(p.PRIVATE_ENDPOINT))
            # A failed actual positive remains invalid even with reviewed source.
            value=copy.deepcopy(fixture.objects['probe0_acceptance']);value['sustained']['final_active1s_complete']=False;fixture.put('probe0_acceptance',value)
            with self.assertRaises(p.AdmissionError):fixture.admit()

if __name__=='__main__':unittest.main(verbosity=2)
