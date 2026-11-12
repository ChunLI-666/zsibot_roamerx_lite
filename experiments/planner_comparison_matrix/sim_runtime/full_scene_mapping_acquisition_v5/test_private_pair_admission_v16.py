"""Pure old/new whole-family evidence admission; synthetic metadata, no Node/UDP."""
import ast
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import lateral_capability as p
import full_scene_mapping_contract as c
import full_scene_mapping as owner
import mapping_lateral_relay as relay
from test_v4_core import BASE,proof_fixture
from test_sim_endpoint_v15 import bound_release

PRIVATE=BASE/'sim_semantic_lateral_private_candidate_v4/review_source_v4r2'


class PrivateFixture:
    def __init__(self,root):
        self.root=root
        self.path,self.proof,self.scene,self.sensor,self.adapter,self.objects=proof_fixture(root)
        self.artifacts=self.proof['artifacts']
        self.scene.update(argv=['/pure/UeSim','SceneWorld'],pid_namespace='pid:[host]')
        def identity(pid):return dict(pid=pid,starttime_ticks=pid+100,cmdline_sha256=f'{pid:064x}',argv=['/pure/source_'+str(pid)],boot_id=self.scene['boot_id'])
        self.adapter=identity(140);self.sources=dict(new_adapter=self.adapter,bridge=identity(47),projector=identity(97))
        self.cid='a'*64;self.net='net:[private]';self.pidns='pid:[private]'
        self.put('container_attestation',dict(Id=self.cid,Image=p.PRIVATE_IMAGE,netns=self.net,pidns=self.pidns))
        case=copy.deepcopy(self.objects['case']);case.update(executable=True,private_network_endpoint=True,command_endpoint=list(p.PRIVATE_ENDPOINT),
            container_id=self.cid,source_identities=self.sources,scene_instance_id=self.scene,
            container_attestation_artifact=copy.deepcopy(self.artifacts['container_attestation']))
        self.put('case',case);self.put('adapter_identity',dict(state='PASS',sources=self.sources))
        self.put('source_manifest',json.loads((PRIVATE/'source_manifest_v4r2.json').read_text()))
        for name in p.CAL_SOURCES:self.put('source_'+name,(PRIVATE/name).read_bytes(),raw=True)
        for index in [0,1]:
            rows=[json.loads(line) for line in self.objects[f'probe{index}_trace'].decode().splitlines()]
            for row in rows:row['target']=list(p.PRIVATE_ENDPOINT)
            self.put(f'probe{index}_trace',(''.join(json.dumps(v)+'\n' for v in rows)).encode(),raw=True)
        result=copy.deepcopy(self.objects['result']);result['source_package_sha256']=p.PRIVATE_CAL_SOURCE_SHA;self.put('result',result)
        host=dict(state='PASS',mode='root_external_host_attestation',completed_unix_ns=9_900_000_000,scene_identity=self.scene,
            uid=1000,boot_id=self.scene['boot_id'],command_endpoint=list(p.PRIVATE_ENDPOINT),container_id=self.cid,container_image=p.PRIVATE_IMAGE,
            collector_sha256=p.PRIVATE_CAL_SOURCE_SHA['host_preflight.py'],shared_host_network=False,private_bridge_verified=True,
            container_sources=self.sources,container_attestation_sha256=self.artifacts['container_attestation']['sha256'],
            port7447_exact_scene_owner=True,retired_workers_verified=True,forbidden_live_processes=[],artifacts_verified=[dict(pure=True)],
            network_namespace=self.net,container_network_namespace=self.net,host_network_namespace='net:[host]',container_pid_namespace=self.pidns)
        self.put('host_preflight',host)
        self.put('calibration_root_release',dict(scope=p.CAL_SCOPE,allow_execute=True,allow_private_network_endpoint=True,command_endpoint=list(p.PRIVATE_ENDPOINT),
            package_sha256=p.PRIVATE_CAL_SOURCE_SHA,case_sha256=self.artifacts['case']['sha256'],uid=1000,boot_id=self.scene['boot_id'],
            network_namespace=self.net,controller_pid_namespace=self.pidns,
            host_preflight=dict(mode='root_external_host_attestation',sha256=self.artifacts['host_preflight']['sha256'],max_age_sec=30.)))
        measurements=copy.deepcopy(self.objects['model_measurements']);measurements.update(source_package_sha256=p.PRIVATE_CAL_SOURCE_SHA,command_endpoint=list(p.PRIVATE_ENDPOINT))
        self.put('model_measurements',measurements)
        self.proof.update(source_package_sha256=p.PRIVATE_CAL_SOURCE_SHA,scene_instance_id=self.scene,adapter_process_identity=self.adapter,command_endpoint=list(p.PRIVATE_ENDPOINT))
        self.endpoint=dict(schema=c.SIM_ENDPOINT_SCHEMA,ipv4=p.PRIVATE_ENDPOINT[0],port=7447,scene_identity_proof_sha256='f'*64,
            ue_process_identity={key:self.scene[key] for key in p.IDENTITY_KEYS})
        self.sync()

    def put(self,key,value,raw=False):
        path=Path(self.artifacts[key]['path']) if key in self.artifacts else self.root/(key+'.json')
        path.write_bytes(value if raw else (json.dumps(value)+'\n').encode())
        self.artifacts[key]=dict(path=str(path),sha256=p.sha(path));self.objects[key]=copy.deepcopy(value)

    def sync(self):
        # Rebind hashes so failures below exercise semantics, not just changed bytes.
        if 'container_attestation' in self.artifacts:
            case=copy.deepcopy(self.objects['case']);case['container_attestation_artifact']=copy.deepcopy(self.artifacts['container_attestation']);self.put('case',case)
            if 'host_preflight' in self.artifacts:
                host=copy.deepcopy(self.objects['host_preflight']);host['container_attestation_sha256']=self.artifacts['container_attestation']['sha256'];self.put('host_preflight',host)
        if 'calibration_root_release' in self.artifacts:
            release=copy.deepcopy(self.objects['calibration_root_release']);release['case_sha256']=self.artifacts['case']['sha256']
            if 'host_preflight' in self.artifacts:release['host_preflight']['sha256']=self.artifacts['host_preflight']['sha256']
            self.put('calibration_root_release',release)
        measurement=copy.deepcopy(self.objects['model_measurements'])
        measurement['actual_case_artifact_sha256']={key:item['sha256'] for key,item in self.artifacts.items() if key not in ('model_measurements','measurement_manifest')}
        self.put('model_measurements',measurement)
        self.put('measurement_manifest',dict(runtime_approved=False,files={
            'model_measurements.json':dict(sha256=self.artifacts['model_measurements']['sha256']),
            'coverage_correction_v2.json':dict(sha256=self.artifacts['coverage_correction']['sha256'])}))
        self.path.write_text(json.dumps(self.proof))

    def admit(self,endpoint=None,adapter=None,namespace=None):
        self.sync()
        return p.validate_proof(dict(path=str(self.path),sha256=p.sha(self.path)),self.scene,self.sensor,self.adapter if adapter is None else adapter,
            command_endpoint=self.endpoint if endpoint is None else endpoint,
            namespace_binding=dict(network_namespace=self.net,controller_pid_namespace=self.pidns) if namespace is None else namespace)


class PrivateAdmission(unittest.TestCase):
    def test_old_default_and_new_private_each_preserve_original_HOLD(self):
        with tempfile.TemporaryDirectory() as d:
            path,value,scene,sensor,adapter,objects=proof_fixture(Path(d))
            actual=p.validate_proof(dict(path=str(path),sha256=p.sha(path)),scene,sensor,adapter)
            self.assertEqual(actual['calibration_source_family'],'legacy_shared_v4');self.assertEqual(actual['command_endpoint'],list(p.LEGACY_ENDPOINT))
            self.assertFalse(actual['calibration_diagnostic_passed'])
        with tempfile.TemporaryDirectory() as d:
            f=PrivateFixture(Path(d));actual=f.admit()
            self.assertEqual(actual['calibration_source_family'],'private_v4r2');self.assertEqual(actual['command_endpoint'],list(p.PRIVATE_ENDPOINT))
            self.assertFalse(actual['calibration_diagnostic_passed']);self.assertEqual(f.objects['result']['state'],'HOLD_DIAGNOSTIC_FAILED')
            self.assertEqual(json.loads((PRIVATE/'source_manifest_v4r2.json').read_text())['runtime_package_sha256'],p.PRIVATE_CAL_SOURCE_SHA)

    def test_no_mixed_partial_extra_or_unknown_source_family(self):
        for replacement in ['mixed','missing','extra','unknown']:
            with self.subTest(replacement=replacement),tempfile.TemporaryDirectory() as d:
                f=PrivateFixture(Path(d));result=copy.deepcopy(f.objects['result']);sources=copy.deepcopy(result['source_package_sha256'])
                if replacement=='mixed':sources['lateral_once.py']=p.CAL_SOURCE_SHA['lateral_once.py']
                elif replacement=='missing':sources.pop('bag_summary.py')
                elif replacement=='extra':sources['other.py']='1'*64
                else:sources['lateral_once.py']='0'*64
                result['source_package_sha256']=sources;f.put('result',result)
                with self.assertRaises(p.AdmissionError):f.admit()

    def test_family_has_one_pinned_mapping_endpoint_and_exact_typed_trace_endpoint(self):
        with tempfile.TemporaryDirectory() as d:
            f=PrivateFixture(Path(d))
            with self.assertRaises(p.AdmissionError):f.admit(endpoint=dict(schema=c.SIM_ENDPOINT_SCHEMA,ipv4='127.0.0.1',port=7447))
            f.sync()
            with self.assertRaises(p.AdmissionError):p.validate_proof(dict(path=str(f.path),sha256=p.sha(f.path)),f.scene,f.sensor,f.adapter)
        for bad in [['127.0.0.1',7447],['192.168.1.219',7447.],['192.168.1.219',7448],['localhost',7447]]:
            with self.subTest(bad=bad),tempfile.TemporaryDirectory() as d:
                f=PrivateFixture(Path(d));rows=[json.loads(line) for line in f.objects['probe1_trace'].decode().splitlines()];rows[-1]['target']=bad
                f.put('probe1_trace',(''.join(json.dumps(v)+'\n' for v in rows)).encode(),raw=True)
                with self.assertRaises(p.AdmissionError):f.admit()
        with tempfile.TemporaryDirectory() as d:
            path,value,scene,sensor,adapter,objects=proof_fixture(Path(d))
            with self.assertRaises(p.AdmissionError):p.validate_proof(dict(path=str(path),sha256=p.sha(path)),scene,sensor,adapter,
                command_endpoint=dict(schema=c.SIM_ENDPOINT_SCHEMA,ipv4='192.168.1.219',port=7447))

    def test_case_proof_measurement_release_host_endpoint_and_whole8_cannot_mix(self):
        for key,field,value in [('case','command_endpoint',list(p.LEGACY_ENDPOINT)),('model_measurements','command_endpoint',list(p.LEGACY_ENDPOINT)),
                ('host_preflight','command_endpoint',list(p.LEGACY_ENDPOINT)),('calibration_root_release','command_endpoint',list(p.LEGACY_ENDPOINT)),
                ('source_manifest','runtime_package_sha256',p.CAL_SOURCE_SHA),('model_measurements','source_package_sha256',p.CAL_SOURCE_SHA),
                ('calibration_root_release','package_sha256',p.CAL_SOURCE_SHA)]:
            with self.subTest(key=key,field=field),tempfile.TemporaryDirectory() as d:
                f=PrivateFixture(Path(d));obj=copy.deepcopy(f.objects[key]);obj[field]=value;f.put(key,obj)
                with self.assertRaises(p.AdmissionError):f.admit()
        with tempfile.TemporaryDirectory() as d:
            f=PrivateFixture(Path(d));f.proof['command_endpoint']=list(p.LEGACY_ENDPOINT)
            with self.assertRaises(p.AdmissionError):f.admit()

    def test_new_actual_adapter_precheck_must_be_same_current_three_not_old_relabel(self):
        for mutation in ['wrong_adapter','wrong_current_source','old_roles','no_precheck','partial_source']:
            with self.subTest(mutation=mutation),tempfile.TemporaryDirectory() as d:
                f=PrivateFixture(Path(d))
                if mutation=='wrong_adapter':
                    with self.assertRaises(p.AdmissionError):f.admit(adapter=dict(f.adapter,pid=5237))
                    continue
                obj=copy.deepcopy(f.objects['adapter_identity'])
                if mutation=='wrong_current_source':obj['sources']['bridge']['starttime_ticks']+=1
                elif mutation=='old_roles':obj['sources']['bridge24']=obj['sources'].pop('bridge');obj['sources']['projector61']=obj['sources'].pop('projector')
                elif mutation=='no_precheck':obj=f.adapter
                else:obj['sources']['bridge'].pop('argv')
                f.put('adapter_identity',obj)
                with self.assertRaises(p.AdmissionError):f.admit()

    def test_private_identity_ns_image_collector_and_original_ttl_stay_bound(self):
        mutations=[('host_preflight','container_sources',{}),('host_preflight','shared_host_network',True),
            ('host_preflight','container_network_namespace','net:[host]'),('host_preflight','collector_sha256','0'*64),
            ('host_preflight','container_image','wrong'),('host_preflight','completed_unix_ns',-1),
            ('container_attestation','Id','b'*64),('calibration_root_release','controller_pid_namespace','pid:[host]')]
        for key,field,value in mutations:
            with self.subTest(key=key,field=field),tempfile.TemporaryDirectory() as d:
                f=PrivateFixture(Path(d));obj=copy.deepcopy(f.objects[key]);obj[field]=value;f.put(key,obj)
                with self.assertRaises(p.AdmissionError):f.admit()
        with tempfile.TemporaryDirectory() as d:
            f=PrivateFixture(Path(d));endpoint=copy.deepcopy(f.endpoint);endpoint['ue_process_identity']['starttime_ticks']+=1
            with self.assertRaises(p.AdmissionError):f.admit(endpoint=endpoint)
        with tempfile.TemporaryDirectory() as d:
            f=PrivateFixture(Path(d))
            with self.assertRaises(p.AdmissionError):f.admit(namespace=dict(network_namespace='net:[host]',controller_pid_namespace=f.pidns))
        with tempfile.TemporaryDirectory() as d:
            f=PrivateFixture(Path(d));release=copy.deepcopy(f.objects['calibration_root_release']);release['host_preflight']['max_age_sec']=.001;f.put('calibration_root_release',release)
            with self.assertRaisesRegex(p.AdmissionError,'TTL'):f.admit()

    def test_missing_private_artifact_or_measurement_link_is_HOLD(self):
        for key in ['calibration_root_release','host_preflight','container_attestation']:
            with self.subTest(key=key),tempfile.TemporaryDirectory() as d:
                f=PrivateFixture(Path(d));f.artifacts.pop(key)
                with self.assertRaises(p.AdmissionError):f.admit()
        with tempfile.TemporaryDirectory() as d:
            f=PrivateFixture(Path(d));f.sync();obj=copy.deepcopy(f.objects['model_measurements']);obj['actual_case_artifact_sha256'].pop('host_preflight');f.put('model_measurements',obj)
            f.path.write_text(json.dumps(f.proof))
            with self.assertRaises(p.AdmissionError):p.validate_proof(dict(path=str(f.path),sha256=p.sha(f.path)),f.scene,f.sensor,f.adapter,command_endpoint=f.endpoint,
                namespace_binding=dict(network_namespace=f.net,controller_pid_namespace=f.pidns))

    def test_actual_child_admission_passes_same_Root_verified_endpoint_to_proof(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);release=bound_release(root);parent=dict(pid=1);adapter=dict(pid=2)
            release.update(release_id='a'*32,allow_execute=True,scope=relay.SCOPE,mode='controlled_static_sim_scene',lateral_capability=relay.CAPABILITY,
                network_namespace='net:[private]',controller_pid_namespace='pid:[private]',
                output_directory=str(root),sources_sha256={name:p.sha(Path(relay.__file__).with_name(name)) for name in ['mapping_lateral_relay.py','lateral_capability.py','full_scene_mapping_contract.py']},
                lateral_calibration_proof={},sensor_config_sha256='c'*64,sensor_adapter_process_identity=adapter)
            path=root/'release.json';path.write_text(json.dumps(release));digest=p.sha(path)
            admission=dict(root_release_path=str(path),root_release_sha256=digest,owner_identity=parent,motion_model={},sim_command_endpoint=c.sim_endpoint_from_release(release))
            (root/(release['release_id']+'.mapping_once.json')).write_text(json.dumps(dict(release_sha256=digest,output_directory=str(root),owner_identity=parent)))
            a=root/'admission.json';a.write_text(json.dumps(admission))
            with mock.patch.dict(relay.os.environ,dict(MAPPING_OWNER_RUN_ID=release['release_id'],MAPPING_OWNER_OUTPUT=str(root))),\
                 mock.patch.object(relay.os,'getppid',return_value=1),mock.patch.object(relay,'identity',side_effect=lambda pid:parent if pid==1 else adapter),\
                 mock.patch.object(p,'validate_proof',return_value={}) as validate,mock.patch.object(p,'validate_model_evidence'):
                relay.admit(a,p.sha(a));self.assertEqual(validate.call_args.kwargs['command_endpoint'],release['sim_command_endpoint'])
                self.assertEqual(validate.call_args.kwargs['namespace_binding'],dict(network_namespace='net:[private]',controller_pid_namespace='pid:[private]'))
        tree=ast.parse(Path(owner.__file__).read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='validate_release')
        call=next(n for n in ast.walk(fn) if isinstance(n,ast.Call) and ast.unparse(n.func)=='lateral_capability.validate_proof')
        self.assertEqual([ast.unparse(k.value) for k in call.keywords if k.arg=='command_endpoint'],['sim_endpoint'])


if __name__=='__main__':unittest.main(verbosity=2)
