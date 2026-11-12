"""Pure exact SIM endpoint/root/READY/cleanup tests. No live socket or child."""
import copy,io,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest import mock
import full_scene_mapping as o
import full_scene_mapping_contract as c
import lateral_capability as proof
import mapping_lateral_relay as r
from test_v4_core import Clock,model

def bound_release(root):
    ue=dict(pid=1172665,starttime_ticks=1,cmdline_sha256='a'*64,argv=['/frozen/UeSim','SceneWorld'],boot_id='fixture')
    scene=dict(ue,pid_namespace='pid:[fixture]');path=root/'scene.json';path.write_text(json.dumps(scene));digest=c.sha(path)
    endpoint=dict(schema=c.SIM_ENDPOINT_SCHEMA,ipv4='192.168.1.219',port=7447,scene_identity_proof_sha256=digest,ue_process_identity=ue)
    return dict(allow_sim_host_endpoint=True,sim_command_endpoint=endpoint,scene_instance_id=scene,
        scene_identity_proof=dict(mode='root_external_host_attestation',evidence_path=str(path),evidence_sha256=digest,host_pid_namespace=scene['pid_namespace']))

class Endpoint(unittest.TestCase):
    def test_exact_flag_structure_ip_port_and_scene_attestation(self):
        self.assertEqual(c.sim_endpoint_from_release({})['ipv4'],'127.0.0.1')
        with tempfile.TemporaryDirectory() as name:
            release=bound_release(Path(name));self.assertEqual(c.sim_endpoint_from_release(release),release['sim_command_endpoint'])
            for field,values in [('ipv4',['127.0.0.1','192.168.1.218','localhost','::1']),('port',[0,6000,7448,True,7447.,'7447'])]:
                for value in values:
                    bad=copy.deepcopy(release);bad['sim_command_endpoint'][field]=value
                    with self.subTest(field=field,value=value),self.assertRaises(c.Hold):c.sim_endpoint_from_release(bad)
            for flag in [None,0,1,'true',False]:
                bad=copy.deepcopy(release);bad['allow_sim_host_endpoint']=flag
                with self.assertRaises(c.Hold):c.sim_endpoint_from_release(bad)
            bad=copy.deepcopy(release);bad.pop('allow_sim_host_endpoint')
            with self.assertRaises(c.Hold):c.sim_endpoint_from_release(bad)
            for mutate in [lambda x:x['sim_command_endpoint']['ue_process_identity'].update(pid=2),lambda x:x['sim_command_endpoint'].update(scene_identity_proof_sha256='b'*64),lambda x:x['scene_identity_proof'].update(host_pid_namespace='other')]:
                bad=copy.deepcopy(release);mutate(bad)
                with self.assertRaises(c.Hold):c.sim_endpoint_from_release(bad)
            Path(release['scene_identity_proof']['evidence_path']).write_text('{}')
            with self.assertRaises(c.Hold):c.sim_endpoint_from_release(release)
    def test_passive_constructor_freezes_matching_endpoint_and_contract(self):
        with tempfile.TemporaryDirectory() as name,mock.patch.object(o.subprocess,'Popen',side_effect=AssertionError('no child')) as popen:
            root=Path(name);release=bound_release(root);path=root/'release.json';path.write_text(json.dumps(release));out=root/'out';out.mkdir()
            relay=o.OwnedRelay(out,'fixture',path,model(),{'pid':1});admission=c.read_json(relay.admission_path)
            self.assertEqual(relay.sim_command_endpoint,release['sim_command_endpoint']);self.assertEqual(c.sim_endpoint_from_admission(admission,release),release['sim_command_endpoint'])
            self.assertEqual((out/'full_scene_mapping_contract.py').read_bytes(),Path(c.__file__).read_bytes())
            self.assertTrue(relay.expected_ready.endswith('192.168.1.219:7447'));relay.close();popen.assert_not_called()
    def test_actual_child_admission_rejects_endpoint_mismatch(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);release=bound_release(root);owner=dict(pid=1);adapter=dict(pid=2)
            release.update(release_id='a'*32,allow_execute=True,scope=r.SCOPE,mode='controlled_static_sim_scene',lateral_capability=r.CAPABILITY,output_directory=str(root),
                sources_sha256={n:proof.sha(Path(r.__file__).with_name(n)) for n in ['mapping_lateral_relay.py','lateral_capability.py','full_scene_mapping_contract.py']},
                lateral_calibration_proof={},sensor_config_sha256='c'*64,sensor_adapter_process_identity=adapter)
            path=root/'release.json';path.write_text(json.dumps(release));digest=proof.sha(path)
            admission=dict(root_release_path=str(path),root_release_sha256=digest,owner_identity=owner,motion_model={},sim_command_endpoint=c.sim_endpoint_from_release(release))
            (root/(release['release_id']+'.mapping_once.json')).write_text(json.dumps(dict(release_sha256=digest,output_directory=str(root),owner_identity=owner)))
            a=root/'admission.json';a.write_text(json.dumps(admission))
            with mock.patch.dict(r.os.environ,dict(MAPPING_OWNER_RUN_ID=release['release_id'],MAPPING_OWNER_OUTPUT=str(root))),mock.patch.object(r.os,'getppid',return_value=1),\
                mock.patch.object(r,'identity',side_effect=lambda pid:owner if pid==1 else adapter),mock.patch.object(proof,'validate_proof',return_value={}),mock.patch.object(proof,'validate_model_evidence'):
                self.assertEqual(r.admit(a,proof.sha(a))['sim_command_endpoint']['ipv4'],'192.168.1.219')
                admission['sim_command_endpoint']['ipv4']='127.0.0.1';a.write_text(json.dumps(admission))
                with self.assertRaises(c.Hold):r.admit(a,proof.sha(a))
    def test_actual_READY_requires_exact_same_endpoint(self):
        for actual,good in [('127.0.0.1:7447',False),('192.168.1.219:7447',True)]:
            clock=Clock();relay=o.OwnedRelay.__new__(o.OwnedRelay);relay.argv=['fixture'];relay.run_id='fixture';relay.output=Path('/fixture');relay.log=object();relay.frozen=Path('/fixture/relay.py');relay.source_sha256='x';relay.close=mock.Mock();relay.identity=None
            relay.expected_ready='MAPPING_LATERAL_RELAY_RECT_V5_READY 192.168.1.219:7447';process=N(pid=17,poll=lambda:None,stdin=N(fileno=lambda:1),stdout=N(fileno=lambda:2))
            with mock.patch.object(o,'time',clock),mock.patch.object(o.subprocess,'Popen',return_value=process),mock.patch.object(o.os,'pidfd_open',return_value=19),mock.patch.object(o.os,'set_blocking'),\
                mock.patch.object(o.select,'select',side_effect=lambda *a:([process.stdout],[],[])),mock.patch.object(o.os,'read',return_value=('MAPPING_LATERAL_RELAY_RECT_V5_READY '+actual+'\n').encode()),\
                mock.patch.object(c,'process_identity',return_value={'argv':['fixture']}),mock.patch.object(c,'sha',return_value='x'):
                if good:relay.start(lambda:clock.sleep(.01));relay.close.assert_not_called()
                else:
                    with self.assertRaises(c.Hold):relay.start(lambda:clock.sleep(.01))
                    relay.close.assert_called_once()
    def test_positive_and_all_cleanup_zero_use_same_endpoint_and_bad_tuple_never_sends(self):
        for host in ['127.0.0.1','192.168.1.219']:
            clock=Clock();rows=[];sock=N(sendto=lambda raw,target:rows.append((json.loads(raw),target)) or len(raw),close=mock.Mock())
            stream=io.StringIO(json.dumps(c.primitive_packet(.4,0.,clock.time_ns()))+'\n')
            result=r.run_loop(sock,stream,lambda:None,clock,lambda a,b,c,t:(a,[],[]),endpoint=(host,7447))
            self.assertEqual(result['accepted'],1);self.assertEqual(len(rows),6);self.assertTrue(all(target==(host,7447) for packet,target in rows));self.assertTrue(all(packet==r.ZERO for packet,target in rows[-5:]));sock.close.assert_called_once()
        for endpoint in [('192.168.1.220',7447),('192.168.1.219',7448),('192.168.1.219',7447.),['192.168.1.219',7447]]:
            sock=N(sendto=mock.Mock(),close=mock.Mock())
            with self.assertRaises(ValueError):r.run_loop(sock,io.StringIO(''),lambda:None,endpoint=endpoint)
            sock.sendto.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
