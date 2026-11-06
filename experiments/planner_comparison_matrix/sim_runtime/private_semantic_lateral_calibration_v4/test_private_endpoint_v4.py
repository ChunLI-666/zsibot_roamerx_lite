"""Finite fake transport/host collector/identity tests; no Node or real socket."""
import copy,hashlib,io,json,tempfile,unittest
from pathlib import Path
from unittest import mock
import lateral_contract as c
import lateral_once as parent
import semantic_tool_timed as backend
import host_preflight as host
from test_sustained_backend import FakeClock,FakeSignals,FakeSocket
from test_sustained_parent import trace_and_history,write_trace

def identity(pid):
    return dict(pid=pid,starttime_ticks=pid+100,cmdline_sha256=hashlib.sha256(str(pid).encode()).hexdigest(),argv=['/pure/source',str(pid)],boot_id='fixture-boot')

def setup(root):
    sources={role:identity(pid)for role,pid in [('new_adapter',140),('bridge',47),('projector',97)]}
    scene=identity(1172665);scene['starttime_ticks']=9427026
    a=dict(Id='a'*64,Image=c.PRIVATE_IMAGE,pidns='pid:new',netns='net:new')
    att=root/'container.json';att.write_text(json.dumps(a))
    case=dict(private_network_endpoint=True,command_endpoint=list(c.PRIVATE_ENDPOINT),container_id='a'*64,container_attestation_artifact=dict(path=str(att),sha256=c.sha(att)),source_identities=sources,scene_instance_id=scene)
    proof=dict(state='PASS',scene_identity=scene,boot_id='fixture-boot',uid=1000,container_id='a'*64,container_image=c.PRIVATE_IMAGE,container_pid_namespace='pid:new',network_namespace='net:new',container_network_namespace='net:new',host_network_namespace='net:host',shared_host_network=False,private_bridge_verified=True,command_endpoint=list(c.PRIVATE_ENDPOINT),container_attestation_sha256=c.sha(att),container_sources=sources,port7447_exact_scene_owner=True,retired_workers_verified=True,forbidden_live_processes=[],artifacts_verified=[{'fixture':True}],collector_sha256=c.sha(c.HERE/'host_preflight.py'),completed_unix_ns=1_000_000_000)
    pp=root/'proof.json';pp.write_text(json.dumps(proof))
    release=dict(allow_private_network_endpoint=True,command_endpoint=list(c.PRIVATE_ENDPOINT),boot_id='fixture-boot',network_namespace='net:new',controller_pid_namespace='pid:new',host_preflight=dict(mode='root_external_host_attestation',path=str(pp),sha256=c.sha(pp),max_age_sec=30.))
    return case,release,proof,pp

class PrivateEndpoint(unittest.TestCase):
    def test_explicit_case_root_endpoint_and_old_default(self):
        self.assertEqual(c.command_endpoint({},{}),c.LEGACY_ENDPOINT)
        case=dict(private_network_endpoint=True,command_endpoint=list(c.PRIVATE_ENDPOINT));release=dict(allow_private_network_endpoint=True,command_endpoint=list(c.PRIVATE_ENDPOINT))
        self.assertEqual(c.command_endpoint(case,release),c.PRIVATE_ENDPOINT)
        for target in [['127.0.0.1',7447],['192.168.1.218',7447],['192.168.1.219',7448],['192.168.1.219',True],['192.168.1.219',7447.],['localhost',7447]]:
            with self.subTest(target=target),self.assertRaises(c.Hold):c.command_endpoint(dict(case,command_endpoint=target),dict(release,command_endpoint=target))
        for flag in [None,0,1,'true']:
            with self.assertRaises(c.Hold):c.command_endpoint(dict(case,private_network_endpoint=flag),release)
        with self.assertRaises(c.Hold):c.command_endpoint(case,{})
        with self.assertRaises(c.Hold):c.command_endpoint({},release)
    def test_real_private_host_proof_and_wrong_identity_topology_target(self):
        with tempfile.TemporaryDirectory()as name:
            case,release,proof,path=setup(Path(name));self.assertEqual(c.host_proof(release,case,2_000_000_000),proof)
            changes=[dict(shared_host_network=True),dict(host_network_namespace='net:new'),dict(container_pid_namespace='pid:old'),dict(container_network_namespace='net:other'),dict(container_image='wrong'),dict(command_endpoint=list(c.LEGACY_ENDPOINT)),dict(container_attestation_sha256='0'*64),dict(collector_sha256='0'*64),dict(container_sources={'new_adapter':identity(5237),'bridge':identity(47),'projector':identity(97)})]
            for change in changes:
                value=dict(proof,**change);path.write_text(json.dumps(value));r=copy.deepcopy(release);r['host_preflight']['sha256']=c.sha(path)
                with self.subTest(change=change),self.assertRaises(c.Hold):c.host_proof(r,case,2_000_000_000)
    def test_proof_sha_freshness_and_partial_scene_cannot_be_relabelled(self):
        with tempfile.TemporaryDirectory()as name:
            case,release,proof,path=setup(Path(name))
            with self.assertRaises(c.Hold):c.host_proof(release,case,31_000_000_001)
            path.write_text(json.dumps(dict(proof,uid=0)))
            with self.assertRaises(c.Hold):c.host_proof(release,case,2_000_000_000)
            case['scene_instance_id']={'pid':1172665,'starttime_ticks':9427026};proof['scene_identity']=case['scene_instance_id'];path.write_text(json.dumps(proof));release['host_preflight']['sha256']=c.sha(path)
            with self.assertRaises(c.Hold):c.host_proof(release,case,2_000_000_000)
    def test_fake_private_nonzero_and_finally_neutral_target_same_and_default_old(self):
        for target in [c.LEGACY_ENDPOINT,c.PRIVATE_ENDPOINT]:
            for sign in [.4,-.4]:
                clock=FakeClock();sock=FakeSocket(clock);stream=io.StringIO()
                result=backend.run_probe(sign,1.5,lambda:sock,lambda:None,stream,clock,FakeSignals(),target)
                self.assertTrue(result['completed']);self.assertEqual(result['motion_packets_sent'],30);self.assertEqual(result['final_zero_logged_successes'],5)
                self.assertTrue(all(t==target for command,t,ns in sock.calls));self.assertTrue(all(row['target']==list(target)for row in map(json.loads,stream.getvalue().splitlines())))
                self.assertTrue(all(command['forward']==command['yaw']==0 for command,t,ns in sock.calls))
    def test_parent_trace_and_metrics_require_selected_target(self):
        with tempfile.TemporaryDirectory()as name:
            rows,history=trace_and_history(magnitude=.4,duration=1.5)
            for row in rows:row['target']=list(c.PRIVATE_ENDPOINT)
            path=Path(name)/'trace';write_trace(path,rows)
            with self.assertRaises(c.Hold):parent.semantic_sends(path,.4)
            self.assertEqual(parent.metrics(history,path,.4,c.PRIVATE_ENDPOINT)['state'],'MEASURED_SHORT_PROBE')
            measured=parent.metrics(history,path,.4,c.PRIVATE_ENDPOINT)
            self.assertEqual(parent.sustained_classification(history,path,.4,measured,history[-1]['stamp_ns'],c.PRIVATE_ENDPOINT)['state'],'MEASURED_SUSTAINED_LATERAL_PROBE')
            rows[0]['target']=list(c.LEGACY_ENDPOINT);write_trace(path,rows)
            with self.assertRaises(c.Hold):parent.semantic_sends(path,.4,c.PRIVATE_ENDPOINT)
    def collector(self,root,private=True,network_override=None):
        case,release,proof,path=setup(root);boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip();scene=dict(case['scene_instance_id'],boot_id=boot);init=dict(identity(800),pid_namespace='pid:new')
        artifact=root/'artifact';artifact.write_text('fixture')
        cfg=dict(root_inputs_reviewed=True,expected_host=dict(uid=1000,boot_id=boot,hostname='host-fixture',pid_namespace='pid:host'),expected_scene_identity=scene,expected_container_init=init,expected_container_id='a'*64,retired_workers=[dict(role=n,identity={})for n in ['63','123','2689']],artifacts=[dict(path=str(artifact),sha256=c.sha(artifact))])
        network='bridge'if private else'host';container=dict(id='a'*64,pid=800,running=True,network=network)
        if private:cfg.update(private_network_endpoint=True,command_endpoint=list(c.PRIVATE_ENDPOINT),container_attestation_artifact=case['container_attestation_artifact'],expected_container_sources=case['source_identities']);container['image']=c.PRIVATE_IMAGE
        if network_override is not None:container['network']=network_override
        def ns(p):
            if str(p)=='/proc/self/ns/pid':return'pid:host'
            if str(p)=='/proc/self/ns/net':return'net:host'
            return'net:new'if private else'net:host'
        ports=[dict(protocol=p,local='00000000:1D17',owned_by_frozen_scene=True)for p in ['tcp','udp']]
        with mock.patch.object(host.os,'getuid',return_value=1000),mock.patch.object(host.socket,'gethostname',return_value='host-fixture'),mock.patch.object(host.os,'readlink',side_effect=ns),mock.patch.object(host,'exact',side_effect=lambda expected:expected),mock.patch.object(host,'retired',return_value={'state':'ABSENT'}),mock.patch.object(host,'port_owner',return_value=ports),mock.patch.object(host,'forbidden_live',return_value=[]),mock.patch.object(host.subprocess,'check_output',return_value=json.dumps(container).encode())as docker,mock.patch.object(host,'private_sources',return_value=(case['source_identities'],{}))as sources:
            result=host.collect(cfg)
        return result,docker.call_args,sources.call_count
    def test_actual_collector_private_has_distinct_host_and_container_ns(self):
        with tempfile.TemporaryDirectory()as name:
            proof,command,calls=self.collector(Path(name));self.assertFalse(proof['shared_host_network']);self.assertNotEqual(proof['host_network_namespace'],proof['container_network_namespace']);self.assertEqual(proof['network_namespace'],'net:new');self.assertEqual(proof['command_endpoint'],list(c.PRIVATE_ENDPOINT));self.assertEqual(calls,2);self.assertEqual(command.args[0][:2],['/usr/bin/docker','inspect'])
        with tempfile.TemporaryDirectory()as name,self.assertRaises(c.Hold):self.collector(Path(name),network_override='host')
    def test_actual_collector_legacy_default_stays_shared(self):
        with tempfile.TemporaryDirectory()as name:
            proof,command,calls=self.collector(Path(name),False);self.assertTrue(proof['shared_host_network']);self.assertEqual(proof['network_namespace'],'net:host');self.assertNotIn('container_network_namespace',proof);self.assertEqual(calls,0)
    def test_actual_host_nspid_translation_and_source_change_rejection(self):
        expected={role:identity(pid)for role,pid in [('new_adapter',140),('bridge',47),('projector',97)]};mapping={1140:140,1047:47,1097:97}
        class Proc:
            def __init__(self,pid):self.pid=pid;self.name=str(pid)
            def __truediv__(self,key):
                if key=='status':return type('Status',(),{'read_text':lambda unused:'Uid:\t1000 1000 1000 1000\nNSpid:\t%d %d\n'%(self.pid,mapping[self.pid])})()
                return Path('/pure')/self.name/key
        proc=type('Root',(),{'iterdir':lambda unused:[Proc(pid)for pid in mapping]})()
        def actual(pid):return dict(identity(mapping[pid]),pid=pid,state='S')
        with mock.patch.object(host.os,'readlink',side_effect=lambda p:'pid:new'if str(p).endswith('/pid')else'net:new'),mock.patch.object(host,'identity',side_effect=actual):
            found,hosts=host.private_sources(expected,'pid:new','net:new',proc);self.assertEqual(found,expected);self.assertEqual(hosts['new_adapter']['pid'],1140)
            bad=copy.deepcopy(expected);bad['new_adapter']['cmdline_sha256']='0'*64
            with self.assertRaises(c.Hold):host.private_sources(bad,'pid:new','net:new',proc)

if __name__=='__main__':unittest.main(verbosity=2)
