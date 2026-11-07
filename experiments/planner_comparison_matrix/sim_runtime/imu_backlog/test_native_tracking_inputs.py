"""Mocked preparation/startup/cleanup only; never starts a process or ROS."""
import argparse
import contextlib
import copy
import io
import json
import os
from pathlib import Path
import signal
import tempfile
import unittest
from unittest import mock

import yaml
import start_native_tracking_inputs as owner


class Process:
    def __init__(self,pid,argv):self.pid=pid;self.argv=argv;self.code=None
    def poll(self):return self.code
    def wait(self,timeout):self.code=0;return 0


class BootstrapTests(unittest.TestCase):
    def fixture(self,root):
        case=root/'case';(case/'controller').mkdir(parents=True)
        owner.write(case/'recording_result.json',dict(recorder_exited=True,classification='CLOSED'))
        owner.write(case/'controller/result.json',dict(stop_verified=True,relay_closed=True,classification='closed_mapping'))
        map_dir=root/'mapper/data/new_map';map_dir.mkdir(parents=True)
        (map_dir/'index.txt').write_text('0 0 0\n0 0 0 data/new_map/0.pcd\n# functional points\nstart 0 0 0 0 0 0 1\nlast 2 3 .7 0 0 .7071067811865475 .7071067811865476\n')
        (map_dir/'map_frame.yaml').write_text('native: true\n')
        provenance=root/'mapper/new_map_seed_provenance.json'
        owner.write(provenance,dict(pose_semantics='T_map_imu; xyz + qxyzw; exact SLAM index values',index_sha256=owner.sha(map_dir/'index.txt'),map_frame_sha256=owner.sha(map_dir/'map_frame.yaml'),seed=dict(name='last',pose_xyz_qxyzw=[2.,3.,.7,0.,0.,.7071067811865475,.7071067811865476],index_line_number=5,original_line=(map_dir/'index.txt').read_text().splitlines()[-1])))
        sources=[dict(pid=pid,starttime_ticks=1,cmdline_sha256='a'*64,argv=['frozen_source'],boot_id='boot') for pid in [47,97,140]]
        release=root/'source_release.json';owner.write(release,dict(allowed_udp_source_processes=sources,sensor_adapter_process_identity=sources[-1],boot_id='boot',network_namespace='net:private',controller_pid_namespace='pid:private',sim_command_endpoint=dict(ipv4='192.168.1.219',port=7447)))
        return argparse.Namespace(closed_case_dir=case,map_dir=map_dir,seed_provenance=None,source_release=release,output=root/'owner')

    def plan(self,args):
        args.output.mkdir()
        with mock.patch.object(owner,'artifacts',return_value={'pinned':'sha'}):return owner.prepare(args)

    def test_default_hold_preserves_template_and_uses_native_last_FP(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=self.fixture(Path(tmp))
            with mock.patch.object(owner,'artifacts',return_value={'pinned':'sha'}),mock.patch.object(owner.subprocess,'Popen',side_effect=AssertionError('no startup')),contextlib.redirect_stdout(io.StringIO()):
                code=owner.main(['--closed-case-dir',str(args.closed_case_dir),'--map-dir',str(args.map_dir),'--source-release',str(args.source_release),'--output',str(args.output)])
            self.assertEqual(code,0)
            self.assertEqual(owner.read(args.output/'result.json')['state'],'HOLD_PREPARED_NO_PROCESS')
            config=yaml.safe_load((args.output/'localization.yaml').read_text());old=yaml.safe_load(owner.TEMPLATE.read_text())
            self.assertEqual((config['system']['init_x'],config['system']['init_y']),(2.,3.))
            self.assertAlmostEqual(config['system']['init_yaw'],1.5707963267948966)
            self.assertFalse(config['system']['use_fp_init']);self.assertFalse(config['lidar_loc']['init_with_fp'])
            old['system'].update({key:config['system'][key] for key in ['map_path','use_fp_init','init_x','init_y','init_yaw']});old['lidar_loc']['init_with_fp']=False
            self.assertEqual(config,old)
            with self.assertRaises(FileExistsError):args.output.mkdir()

    def test_actual_controller_preferred_and_nonlast_or_changed_seed_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=self.fixture(Path(tmp));owner.write(args.closed_case_dir/'controller_result.frozen.json',dict(stop_verified=True,relay_closed=True))
            (args.closed_case_dir/'controller/result.json').write_text(json.dumps(dict(stop_verified=False,relay_closed=True)))
            with self.assertRaisesRegex(ValueError,'stop/relay'):owner.closed_inputs(args.closed_case_dir)
            path=args.map_dir.parent.parent/'new_map_seed_provenance.json';seed=owner.read(path);seed['seed']['index_line_number']=4;path.write_text(json.dumps(seed))
            with self.assertRaisesRegex(ValueError,'last native FP'):owner.native_seed(args.map_dir,path)
            seed['index_sha256']='0'*64;path.write_text(json.dumps(seed))
            with self.assertRaisesRegex(ValueError,'hashes'):owner.native_seed(args.map_dir,path)

    def test_environment_and_private_namespace_admission(self):
        parent=dict(ROS_DOMAIN_ID='185',PRIVATE_MARKER='inherited',LD_LIBRARY_PATH='adapter-overlay')
        lm=owner.environment('localization',Path('/new_output'),parent);adapter=owner.environment('adapter',Path('/new_output'),parent)
        self.assertEqual(lm['OMP_NUM_THREADS'],'2');self.assertEqual(lm['OPENBLAS_NUM_THREADS'],'1')
        self.assertTrue(lm['LD_LIBRARY_PATH'].startswith('/opt/frozen_runtime/lib:'+str(owner.RUNTIME/'lib')+':'))
        self.assertEqual(adapter['LD_LIBRARY_PATH'],'adapter-overlay');self.assertEqual(adapter['PRIVATE_MARKER'],'inherited')
        binding=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),network_namespace='net:expected',controller_pid_namespace='pid:expected')
        with mock.patch.dict(owner.os.environ,{'ROS_DOMAIN_ID':'185'}),mock.patch.object(owner.os,'readlink',return_value='wrong'):
            with self.assertRaisesRegex(ValueError,'namespace'):owner.execution_context(binding)

    def run_mocked(self,plan,output,fail_second=False):
        processes=[];handlers={}
        def popen(argv,**kwargs):
            if fail_second and processes:raise OSError('adapter startup failed')
            process=Process(1000+len(processes),argv);processes.append(process);return process
        def identity(pid):
            p=next(p for p in processes if p.pid==pid);return dict(pid=pid,argv=p.argv,executable_sha256=owner.LM_SHA)
        def handler(number,callback):handlers[number]=callback;return signal.SIG_DFL
        def sleep(seconds):handlers[signal.SIGINT](signal.SIGINT,None)
        with mock.patch.object(owner,'execution_context'),mock.patch.object(owner,'artifacts',return_value=plan['artifacts']),\
             mock.patch.object(owner.subprocess,'Popen',side_effect=popen),mock.patch.object(owner.os,'pidfd_open',side_effect=lambda pid:pid+10),\
             mock.patch.object(owner,'process_identity',side_effect=identity),mock.patch.object(owner.signal,'signal',side_effect=handler),\
             mock.patch.object(owner.signal,'pidfd_send_signal') as sends,mock.patch.object(owner.os,'close') as closes,mock.patch.object(owner.time,'sleep',side_effect=sleep):
            result=owner.execute(plan,output,14400.)
        return result,sends.call_args_list,closes.call_args_list

    def test_second_startup_failure_cleans_only_first_owned_pidfd(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=self.fixture(Path(tmp));plan=self.plan(args);result,sends,closes=self.run_mocked(plan,args.output,True)
            self.assertIn('adapter startup failed',result['error']);self.assertTrue(result['all_owned_children_closed'])
            self.assertEqual([call.args for call in sends],[(1010,signal.SIGINT)])
            self.assertEqual([call.args for call in closes],[(1010,)])

    def test_root_sigint_closes_adapter_then_LM_without_touching_source3(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=self.fixture(Path(tmp));plan=self.plan(args);result,sends,closes=self.run_mocked(plan,args.output)
            self.assertEqual(result['state'],'ROOT_STOP_INPUTS_CLOSED');self.assertTrue(result['all_owned_children_closed'])
            self.assertEqual([call.args for call in sends],[(1011,signal.SIGINT),(1010,signal.SIGINT)])
            self.assertEqual([row['role'] for row in result['children']],['adapter','localization'])
            self.assertEqual([call.args for call in closes],[(1011,),(1010,)])
            self.assertTrue((args.output/'inputs_started.json').is_file())


if __name__=='__main__':unittest.main()
