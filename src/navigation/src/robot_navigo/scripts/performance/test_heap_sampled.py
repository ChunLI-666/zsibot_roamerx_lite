"""Fail-closed sampled-heap contracts; optional native fixture uses external build."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import run as runner


class SampledHeapTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.application = self.root/'native_app'
        self.library, self.helper = self.root/'libtcmalloc.so.4', self.root/'liblocnav_heap_sample.so'
        for path in [self.application,self.library,self.helper]:
            path.write_bytes(b'fixture only, ELF reader is mocked'); path.chmod(0o755)
        self.manifest = self.root/'build_manifest.json'
        self.build = dict(schema='locnav.heap_sampled.build.v1',success=True,finished_unix=1,
            library_sha256=runner.file_identity(self.library)['sha256'],
            headers_sha256={'malloc_extension.h':'a'*64,'heap-profiler.h':'b'*64},
            artifacts_sha256={str(self.helper):runner.file_identity(self.helper)['sha256']})
        self.manifest.write_text(json.dumps(self.build))
        self.args = SimpleNamespace(command=[str(self.application)],heap_lib=self.library,
            heap_sampled_helper=self.helper,heap_sampled_manifest=self.manifest,backend='heap_sampled')

    def tearDown(self):
        self.temporary.cleanup()

    def elf(self,path):
        path=Path(path)
        return dict(bits=64,byte_order=1,machine=62,arm_float_abi=None,
            type=2 if path in [self.application,Path('/proc/self/exe')] else 3,
            interpreter='/lib/ld.so' if path==self.application else None,
            soname='libtcmalloc.so.4' if path==self.library else None,
            needed=['libtcmalloc.so.4'] if path==self.helper else [])

    def test_disabled_exec_precedes_all_sampled_dependencies_and_launcher_checks(self):
        output=self.root/'unused'
        command=[sys.executable,str(Path(runner.__file__)), '--backend','heap_sampled','--output',str(output),
            '--heap-lib','/abs/missing.so','--heap-sampled-helper','/abs/missing_helper.so',
            '--heap-sampled-manifest','/abs/missing.json','--',sys.executable,'-c','raise SystemExit(7)']
        result=subprocess.run(command,env=dict(os.environ,LOCNAV_PROFILE='0'),capture_output=True)
        self.assertEqual(result.returncode,7,result.stderr)
        self.assertFalse(output.exists())

    def test_resources_does_not_consult_sampled_dependencies_or_change_environment(self):
        self.args.backend='resources'; env={'LD_PRELOAD':'original'}
        with mock.patch.object(runner,'sampled_heap_setup',side_effect=AssertionError('unexpected validation')):
            self.assertEqual(runner.profiler_command(self.args,self.root,env),self.args.command)
        self.assertEqual(env,{'LD_PRELOAD':'original'})

    def test_manifest_and_plain_needed_library_match(self):
        with mock.patch.object(runner,'elf_info',side_effect=self.elf):
            setup=runner.sampled_heap_setup(self.args)
        self.assertEqual(setup['sample_parameter'],524288)
        self.assertEqual(setup['executable'],str(self.application))

    def test_hash_mismatch_missing_manifest_and_unknown_schema_are_rejected(self):
        for field in ['library_sha256','artifacts_sha256','schema','headers_sha256']:
            with self.subTest(field=field):
                bad=dict(self.build)
                bad[field]={'artifacts_sha256':{str(self.helper):'0'*64},'headers_sha256':{},
                    'schema':'unknown','library_sha256':'0'*64}[field]
                self.manifest.write_text(json.dumps(bad))
                with mock.patch.object(runner,'elf_info',side_effect=self.elf), self.assertRaises(RuntimeError):
                    runner.sampled_heap_setup(self.args)

    def test_wrong_machine_static_binary_or_combined_allocator_are_rejected(self):
        for fault in ['machine','static','combined']:
            def info(path):
                data=self.elf(path)
                if fault=='machine' and Path(path)==self.helper: data['machine']=183
                if fault=='static' and Path(path)==self.application: data['interpreter']=None
                if fault=='combined' and Path(path)==self.library: data['soname']='libtcmalloc_and_profiler.so.4'
                return data
            with self.subTest(fault=fault), mock.patch.object(runner,'elf_info',side_effect=info), self.assertRaises(RuntimeError):
                runner.sampled_heap_setup(self.args)

    def test_direct_interpreter_launcher_is_rejected_before_elf_inspection(self):
        self.args.command=[sys.executable,'-c','pass']
        with mock.patch.object(runner,'elf_info',side_effect=AssertionError('must reject first')), self.assertRaises(RuntimeError):
            runner.sampled_heap_setup(self.args)

    def test_sampled_environment_never_enables_full_allocation_or_cpu_tracking(self):
        env={'HEAPPROFILE':'old','HEAPCHECK':'old','CPUPROFILE':'old','TCMALLOC_RELEASE_RATE':'9'}
        with mock.patch.object(runner,'elf_info',side_effect=self.elf):
            runner.sampled_heap_command(self.args,self.root,env)
        self.assertEqual(env['LD_PRELOAD'],str(self.library)+':'+str(self.helper))
        self.assertEqual(env['LOCNAV_HEAP_SAMPLE_EXPECT_EXE'],str(self.application))
        self.assertEqual(env['TCMALLOC_SAMPLE_PARAMETER'],'524288')
        self.assertEqual(env['LOCNAV_HEAP_SAMPLE_INTERVAL_SECONDS'],'10')
        self.assertFalse(any(name in env for name in ['HEAPPROFILE','HEAPCHECK','CPUPROFILE','TCMALLOC_RELEASE_RATE']))

    def test_inherited_preload_is_not_silently_mixed(self):
        with self.assertRaises(RuntimeError):
            runner.sampled_heap_command(self.args,self.root,{'LD_PRELOAD':'unrelated.so'})

    def test_elf_reader_uses_actual_host_and_rejects_non_elf(self):
        info=runner.elf_info('/proc/self/exe')
        self.assertIn(info['bits'],[32,64]); self.assertGreater(info['machine'],0)
        with self.assertRaises(RuntimeError): runner.elf_info(self.helper)


@unittest.skipUnless(all(os.environ.get(name) for name in ['LOCNAV_PROFILE_HEAP_LIB',
    'LOCNAV_PROFILE_HEAP_SAMPLED_HELPER','LOCNAV_PROFILE_HEAP_SAMPLED_MANIFEST','LOCNAV_PROFILE_HEAP_SAMPLED_FIXTURE']),
    'optional matching externally built sampled helper/fixture not supplied')
class NativeSampledHeapTests(unittest.TestCase):
    def command(self,output,option=None):
        return [sys.executable,str(Path(runner.__file__)), '--backend','heap_sampled','--output',str(output),
            '--interval','.02','--grace','1','--',os.environ['LOCNAV_PROFILE_HEAP_SAMPLED_FIXTURE'],
            *([option] if option else [])]

    def env(self):
        value=dict(os.environ,LOCNAV_PROFILE='1',LOCNAV_PROFILE_FAST_CPU='0')
        value.pop('LD_PRELOAD',None)
        return value

    def test_live_data_nonzero_exit_and_worker_join(self):
        with tempfile.TemporaryDirectory() as temporary:
            for option,expected in [(None,0),('--nonzero',7)]:
                output=Path(temporary)/('good' if option is None else 'nonzero')
                result=subprocess.run(self.command(output,option),env=self.env(),capture_output=True,timeout=10)
                self.assertEqual(result.returncode,expected,result.stderr)
                summary=json.loads((output/'summary.json').read_text())
                self.assertTrue(summary['sampled_heap']['valid'])
                self.assertTrue(summary['sampled_heap']['worker_stopped'])
                self.assertEqual(summary['remaining_owned_pids'],[])

    def test_signal_only_cleans_owned_process_and_sampler_joins(self):
        with tempfile.TemporaryDirectory() as temporary:
            output=Path(temporary)/'signal'
            unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(10)'])
            process=subprocess.Popen(self.command(output,'--wait-signal'),env=self.env(),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            try:
                deadline=time.monotonic()+5
                while len(list(output.glob('heap_sampled_profiles/*/sample_*.json')))<3 and time.monotonic()<deadline:
                    time.sleep(.01)
                self.assertEqual(len(list(output.glob('heap_sampled_profiles/*/sample_*.json'))),3)
                process.send_signal(signal.SIGINT)
                stdout,stderr=process.communicate(timeout=5)
                self.assertEqual(process.returncode,130,(stdout,stderr))
                summary=json.loads((output/'summary.json').read_text())
                self.assertEqual(summary['command_returncode'],0)
                self.assertTrue(summary['sampled_heap']['worker_stopped'])
                self.assertIsNone(unrelated.poll())
            finally:
                if process.poll() is None: process.kill(); process.communicate()
                unrelated.terminate(); unrelated.wait()


if __name__=='__main__':
    unittest.main()
