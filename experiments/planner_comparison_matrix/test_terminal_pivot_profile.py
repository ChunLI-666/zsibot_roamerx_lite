"""Real prepare-only CLI tests. Synthetic map bytes are configuration fixtures."""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
import yaml

ROOT=Path(__file__).resolve().parent
class P2PivotProfile(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.output=Path(os.environ['P2_PROFILE_TEST_OUTPUT']).resolve();cls.output.mkdir(exist_ok=False,parents=True)
        cls.install=Path(os.environ['P2_TEST_ADAPTER_INSTALL']).resolve()
        (cls.output/'map.pgm').write_bytes(b'P5\n2 2\n255\n'+bytes([254])*4)
        (cls.output/'map.yaml').write_text('image: map.pgm\nresolution: 0.01\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    def invoke(self,label,extra=(),variant='official'):
        out=self.output/label
        command=[sys.executable,'-B',str(ROOT/'matrix_navigation.py'),'--map',str(self.output/'map.yaml'),
            '--footprint','[[-0.647,-0.530],[0.647,-0.530],[0.647,0.530],[-0.647,0.530]]',
            '--padding','.15','--variant',variant,'--output',str(out),'--official-install',str(self.install),
            '--template',str(ROOT/'matrix_profile.yaml')]+list(extra)
        result=subprocess.run(command,capture_output=True,text=True,timeout=20)
        (self.output/(label+'.invocation.json')).write_text(json.dumps(dict(argv=command,returncode=result.returncode,
            stdout=result.stdout,stderr=result.stderr,run_requested=False),indent=2)+'\n')
        return result,out
    def test_normal_prepare_defaults_and_explicit_opt_in_preserve_original_acceptance(self):
        for label,value,extra in [('default',0.,[]),('explicit',.03,['--terminal-rotation-aim-yaw-tolerance','.03'])]:
            with self.subTest(label=label):
                result,out=self.invoke(label,extra);self.assertEqual(result.returncode,0,result.stderr)
                manifest=json.loads((out/'runtime_manifest.json').read_text());params=yaml.safe_load((out/'params.yaml').read_text())
                follow=params['controller_server']['ros__parameters']['FollowPath']
                self.assertTrue(follow['terminal_original_region_pivot'])
                self.assertEqual(follow['terminal_rotation_aim_yaw_tolerance'],value)
                self.assertTrue(follow['terminal_stop_before_rotation']);self.assertEqual(follow['terminal_stop_dwell_sec'],.3)
                self.assertEqual(manifest['goal_tolerances'],dict(xy=.15,yaw=.08,stateful=False))
                aim=manifest['terminal_zero_aim'];self.assertEqual(aim['enabled'],bool(value));self.assertTrue(aim['default_disabled'])
                self.assertFalse(aim['stopping_envelope_validated']);self.assertFalse(manifest['execution_requested'])
                self.assertTrue((out/'navigate.xml').is_file());self.assertFalse((out/'ready.json').exists())
    def test_unsupported_aim_and_nonofficial_opt_in_are_rejected_before_preparing(self):
        for label,extra,variant in [('unsupported',['--terminal-rotation-aim-yaw-tolerance','.02'],'official'),
                                     ('segmented',['--terminal-rotation-aim-yaw-tolerance','.03'],'segmented')]:
            with self.subTest(label=label):
                result,out=self.invoke(label,extra,variant);self.assertNotEqual(result.returncode,0)
                self.assertFalse((out/'runtime_manifest.json').exists())

if __name__=='__main__':unittest.main()
