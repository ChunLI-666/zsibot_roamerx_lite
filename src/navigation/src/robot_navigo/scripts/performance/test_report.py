import unittest
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest import mock
from xml.etree import ElementTree

import report as report_module
from report import parse_collapsed, summarize, svg, sampled_profile_info


class ReportTests(unittest.TestCase):
    def test_duplicate_stacks_and_recursion_do_not_inflate_totals(self):
        stacks = parse_collapsed('root;f;f 2\nroot;f;f 3\nroot;other 5\n')
        report = summarize(stacks)
        self.assertEqual(report['total_weight'], 10)
        values = {row['function']: row['weight'] for row in report['inclusive']}
        self.assertEqual(values, dict(root=10, f=5, other=5))
        self.assertEqual(sum(row['weight'] for row in report['self']), 10)

    def test_untrusted_symbols_are_escaped_in_svg(self):
        document = svg(parse_collapsed('root;vector<x>&"foo" 1\n'), '<CPU & heap>', 'samples')
        ElementTree.fromstring(document)
        self.assertIn('vector&lt;x&gt;&amp;', document)
        self.assertNotIn('<CPU & heap>', document)

    def test_empty_or_invalid_weights_are_rejected(self):
        for content in ('', 'root 0', 'root nan', 'root inf', 'root -1'):
            with self.subTest(content=content), self.assertRaises(ValueError):
                parse_collapsed(content)


class SampledReportTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.root=Path(self.temporary.name)
        self.binary=self.root/'native_app';self.binary.write_bytes(b'identity fixture')
        self.profile=self.root/'sample.heap';self.metadata=self.root/'sample.json'
        header='heap profile: 1: 4096 [1: 4096] @ heap_v2/524288'
        self.text=header+'\n1: 4096 [1: 4096] @ 0x100\nMAPPED_LIBRARIES:\n'+str(self.binary)+'\n'
        self.profile.write_text(self.text)
        self.witness=dict(valid=True,kind='sampled_live_heap_estimate',raw_header=header,
            profile_bytes=len(self.text.encode()),sample_parameter=524288,executable=str(self.binary),
            full_heap_profiler_running=False,cumulative_allocations_available=False)
        self.metadata.write_text(json.dumps(self.witness))

    def tearDown(self):
        self.temporary.cleanup()

    def argv(self,kind):
        return ['report.py','--profile',str(self.profile),'--binary',str(self.binary),
            '--sample-metadata',str(self.metadata),'--kind',kind,'--output',str(self.root/'report')]

    def test_v2_retains_raw_sample_amount_separate_from_corrected_estimates(self):
        data=sampled_profile_info(self.profile,self.metadata,self.binary)
        self.assertEqual(data['raw_sampled_live_bytes'],4096)
        self.assertEqual(data['sample_period_bytes'],524288)
        self.assertFalse(data['cumulative_allocations_available'])

    def test_v2_is_never_rendered_as_alloc_or_cpu(self):
        for kind in ['alloc','cpu']:
            with self.subTest(kind=kind), mock.patch('sys.argv',self.argv(kind)), \
                    mock.patch.object(report_module.subprocess,'run') as command, self.assertRaises(SystemExit) as error:
                report_module.main()
            self.assertEqual(error.exception.code,2)
            command.assert_not_called()
            self.assertFalse((self.root/'report').exists())

    def test_empty_truncated_or_wrong_identity_samples_fail_closed(self):
        for fault in ['empty','truncated','identity']:
            self.profile.write_text(self.text);self.metadata.write_text(json.dumps(self.witness))
            if fault=='empty':
                self.profile.write_text('heap profile: 0: 0 [0: 0] @ heap_v2/524288\nMAPPED_LIBRARIES:\n')
            elif fault=='truncated':
                self.profile.write_text(self.text.replace('1: 4096 [1: 4096] @ 0x100',''))
            else:
                self.metadata.write_text(json.dumps(dict(self.witness,executable='/wrong/native')))
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                sampled_profile_info(self.profile,self.metadata,self.binary)

    def test_report_uses_confirmed_pprof_v2_inuse_correction_and_estimated_units(self):
        completed=SimpleNamespace(returncode=0,stdout='root;retained 526339\n',
            stderr='Adjusting heap profiles for 1-in-524288 sampling rate\nHeap version 2\n')
        with mock.patch('sys.argv',self.argv('sampled_inuse')), \
                mock.patch.object(report_module.subprocess,'run',return_value=completed) as command:
            report_module.main()
        for call in command.call_args_list:
            self.assertIn('--inuse_space',call.args[0]);self.assertNotIn('--alloc_space',call.args[0])
        data=json.loads((self.root/'report/summary.json').read_text())
        self.assertEqual(data['kind'],'sampled_inuse')
        self.assertIn('estimated live',data['units'])
        self.assertEqual(data['sampled_live_heap']['raw_sampled_live_bytes'],4096)
        self.assertEqual(data['total_weight'],526339)

    def test_unconfirmed_correction_and_unproven_collapsed_sample_are_rejected(self):
        completed=SimpleNamespace(returncode=0,stdout='root 4096\n',stderr='')
        with mock.patch('sys.argv',self.argv('sampled_inuse')), \
                mock.patch.object(report_module.subprocess,'run',return_value=completed), self.assertRaises(RuntimeError):
            report_module.main()
        self.assertFalse((self.root/'report/flame.svg').exists())


if __name__ == '__main__':
    unittest.main()
