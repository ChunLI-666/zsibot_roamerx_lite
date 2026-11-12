"""Receipt mutation and actual loaded-binary binding tests; no ROS entities."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import waypoint_runtime as w


class BindingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.prefix = self.root/'install'
        self.source = self.root/'source'
        self.files = {}
        self.sources = {}
        for root, names, output in ((self.prefix, [w.EXECUTABLE, w.CORE], self.files),
                                    (self.source, w.SOURCE_FILES, self.sources)):
            for name in names:
                path = root/name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(name)
                output[name] = w.sha(path)
        self.data = dict(schema='matrix.waypoint.build_receipt.v1', install_prefix=str(self.prefix),
                         source_files_match_commit=True, repository_commit='a'*40,
                         upstream_commit=w.UPSTREAM_COMMIT, artifact_sha256=self.files,
                         source_sha256=self.sources, underlay_sha256={n:w.sha(n) for n in w.UNDERLAY_FILES})
        self.receipt = self.root/'receipt.json'
        self.save()

    def save(self):
        self.receipt.write_text(json.dumps(self.data))

    def binding(self):
        return w.artifacts(self.prefix, self.receipt, self.source)

    def test_explicit_pair_required_before_launcher_preparation(self):
        import matrix_navigation as launcher
        from types import SimpleNamespace
        for prefix, receipt in ((None, None), (self.prefix, None), (None, self.receipt)):
            with self.assertRaisesRegex(ValueError, 'requires'):
                w.artifacts(prefix, receipt, self.source)
        with self.assertRaisesRegex(ValueError, 'requires'):
            launcher.prepare(SimpleNamespace(with_waypoints=True))

    def test_exact_binding(self):
        binding = self.binding()
        self.assertEqual(binding['artifact_sha256'], self.files)
        self.assertEqual(binding['receipt_sha256'], w.sha(self.receipt))

    def test_missing_and_changed_artifact_or_source_rejected(self):
        for key, names in (('artifact_sha256', [w.CORE]), ('source_sha256', [w.SOURCE_FILES[0]])):
            old = copy.deepcopy(self.data)
            self.data[key].pop(names[0])
            self.save()
            with self.assertRaises(ValueError):
                self.binding()
            self.data = old
        self.save()
        (self.source/w.SOURCE_FILES[0]).write_text('changed')
        with self.assertRaises(ValueError):
            self.binding()

    def test_wrong_contract_commit_or_prefix_rejected(self):
        for key, value in (('upstream_commit', 'b'*40), ('repository_commit', 'short'),
                           ('source_files_match_commit', False), ('install_prefix', str(self.root))):
            old = self.data[key]
            self.data[key] = value
            self.save()
            with self.assertRaises(ValueError):
                self.binding()
            self.data[key] = old

    def test_source_and_artifact_symlink_escape_rejected(self):
        for root, name in ((self.prefix, w.CORE), (self.source, w.SOURCE_FILES[0])):
            path = root/name
            outside = self.root/('outside'+root.name)
            outside.write_bytes(path.read_bytes())
            path.unlink()
            path.symlink_to(outside)
            with self.assertRaises(ValueError):
                self.binding()

    def test_changed_client_contract_rejected(self):
        self.data['underlay_sha256'][w.UNDERLAY_FILES[-1]] = '0'*64
        self.save()
        with self.assertRaisesRegex(ValueError, 'contract changed'):
            self.binding()

    def test_actual_mapped_core_and_executable_required(self):
        binding = self.binding()
        identity = dict(selected_libraries_sha256={str(self.prefix/w.CORE):self.files[w.CORE]}, maps='')
        w.verify_runtime(binding, identity, self.prefix/w.EXECUTABLE)
        for mutation in ('missing', 'wrong_hash', 'legacy_outside_prefix', 'wrong_executable'):
            item = copy.deepcopy(identity)
            if mutation == 'missing': item['selected_libraries_sha256'].clear()
            if mutation == 'wrong_hash': item['selected_libraries_sha256'][str(self.prefix/w.CORE)] = '0'*64
            if mutation == 'legacy_outside_prefix': item['maps'] = '/tmp/other/libwaypoint_follower_core.so'
            executable = self.prefix/(w.CORE if mutation == 'wrong_executable' else w.EXECUTABLE)
            with self.assertRaises(RuntimeError):
                w.verify_runtime(binding, item, executable)


if __name__ == '__main__':
    unittest.main()
