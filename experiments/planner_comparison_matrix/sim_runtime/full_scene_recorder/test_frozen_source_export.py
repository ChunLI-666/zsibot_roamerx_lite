"""Pure temporary-export fixtures: no Git process, ROS, remote or recorder."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
spec = importlib.util.spec_from_file_location('source_export_owner_under_test', HERE/'full_scene_recording_owner.py')
owner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(owner)


def digest(data):
    return hashlib.sha256(data).hexdigest()


class FrozenSourceExportFixtures(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.root = self.directory/'export'
        (self.root/'src').mkdir(parents=True)
        (self.root/'src/worker.py').write_bytes(b'value = 7\n')
        self.origin = dict(canonical_repository_path='/canonical/repository', branch='feature/合法+branch',
                           head='a'*40, status_porcelain=' M src/worker.py\n', diff_sha256='b'*64)
        self.manifest = dict(schema='canonical_git_source_export.v1', origin=copy.deepcopy(self.origin),
                             files_sha256={'src/worker.py':digest(b'value = 7\n')})
        self.repository = dict(kind='frozen_source_export', root_reviewed=True,
                               export_root=str(self.root), manifest_path=str(self.root/'source_export.json'),
                               expected_origin=copy.deepcopy(self.origin))
        self.artifacts = {}
        self.persist()

    def persist(self, raw=None):
        path = Path(self.repository['manifest_path'])
        path.write_bytes(raw if raw is not None else (json.dumps(self.manifest, ensure_ascii=False)+'\n').encode())
        self.repository['manifest_sha256'] = owner.sha(path)
        self.artifacts = dict(source_export_manifest=dict(path=str(path), sha256=owner.sha(path), kind='source_export_manifest'))

    def verify(self):
        return owner.verify_frozen_source_export(self.repository, self.artifacts)

    def assert_hold(self):
        with self.assertRaises(owner.Hold):
            self.verify()

    def test_valid_export_and_snapshot_have_no_runtime_git_claim(self):
        config = dict(repositories=[self.repository], artifacts=self.artifacts)
        output = self.directory/'run'; output.mkdir()
        with patch.object(owner.subprocess, 'run', side_effect=AssertionError('Git must not run for export')):
            verified, raw = self.verify()
            owner.snapshot_repositories(config, output)
        self.assertFalse(verified['observed_git_on_runtime_host'])
        self.assertTrue(verified['origin_is_canonical_metadata_not_runtime_git_observation'])
        self.assertEqual(verified['origin'], self.origin)
        self.assertNotIn('head', verified)
        self.assertEqual(verified['directory_inventory'], ['source_export.json', 'src/worker.py'])
        rows = owner.read_json(output/'repositories_at_recording_start.json')
        self.assertEqual((output/rows[0]['manifest_frozen_file']).read_bytes(), raw)
        self.assertEqual(rows[0]['verified_files'][0]['sha256'], digest(b'value = 7\n'))

    def test_source_hash_drift_with_same_origin_is_rejected(self):
        (self.root/'src/worker.py').write_bytes(b'value = 8\n')
        self.assert_hold()

    def test_second_start_snapshot_rechecks_drift(self):
        self.verify()
        (self.root/'src/worker.py').write_bytes(b'value = 9\n')
        output = self.directory/'run'; output.mkdir()
        with self.assertRaises(owner.Hold):
            owner.snapshot_repositories(dict(repositories=[self.repository], artifacts=self.artifacts), output)
        self.assertFalse((output/'repositories_at_recording_start.json').exists())

    def test_manifest_hash_drift_is_rejected(self):
        Path(self.repository['manifest_path']).write_text('{}\n')
        self.assert_hold()

    def test_origin_mismatch_is_rejected(self):
        self.manifest['origin']['head'] = 'c'*40
        self.persist()
        self.assert_hold()

    def test_origin_basic_types_and_root_review_are_required(self):
        cases = [dict(head='short'), dict(branch=3), dict(branch=''), dict(status_porcelain=[]),
                 dict(diff_sha256='not a hash'), dict(canonical_repository_path='relative/repo')]
        for replacement in cases:
            with self.subTest(replacement=replacement):
                expected = copy.deepcopy(self.origin); expected.update(replacement)
                self.repository['expected_origin'] = expected
                self.manifest['origin'] = copy.deepcopy(expected)
                self.persist(); self.assert_hold()
        self.repository['expected_origin'] = copy.deepcopy(self.origin)
        self.manifest['origin'] = copy.deepcopy(self.origin); self.persist()
        self.repository['root_reviewed'] = False
        self.assert_hold()

    def test_path_escape_absolute_and_non_normalized_are_rejected(self):
        for name in ['../outside.py', '/tmp/outside.py', 'src/../../outside.py', './src/worker.py', 'src//worker.py']:
            with self.subTest(name=name):
                self.manifest['files_sha256'] = {name:digest(b'value = 7\n')}
                self.persist(); self.assert_hold()

    def test_manifest_outside_export_root_is_rejected(self):
        self.repository['manifest_path'] = str(self.directory/'outside_manifest.json')
        self.persist(); self.assert_hold()

    def test_leaf_symlink_is_rejected_even_with_matching_bytes(self):
        outside = self.directory/'outside.py'; outside.write_bytes(b'value = 7\n')
        leaf = self.root/'src/worker.py'; leaf.unlink(); leaf.symlink_to(outside)
        self.assert_hold()

    def test_directory_symlink_is_rejected(self):
        outside = self.directory/'outside'; outside.mkdir()
        (outside/'worker.py').write_bytes(b'value = 7\n')
        (self.root/'src/worker.py').unlink(); (self.root/'src').rmdir()
        (self.root/'src').symlink_to(outside, target_is_directory=True)
        self.assert_hold()

    def test_manifest_symlink_is_rejected(self):
        manifest = Path(self.repository['manifest_path'])
        outside = self.directory/'outside_manifest.json'; outside.write_bytes(manifest.read_bytes())
        manifest.unlink(); manifest.symlink_to(outside)
        self.assert_hold()

    def test_export_root_symlink_is_rejected(self):
        alias = self.directory/'export_alias'; alias.symlink_to(self.root, target_is_directory=True)
        self.repository.update(export_root=str(alias), manifest_path=str(alias/'source_export.json'))
        self.artifacts['source_export_manifest']['path'] = self.repository['manifest_path']
        self.assert_hold()

    def test_special_file_is_rejected_without_opening_fifo(self):
        os.mkfifo(self.root/'unused_fifo')
        self.assert_hold()

    def test_unlisted_source_file_is_rejected(self):
        (self.root/'extra.py').write_text('not listed\n')
        self.assert_hold()

    def test_directory_walk_permission_error_fails_closed(self):
        hidden = self.root/'hidden'; hidden.mkdir()
        (hidden/'unlisted.py').write_text('hidden participating bytes\n')
        original_scandir = owner.os.scandir
        def denied_scandir(path):
            if Path(path) == hidden:
                raise PermissionError(13, 'fixture unreadable export directory', str(path))
            return original_scandir(path)
        with patch.object(owner.os, 'scandir', side_effect=denied_scandir):
            with self.assertRaisesRegex(owner.Hold, 'directory inventory unreadable'):
                self.verify()

    def test_manifest_must_be_in_matching_complete_artifact_inventory(self):
        self.artifacts = {}; self.assert_hold()
        self.persist(); self.artifacts['source_export_manifest']['sha256'] = '0'*64
        self.assert_hold()

    def test_manifest_self_list_schema_and_duplicate_keys_are_rejected(self):
        self.manifest['files_sha256']['source_export.json'] = '0'*64
        self.persist(); self.assert_hold()
        del self.manifest['files_sha256']['source_export.json']
        self.manifest['schema'] = 'other.v1'; self.persist(); self.assert_hold()
        raw = b'{"schema":"canonical_git_source_export.v1","schema":"canonical_git_source_export.v1"}'
        self.persist(raw); self.assert_hold()

    def test_original_artifact_freeze_copies_export_manifest(self):
        self.verify()
        output = self.directory/'freeze'; output.mkdir()
        config = dict(artifacts=self.artifacts, expected_environment={}, source_processes=[], scene_instance_id='fixture')
        config_path, release_path = self.directory/'config.json', self.directory/'release.json'
        config_path.write_text(json.dumps(config)); release_path.write_text('{}')
        fake_elf = self.directory/'python_fixture'; fake_elf.write_bytes(b'ELF fixture only')
        fake_cli = self.directory/'ros2_fixture'; fake_cli.write_bytes(b'CLI fixture only')
        original_copy = owner.shutil.copyfile
        def fixture_copy(source, destination):
            return original_copy(fake_cli if str(source)=='/opt/ros/jazzy/bin/ros2' else source, destination)
        with patch.object(owner.sys, 'executable', str(fake_elf)), patch.object(owner.shutil, 'copyfile', side_effect=fixture_copy):
            owner.freeze(config, output, config_path, release_path)
        frozen = owner.read_json(output/'frozen_inputs_manifest.json')['artifacts']['source_export_manifest']
        self.assertEqual(owner.sha(output/frozen['frozen_path']), self.repository['manifest_sha256'])

    def test_legacy_git_branch_keeps_original_snapshot_behavior(self):
        head, status, diff = 'd'*40, ' M file.cpp\n', b'diff fixture\n'
        repository = dict(path='/fixture/git', expected_head=head, expected_status_porcelain=status,
                          expected_diff_sha256=digest(diff))
        for explicit_kind in (False, True):
            with self.subTest(explicit_kind=explicit_kind):
                item = dict(repository)
                if explicit_kind:item['kind']='git'
                output = self.directory/('git_explicit' if explicit_kind else 'git_legacy'); output.mkdir()
                with patch.object(owner.subprocess, 'run', side_effect=[SimpleNamespace(stdout=(head+'\n').encode()), SimpleNamespace(stdout=status.encode()), SimpleNamespace(stdout=diff)]) as mocked:
                    owner.snapshot_repositories(dict(repositories=[item]), output)
                self.assertEqual(mocked.call_count, 3)
                self.assertEqual(owner.read_json(output/'repositories_at_recording_start.json'),
                    [dict(path=repository['path'], head=head, status_porcelain=status,
                          diff_sha256=digest(diff), diff_file='repository_0.diff')])
                self.assertEqual((output/'repository_0.diff').read_bytes(), diff)


if __name__ == '__main__':
    unittest.main()
