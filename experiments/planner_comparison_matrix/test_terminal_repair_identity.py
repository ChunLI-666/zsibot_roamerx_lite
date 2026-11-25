"""Exercise the launcher's real receipt validation without starting ROS actors."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import matrix_navigation as m


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ReceiptIdentity(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.nav=self.root/'nav'
        self.actual_nav=m.NAV;self.actual_base=m.BASE
        rels=[p.relative_to(self.actual_nav) for p in (self.actual_nav/'src/navigation/src/official_lattice_rpp_adapter').rglob('*') if p.is_file()]
        rels += [Path(x) for x in [
          'src/navigation/src/robot_feasible_navigation/include/robot_feasible_navigation/footprint_sweep.hpp',
          'src/navigation/src/robot_feasible_navigation/vendor/nav2_smac_planner/LICENSE.Apache-2.0',
          'experiments/official_path_handler_turn_validation/inputs/global_planner/diagnostic_small_radius_same_actions.json',
          'src/navigation/src/robot_navigo/scripts/nav_safety_gate.py']]
        rels += [Path('experiments/planner_comparison_matrix')/name for name in [
          'matrix_navigation.py','recovery_navigation.py','waypoint_runtime.py','static_layer_snapshot.py',
          'recording_deadline.py','run_matrix_navigation.sh','matrix_profile.yaml']]
        for rel in rels:
            target=self.nav/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(self.actual_nav/rel,target)
        self.base=self.nav/'experiments/planner_comparison_matrix';self.template=self.base/'matrix_profile.yaml'
        self.patch=mock.patch.multiple(m,NAV=self.nav,BASE=self.base,__file__=str(self.base/'matrix_navigation.py'));self.patch.start()
        files={p.relative_to(self.nav).as_posix():digest(p) for p in self.nav.rglob('*') if p.is_file()}
        self.snapshot=self.root/'snapshot.json';self.snapshot.write_text(json.dumps(dict(schema_version=1,
          origin=dict(kind='canonical_checkout_snapshot',canonical_repository=str(self.nav),commit='a'*40,branch='fixture',status=' M file'),files_sha256=files)))
        self.prefix=self.root/'install';self.prefix.mkdir();self.artifacts={}
        for name in ['lib/libofficial_lattice_rpp_adapter.so','share/official_lattice_rpp_adapter/plugins.xml',
                     'share/ament_index/resource_index/nav2_core__pluginlib__plugin/official_lattice_rpp_adapter']:
            path=self.prefix/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(name.encode());self.artifacts[name]=digest(path)
        for name in ('base.patch','cache.txt','configure.txt','underlay.so'):(self.root/name).write_text(name)
        self.receipt=self.root/'receipt.json';self.data=dict(repository_commit='a'*40,source_files_match_commit=False,
          source_identity_kind='working_tree_snapshot',install_prefix=str(self.prefix),source_sha256=files,
          source_snapshot_sha256=digest(self.snapshot),base_patch_file='base.patch',base_patch_sha256=digest(self.root/'base.patch'),
          artifact_sha256=self.artifacts,build_inputs=dict(commands=['cmake configure','cmake build'],
          cmake_cache_file='cache.txt',cmake_cache_sha256=digest(self.root/'cache.txt'),
          configure_log_file='configure.txt',configure_log_sha256=digest(self.root/'configure.txt'),
          underlay_sha256={str(p):digest(p) for p in [self.root/'underlay.so']+[Path('/opt/ros/jazzy/lib')/x for x in ('libnav2_costmap_2d_core.so','libnav2_regulated_pure_pursuit_controller.so','libcontroller_server_core.so','libpose_progress_checker.so')]+[Path('/opt/ros/jazzy/include/nav2_core/controller.hpp'),Path('/opt/ros/jazzy/include/nav2_core/progress_checker.hpp')]}))
        self.save()
    def tearDown(self):
        self.patch.stop();self.temp.cleanup()
    def save(self):self.receipt.write_text(json.dumps(self.data,sort_keys=True))
    def validate(self,variant='official',repair=True):
        snapshot,raw=m.source_snapshot(self.snapshot,self.template)
        return m.validate_build_receipt(self.receipt,self.prefix,variant,repair,snapshot,raw)
    def test_working_tree_receipt_and_source_are_validated(self):
        identity=self.validate();self.assertEqual(identity['repository_commit_role'],'base')
        self.assertFalse(identity['source_files_match_commit'])
    def test_old_profile_rejects_dirty_receipt(self):
        with self.assertRaises(ValueError):self.validate(repair=False)
    def test_missing_closure_is_rejected(self):
        data=json.loads(self.snapshot.read_text());name=next(x for x in data['files_sha256'] if x.endswith('terminal_stop_hold.hpp'))
        del data['files_sha256'][name];self.snapshot.write_text(json.dumps(data));self.data['source_sha256']=data['files_sha256']
        self.data['source_snapshot_sha256']=digest(self.snapshot);self.save()
        with self.assertRaises(ValueError):self.validate()
    def test_changed_source_is_rejected(self):
        path=self.nav/'src/navigation/src/official_lattice_rpp_adapter/src/plugins.cpp';path.write_bytes(path.read_bytes()+b'\n')
        with self.assertRaises(ValueError):self.validate()
    def test_changed_dso_or_metadata_is_rejected(self):
        for name in self.artifacts:
            path=self.prefix/name;original=path.read_bytes();path.write_bytes(original+b'x')
            with self.assertRaises(ValueError):self.validate()
            path.write_bytes(original)
    def test_changed_patch_or_build_input_is_rejected(self):
        for name in ['base.patch','cache.txt','configure.txt','underlay.so']:
            path=self.root/name;original=path.read_bytes();path.write_bytes(original+b'x')
            with self.assertRaises(ValueError):self.validate()
            path.write_bytes(original)
    def prepared(self):
        out=self.root/'prepared';out.mkdir()
        for path in (self.receipt,self.snapshot,self.root/'base.patch',self.root/'cache.txt',self.root/'configure.txt'):
            name='source_build_receipt.json' if path==self.receipt else 'deployment_source_snapshot.json' if path==self.snapshot else path.name
            shutil.copy2(path,out/name)
        identity=self.validate();manifest=dict(source_identity=identity,build_receipt_sha256=digest(self.receipt),
          deployment_source_snapshot=dict(sha256=digest(self.snapshot)),lifecycle_manager_runtime=None)
        args=SimpleNamespace(build_receipt=str(self.receipt),message_install='unused',source_snapshot=str(self.snapshot),
          template=str(self.template),with_waypoints=False,variant='official',terminal_settle_repair=True,
          lifecycle_install=None,lifecycle_build_receipt=None)
        return args,out,manifest
    def test_independent_waypoint_receipt_uses_matching_snapshot_bytes(self):
        snapshot,raw=m.source_snapshot(self.snapshot,self.template)
        identity=self.validate();name=next(iter(snapshot['files_sha256']))
        waypoint=dict(repository_commit='b'*40,source_sha256={name:snapshot['files_sha256'][name]})
        binding=m.validate_waypoint_source_binding(identity,snapshot,waypoint)
        self.assertEqual(binding['method'],'source_sha256');self.assertFalse(binding['adapter_source_is_base_commit'])
        waypoint['source_sha256'][name]='0'*64
        with self.assertRaises(ValueError):m.validate_waypoint_source_binding(identity,snapshot,waypoint)
        identity['source_identity_kind']='committed'
        with self.assertRaises(ValueError):m.validate_waypoint_source_binding(identity,snapshot,waypoint)
    def test_run_rechecks_receipt_before_any_actor(self):
        args,out,manifest=self.prepared();self.receipt.write_text(self.receipt.read_text()+' ')
        with mock.patch.dict(os.environ,{'ROS_DOMAIN_ID':'185'}):
            with self.assertRaisesRegex(RuntimeError,'receipt changed'):
                m.run(args,out,self.prefix,manifest,None,None)
    def test_changed_frozen_receipt_is_rejected_before_any_actor(self):
        args,out,manifest=self.prepared();path=out/'source_build_receipt.json';path.write_text(path.read_text()+' ')
        with mock.patch.dict(os.environ,{'ROS_DOMAIN_ID':'185'}):
            with self.assertRaisesRegex(RuntimeError,'receipt changed'):
                m.run(args,out,self.prefix,manifest,None,None)
    def test_run_reaches_post_identity_validation_before_any_actor(self):
        args,out,manifest=self.prepared()
        class IdentityPassed(Exception):pass
        with mock.patch.dict(os.environ,{'ROS_DOMAIN_ID':'185'}),mock.patch.object(m,'lifecycle_artifacts',side_effect=IdentityPassed):
            with self.assertRaises(IdentityPassed):m.run(args,out,self.prefix,manifest,None,None)


if __name__=='__main__':unittest.main()
