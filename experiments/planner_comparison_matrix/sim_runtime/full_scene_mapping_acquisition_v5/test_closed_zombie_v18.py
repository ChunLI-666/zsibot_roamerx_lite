"""Pure proc fixtures. No real socket/process/signal/Node/remote operation."""
import errno
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import full_scene_mapping_contract as c
from test_v4_core import BASE


def fixture(root,pid=549,state='Z',threads=1,ticks=15557163):
    (root/'self/net').mkdir(parents=True,exist_ok=True)
    for name in ['udp','udp6']:(root/'self/net'/name).write_text('sl local remote state tx time retrans uid timeout inode\n')
    (root/'sys/kernel/random').mkdir(parents=True,exist_ok=True);(root/'sys/kernel/random/boot_id').write_text('same-boot\n')
    directory=root/str(pid);(directory/'fd').mkdir(parents=True);(directory/'task'/str(pid)).mkdir(parents=True)
    write_stat(directory,state,threads,ticks)
    (directory/'status').write_text(f'Name:\tpure\nState:\t{state} (fixture)\nTgid:\t{pid}\nPid:\t{pid}\nThreads:\t{threads}\n')
    return directory

def write_stat(directory,state='Z',threads=1,ticks=15557163):
    fields=['0']*20;fields[0]=state;fields[17]=str(threads);fields[19]=str(ticks)
    (directory/'stat').write_text(directory.name+' (pure process) '+' '.join(fields)+'\n')

def denied_fd(path,mutate=None):
    original=Path.iterdir
    def iterdir(value):
        if value==path:
            if mutate is not None:mutate()
            raise PermissionError(errno.EACCES,'fixture fd denied',str(value))
        return original(value)
    return mock.patch.object(Path,'iterdir',iterdir)


class ClosedZombie(unittest.TestCase):
    def test_red_old_rejects_green_new_proves_single_Z_permission_denied(self):
        path=BASE/'full_scene_mapping_camera_refresh_admission_candidate_v17/review_source_v17/full_scene_mapping_contract.py'
        spec=importlib.util.spec_from_file_location('old17_zombie_pure_contract',path);old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
        with tempfile.TemporaryDirectory()as temp:
            root=Path(temp);directory=fixture(root)
            with denied_fd(directory/'fd'):
                with self.assertRaises(old.Hold):old.udp_inventory(root)
                inventory=c.udp_inventory(root)
            self.assertEqual(list(inventory),[]);self.assertEqual(len(inventory.closed_zombies),1)
            record=inventory.closed_zombies[0];self.assertEqual(record['event'],'closed_zombie');self.assertEqual(record['birth']['starttime_ticks'],15557163)
            self.assertEqual(record['before'],record['after']);self.assertEqual(record['before']['task_pids'],[549]);self.assertEqual(record['fd_entries_observed'],0)
            self.assertIn('permission_denied',record['fd_access']);self.assertTrue(c.verify_udp_inventory(inventory,[],[]))

    def test_readable_closed_Z_requires_empty_fd_and_records_evidence(self):
        with tempfile.TemporaryDirectory()as temp:
            root=Path(temp);fixture(root);result=c.udp_inventory(root)
            self.assertEqual(result.closed_zombies[0]['fd_access'],'readable_empty');self.assertEqual(result,[])

    def test_live_permission_error_is_still_HOLD(self):
        with tempfile.TemporaryDirectory()as temp:
            root=Path(temp);directory=fixture(root,state='S',threads=12)
            with denied_fd(directory/'fd'),self.assertRaises(c.Hold):c.udp_inventory(root)

    def test_zombie_leader_live_worker_or_multi_threads_never_skipped(self):
        for threads,extra in [(2,True),(2,False),(1,True)]:
            with self.subTest(threads=threads,extra=extra),tempfile.TemporaryDirectory()as temp:
                root=Path(temp);directory=fixture(root,threads=threads)
                if extra:(directory/'task/550').mkdir()
                with denied_fd(directory/'fd'),self.assertRaises(c.Hold):c.udp_inventory(root)

    def test_stat_status_conflict_incomplete_and_wrong_Tgid_are_HOLD(self):
        for text in ['State:\tS\nPid:\t549\nTgid:\t549\nThreads:\t1\n','State:\tZ\nPid:\t549\nTgid:\t549\n',
                     'State:\tZ\nPid:\t549\nTgid:\t550\nThreads:\t1\n','State:\tZ\nPid:\t549\nTgid:\t549\nThreads:\t2\n']:
            with self.subTest(text=text),tempfile.TemporaryDirectory()as temp:
                root=Path(temp);directory=fixture(root);(directory/'status').write_text(text)
                with denied_fd(directory/'fd'),self.assertRaises(c.Hold):c.udp_inventory(root)

    def test_pid_birth_or_boot_changes_around_permission_error_are_HOLD(self):
        for changed in ['birth','boot','live']:
            with self.subTest(changed=changed),tempfile.TemporaryDirectory()as temp:
                root=Path(temp);directory=fixture(root)
                def mutate():
                    if changed=='birth':write_stat(directory,ticks=15557164)
                    elif changed=='boot':(root/'sys/kernel/random/boot_id').write_text('different-boot')
                    else:write_stat(directory,state='S')
                with denied_fd(directory/'fd',mutate),self.assertRaises(c.Hold):c.udp_inventory(root)

    def test_readable_nonempty_and_partial_read_then_denied_are_HOLD(self):
        with tempfile.TemporaryDirectory()as temp:
            root=Path(temp);directory=fixture(root);(directory/'fd/3').touch()
            with self.assertRaises(c.Hold):c.udp_inventory(root)
            original=Path.iterdir
            def partial(path):
                if path!=directory/'fd':return original(path)
                def rows():
                    yield directory/'fd/3'
                    raise PermissionError('partial fixture')
                return rows()
            with mock.patch.object(Path,'iterdir',partial),self.assertRaises(c.Hold):c.udp_inventory(root)

    def test_status_task_or_other_fd_read_failure_still_HOLD(self):
        for failed in ['status','task','fd_EIO']:
            with self.subTest(failed=failed),tempfile.TemporaryDirectory()as temp:
                root=Path(temp);directory=fixture(root)
                if failed=='status':
                    (directory/'status').unlink()
                    with self.assertRaises(c.Hold):c.udp_inventory(root)
                elif failed=='task':
                    (directory/'task/549').rmdir();(directory/'task').rmdir()
                    with self.assertRaises(c.Hold):c.udp_inventory(root)
                else:
                    original=Path.iterdir
                    def broken(path):
                        if path==directory/'fd':raise OSError(errno.EIO,'fixture IO error')
                        return original(path)
                    with mock.patch.object(Path,'iterdir',broken),self.assertRaises(c.Hold):c.udp_inventory(root)

    def test_closed_Z_never_authorizes_unknown_UDP_and_foreign_writer_still_rejected(self):
        with tempfile.TemporaryDirectory()as temp:
            root=Path(temp);directory=fixture(root)
            (root/'self/net/udp').write_text('header\n0: 00000000:1234 00000000:0000 07 0 0 0 1000 0 888\n')
            with denied_fd(directory/'fd'):inventory=c.udp_inventory(root)
            self.assertEqual(inventory[0]['owners'],[])
            with self.assertRaises(c.Hold):c.verify_udp_inventory(inventory,[],[])
            live=fixture(root,pid=550,state='S',threads=8)
            # fixture writes table header, restore exact UDP inode fixture.
            (root/'self/net/udp').write_text('header\n0: 00000000:1234 00000000:0000 07 0 0 0 1000 0 888\n')
            os.symlink('socket:[888]',live/'fd/3')
            with denied_fd(directory/'fd'):inventory=c.udp_inventory(root)
            with self.assertRaises(c.Hold):c.verify_udp_inventory(inventory,[],[],reader=lambda pid:{'pid':pid})
            allowed={'pid':550};self.assertTrue(c.verify_udp_inventory(inventory,[allowed],[],reader=lambda pid:allowed))

    def test_both_initial_and_periodic_owner_paths_use_this_same_inventory(self):
        source=(Path(__file__).parent/'full_scene_mapping.py').read_text()
        self.assertEqual(source.count('contract.udp_inventory()'),2)
        self.assertEqual(source.count('contract.verify_udp_inventory(contract.udp_inventory()'),2)

if __name__=='__main__':unittest.main(verbosity=2)
