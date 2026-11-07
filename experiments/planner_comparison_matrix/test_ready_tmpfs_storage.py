"""Run-bound RAM lease transport with real temporary tmpfs files; no ROS/Node/goal."""
import ast,hashlib,json,os,pathlib,shutil,sys,tempfile,time,unittest,uuid
from types import SimpleNamespace
from unittest import mock
HERE=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import recording_deadline
SOURCE=HERE/'matrix_navigation.py'
root=ast.parse(SOURCE.read_text());nodes=[n for n in root.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in ['write','LiveReadyStorage']]
ns=dict(os=os,pathlib=pathlib,shutil=shutil,json=json,hashlib=hashlib,time=time,process_identity=recording_deadline.process_identity)
exec(compile(ast.Module(body=nodes,type_ignores=[]),str(SOURCE),'exec'),ns);Storage=ns['LiveReadyStorage']
class ReadyStorageTests(unittest.TestCase):
    def setUp(self):
        self.ram=tempfile.TemporaryDirectory(prefix='ready-storage-test-',dir='/dev/shm');self.disk=tempfile.TemporaryDirectory(prefix='ready-storage-case-')
        self.addCleanup(self.ram.cleanup);self.addCleanup(self.disk.cleanup)
        self.out=pathlib.Path(self.disk.name);self.root=pathlib.Path(self.ram.name)
        self.manifest=dict(run_id=uuid.uuid4().hex,launcher=recording_deadline.process_identity(os.getpid()))
        self.storage=Storage(self.out,self.manifest,self.root)
    def start(self):self.storage.start();return self.storage
    def test_start_uses_private_real_tmpfs_and_preserves_exact_owner(self):
        s=self.start();self.assertTrue(s.path.is_symlink());self.assertEqual(os.readlink(s.path),str(s.live));self.assertEqual(s.slot.stat().st_mode&0o777,0o700)
        self.assertEqual({p.name for p in s.slot.iterdir()},{'owner.json','ready.json'});self.assertEqual((self.out/'ready_storage_owner.json').read_bytes(),s.owner_raw)
        self.assertEqual(s.storage['startup_tmpfs']['filesystem'],'tmpfs');self.assertEqual(json.loads(s.path.read_bytes())['ready'],False)
    def test_live_publish_never_calls_disk_write_or_replaces_case_symlink(self):
        s=self.start();value=dict(ready=True,time_ns=123,valid_until_ns=456,waypoint_ownership=dict(checked_unix_ns=12))
        original=ns['write'];ns['write']=lambda *a,**kw:(_ for _ in ()).throw(AssertionError('disk write in live publish'))
        try:s.publish(value,compact=True)
        finally:ns['write']=original
        self.assertTrue(s.path.is_symlink());self.assertEqual(json.loads(s.path.read_bytes()),value)
        self.assertEqual(json.loads(s.live.read_bytes()),value)
    def test_closed_archive_is_identical_regular_and_main_except_cannot_rewrite(self):
        s=self.start();s.publish(dict(ready=True,time_ns=100,valid_until_ns=500));s.stop_false(777);raw=s.live.read_bytes();s.stop_false(888);self.assertEqual(s.live.read_bytes(),raw)
        s.materialize();self.assertFalse(s.path.is_symlink());self.assertEqual(s.path.read_bytes(),raw);self.assertFalse(s.slot.exists())
        receipt=json.loads((self.out/'ready_storage.json').read_text());self.assertEqual(receipt['archive_sha256'],hashlib.sha256(raw).hexdigest());self.assertTrue(receipt['owned_slot_removed'])
        s.stop_false(999);s.materialize();self.assertEqual(s.path.read_bytes(),raw)
    def test_foreign_owner_prevents_write_archive_and_cleanup(self):
        s=self.start();s.owner_path.write_bytes(b'foreign')
        for fn in [lambda:s.publish(dict(ready=True)),s.stop_false,s.materialize]:
            with self.assertRaises(RuntimeError):fn()
        self.assertEqual(s.owner_path.read_bytes(),b'foreign');self.assertTrue(s.slot.exists())
    def test_existing_case_and_existing_slot_are_never_overwritten(self):
        s=self.storage;s.path.write_bytes(b'existing case')
        with self.assertRaises(FileExistsError):s.start()
        with self.assertRaises(RuntimeError):s.stop_false()
        self.assertEqual(s.path.read_bytes(),b'existing case');self.assertFalse(s.slot.exists())
        s.path.unlink();s.slot.mkdir();(s.slot/'foreign').write_bytes(b'kept')
        with self.assertRaises(FileExistsError):s.start()
        self.assertEqual((s.slot/'foreign').read_bytes(),b'kept');s.stop_false();self.assertFalse(json.loads(s.path.read_text())['ready'])
    def test_low_free_space_fails_before_business_and_leaves_false_regular(self):
        with mock.patch.object(shutil,'disk_usage',return_value=SimpleNamespace(total=1024,free=0)):
            with self.assertRaisesRegex(RuntimeError,'4MiB'):self.storage.start()
        self.storage.stop_false();self.assertFalse(self.storage.path.is_symlink());self.assertFalse(json.loads(self.storage.path.read_text())['ready']);self.assertFalse(self.storage.slot.exists())
    def test_partial_owner_write_failure_rolls_back_only_created_inodes(self):
        s=self.storage;original=pathlib.Path.open
        class Broken:
            def __init__(self,stream):self.stream=stream
            def __enter__(self):self.stream.__enter__();return self
            def fileno(self):return self.stream.fileno()
            def write(self,data):raise OSError('injected owner write failure')
            def __exit__(self,*args):return self.stream.__exit__(*args)
        def opening(path,*args,**kwargs):
            stream=original(path,*args,**kwargs)
            return Broken(stream) if path==s.owner_path and args[0]=='xb' else stream
        with mock.patch.object(pathlib.Path,'open',opening):
            with self.assertRaisesRegex(OSError,'owner write failure'):s.start()
        self.assertFalse(s.slot.exists());self.assertFalse(s.path.is_symlink());self.assertFalse(json.loads(s.path.read_text())['ready']);self.assertEqual((self.out/'ready_storage_owner.json').read_bytes(),s.owner_raw)
    def test_partial_case_symlink_failure_materializes_false_and_cleans_slot(self):
        with mock.patch.object(pathlib.Path,'symlink_to',side_effect=OSError('injected symlink failure')):
            with self.assertRaisesRegex(OSError,'symlink failure'):self.storage.start()
        s=self.storage;self.assertFalse(s.slot.exists());self.assertFalse(s.path.is_symlink());self.assertFalse(json.loads(s.path.read_text())['ready']);s.stop_false();s.materialize()
    def test_preexisting_live_temp_is_not_deleted_on_open_failure(self):
        s=self.start();temporary=s.slot/'ready.tmp';temporary.write_bytes(b'foreign temp')
        with self.assertRaises(FileExistsError):s.publish(dict(ready=True))
        self.assertEqual(temporary.read_bytes(),b'foreign temp')
    def test_materialize_replace_failure_keeps_reachable_false_ram_then_retries(self):
        s=self.start();s.stop_false();raw=s.live.read_bytes();original=pathlib.Path.replace
        def replacing(path,target):
            if path==self.out/'ready.materialize.tmp':raise OSError('injected archive replace failure')
            return original(path,target)
        with mock.patch.object(pathlib.Path,'replace',replacing):
            with self.assertRaisesRegex(OSError,'archive replace failure'):s.materialize()
        self.assertTrue(s.path.is_symlink());self.assertEqual(s.path.read_bytes(),raw);self.assertTrue(s.slot.exists());self.assertFalse((self.out/'ready.materialize.tmp').exists())
        s.materialize();self.assertFalse(s.path.is_symlink());self.assertEqual(s.path.read_bytes(),raw);self.assertFalse(s.slot.exists())
    def test_unexpected_slot_file_is_preserved_after_regular_archive(self):
        s=self.start();s.stop_false();foreign=s.slot/'foreign';foreign.write_bytes(b'kept')
        with self.assertRaisesRegex(RuntimeError,'Unexpected READY slot files'):s.materialize()
        self.assertFalse(s.path.is_symlink());self.assertFalse(json.loads(s.path.read_text())['ready']);self.assertEqual(foreign.read_bytes(),b'kept');self.assertEqual(s.owner_path.read_bytes(),s.owner_raw)
    def test_runtime_ast_routes_all_live_writes_and_archives_only_after_closed(self):
        run=next(n for n in root.body if isinstance(n,ast.FunctionDef) and n.name=='run');text=ast.unparse(run)
        self.assertNotIn("write(out / 'ready.json'",text);self.assertEqual(text.count('ready_storage.publish(readiness, compact=True)'),2)
        final=next(n.finalbody for n in run.body if isinstance(n,ast.Try) and n.finalbody);self.assertEqual(ast.unparse(final[0]),'shutdown_started = True');self.assertIn('deadline_guard.begin_shutdown(',ast.unparse(final[1]))
        lines=[ast.unparse(n) for n in final];closed=next(i for i,v in enumerate(lines) if v.startswith('deadline_guard.recording_closed('));archive=next(i for i,v in enumerate(lines) if v=='ready_storage.materialize()');self.assertLess(closed,archive)
        main=next(n for n in root.body if isinstance(n,ast.FunctionDef) and n.name=='main');self.assertLess(ast.unparse(main).index('ready_storage.start()'),ast.unparse(main).index('run(args,'))
if __name__=='__main__':unittest.main(verbosity=2)
