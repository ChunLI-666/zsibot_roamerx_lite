"""Persistence phase diagnostics retain actual SQLite durability and outcomes."""
from pathlib import Path
import tempfile
import unittest
import uuid
from unittest.mock import patch
from test_mission_state import fixture
import mission_state


class PersistenceTimingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.store=mission_state.MissionStore(self.root/'mission.db')
        raw,identity,policy,_=fixture();self.store.create(raw,identity,policy)
    def tearDown(self):self.store.close();self.temp.cleanup()
    def test_commit_and_two_exports_are_separately_visible_without_changing_durability(self):
        samples=[];self.store.timing_sink=samples.append;clock=[10**9];db=self.store.db
        class Proxy:
            def execute(inner,sql,*args):
                value=db.execute(sql,*args)
                if sql=='COMMIT':clock[0]+=40_000_000
                return value
            def __getattr__(inner,name):return getattr(db,name)
        self.store.db=Proxy();real=mission_state.atomic_json
        def export(path,value):
            real(path,value);clock[0]+=20_000_000 if path.name=='state.json' else 30_000_000
        with patch('mission_state.time.monotonic_ns',side_effect=lambda:clock[0]),patch('mission_state.atomic_json',side_effect=export):
            attempt=self.store.reserve(str(uuid.uuid4()),self.root);state=self.store.export(self.root)
        phases={r['stage']:r for r in samples}
        self.assertEqual(phases['SEND_INTENT:commit']['wall_ns'],40_000_000)
        self.assertEqual(phases['export_state:atomic_file_and_directory_fsync']['wall_ns'],20_000_000)
        self.assertEqual(phases['export_result:atomic_file_and_directory_fsync']['wall_ns'],30_000_000)
        self.assertTrue(all(r['success'] for r in samples))
        self.assertEqual(self.store.db.execute('PRAGMA synchronous').fetchone()[0],2)
        self.assertEqual(self.store.db.execute('PRAGMA journal_mode').fetchone()[0],'delete')
        self.assertEqual(state['attempt']['parent_action_uuid'],attempt['parent_action_uuid'])
        self.assertEqual(state['budgets']['mission_dispatches_reserved'],1)
    def test_diagnostic_sink_failure_does_not_rollback_a_committed_send_intent(self):
        def broken(sample):raise OSError('fixture diagnostic queue failure')
        self.store.timing_sink=broken
        attempt=self.store.reserve(str(uuid.uuid4()),self.root)
        self.assertEqual(self.store.state()['attempt']['parent_action_uuid'],attempt['parent_action_uuid'])
        self.assertGreater(self.store.timing_drops,0)
        with self.assertRaisesRegex(mission_state.Hold,'UNKNOWN_DISPATCH'):
            self.store.reserve(str(uuid.uuid4()),self.root)
    def test_failed_transaction_keeps_state_and_marks_only_completed_stages(self):
        samples=[];self.store.timing_sink=samples.append;before=self.store.state()
        with self.assertRaises(RuntimeError):
            with self.store.transaction('INJECTED') as state:
                state['progress']['next_index']=5
                raise RuntimeError('fixture failure before commit')
        self.assertEqual(self.store.state(),before)
        self.assertFalse(any(s['stage'].endswith(':commit') for s in samples))
    def test_hot_state_reads_aggregate_without_per_read_events_or_revision_changes(self):
        samples=[];self.store.timing_sink=samples.append;before=self.store.state()
        for _ in range(500):self.assertEqual(self.store.state(),before)
        self.assertEqual(samples,[])
        self.store.flush_state_read_timing('explicit_checkpoint')
        self.assertEqual(len(samples),1);sample=samples[0]
        self.assertEqual(sample['checkpoint'],'explicit_checkpoint')
        for phase in ['select','json_decode']:
            stats=sample['phases'][phase];self.assertEqual(stats['count'],501)
            self.assertGreater(stats['total_wall_ns'],0)
            self.assertGreaterEqual(stats['total_wall_ns'],stats['max_wall_ns'])
        self.store.flush_state_read_timing('no_new_reads');self.assertEqual(len(samples),1)
        self.assertEqual(self.store.state(),before)
    def test_real_checkpoint_flushes_reads_and_sink_failure_keeps_state(self):
        samples=[];self.store.timing_sink=samples.append
        for _ in range(20):self.store.state()
        self.store.export(self.root)
        aggregate=[s for s in samples if s['stage']=='state_reads:aggregate']
        self.assertEqual(len(aggregate),1)
        self.assertEqual(aggregate[0]['phases']['json_decode']['count'],21)
        before=self.store.state()
        def broken(sample):raise OSError('fixture sink failure')
        self.store.timing_sink=broken;self.store.flush_state_read_timing('failed_sink')
        self.assertEqual(self.store.timing_drops,1);self.assertEqual(self.store.state(),before)

if __name__=='__main__':unittest.main()
