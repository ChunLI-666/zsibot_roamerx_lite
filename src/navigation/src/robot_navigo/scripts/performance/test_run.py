#!/usr/bin/env python3
"""Behavioral tests for opt-in launch, ancestry accounting, and owned cleanup."""
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

import run as runner_module


RUNNER = Path(__file__).with_name("run.py")


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def command(self, code, output="capture"):
        return [sys.executable, str(RUNNER), "--output", str(self.root / output),
                "--backend", "resources", "--interval", ".02", "--grace", ".2",
                "--", sys.executable, "-c", code]

    def environment(self, enabled="1"):
        return dict(os.environ, LOCNAV_PROFILE=enabled, LOCNAV_PROFILE_FAST_CPU="0")

    def summary(self):
        return json.loads((self.root / "capture/summary.json").read_text())

    def test_disabled_exec_preserves_status_and_creates_no_output(self):
        env = dict(self.environment("0"), LOCNAV_PROFILE_FAST_CPU="1")
        completed = subprocess.run(self.command("raise SystemExit(7)"), env=env, capture_output=True)
        self.assertEqual(completed.returncode, 7)
        self.assertFalse((self.root / "capture").exists())

    def test_cpu_window_statistics_use_equivalent_cores_not_cpu_count(self):
        stats = runner_module.CpuWindowStats()
        cpu = 0.0
        for index in range(1, 21):
            cpu += 0.2 if 6 <= index <= 15 else 0.0
            stats.add(index / 10, cpu)
        result = stats.summary()
        self.assertAlmostEqual(result['average_cores'], 1.0)
        self.assertAlmostEqual(result['max_100ms_sampled_cores'], 2.0)
        self.assertAlmostEqual(result['max_rolling_1s_sampled_cores'], 2.0)
        self.assertLessEqual(len(stats.points), 12)

    def test_fast_path_reads_only_known_identities_and_retains_dead_cpu(self):
        sampler = runner_module.FastCpuSampler(self.root, 0.0, 100)
        known = dict(pid=987654321, starttime=100, name='owned', user_ticks=0, system_ticks=0)
        sampler.update([known])
        written = []
        writer = mock.Mock(writerow=lambda row: written.append(row))
        rows = [dict(known, state='R', user_ticks=10, system_ticks=2), None,
                dict(known, starttime=101, state='R', user_ticks=9000)]
        with mock.patch.object(runner_module, 'cpu_stat', side_effect=rows) as stat, \
                mock.patch.object(runner_module.time, 'monotonic', side_effect=[.1, .2, .3]), \
                mock.patch.object(Path, 'iterdir', side_effect=AssertionError('fast sample must not inventory /proc')):
            for _ in range(3):
                sampler._sample(writer, mock.Mock(), False)
        self.assertEqual(stat.call_args_list, [mock.call(987654321)] * 3)
        aggregates = [row for row in written if row['scope'] == 'tree']
        self.assertEqual([row['user_ticks'] + row['system_ticks'] for row in aggregates], [12, 12, 12])
        self.assertEqual([row['alive'] for row in aggregates], [1, 0, 0])

    def test_opt_in_fast_sampler_writes_cpu_and_thread_evidence(self):
        env = dict(self.environment(), LOCNAV_PROFILE_FAST_CPU='1')
        completed = subprocess.run(self.command('import time; time.sleep(1.1)'), env=env, capture_output=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        summary = self.summary()['fast_cpu']
        self.assertEqual(summary['status'], 'complete')
        self.assertGreaterEqual(summary['aggregate']['samples'], 10)
        self.assertIsNotNone(summary['aggregate']['max_rolling_1s_sampled_cores'])
        with (self.root / 'capture/fast_cpu.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertTrue(any(row['scope'] == 'process' for row in rows))
        self.assertTrue(any(row['scope'] == 'tree' for row in rows))
        self.assertTrue((self.root / 'capture/thread_cpu.csv').is_file())
        self.assertTrue((self.root / 'capture/fast_cpu_summary.json').is_file())
        self.assertIn('tick_seconds', summary)

    def test_hardware_missing_evidence_is_null_and_counters_are_preserved(self):
        sys_root, proc_root = self.root / 'sys', self.root / 'proc'
        snapshot = runner_module.hardware_snapshot(sys_root, proc_root)
        self.assertIsNone(snapshot['thermal_throttle'])
        self.assertIsNone(snapshot['cooling_devices'])
        self.assertIsNone(snapshot['devfreq'])
        self.assertEqual(snapshot['pressure'], dict(cpu=None, memory=None, io=None))
        self.assertIsNone(snapshot['loadavg'])
        fixtures = {
            sys_root / 'devices/system/cpu/cpu2/thermal_throttle/core_throttle_count': '7\n',
            sys_root / 'class/thermal/cooling_device0/type': 'cpufreq\n',
            sys_root / 'class/thermal/cooling_device0/cur_state': '2\n',
            sys_root / 'class/devfreq/memory/cur_freq': '800000000\n',
            proc_root / 'pressure/cpu': 'some avg10=1.00 avg60=0.50 avg300=0.10 total=900\n',
            proc_root / 'loadavg': '1.00 0.50 0.25 1/100 123\n',
        }
        for path, value in fixtures.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(value)
        snapshot = runner_module.hardware_snapshot(sys_root, proc_root)
        self.assertEqual(snapshot['thermal_throttle']['cpu2']['core_throttle_count'], '7')
        self.assertIsNone(snapshot['thermal_throttle']['cpu2']['package_throttle_count'])
        self.assertEqual(snapshot['cooling_devices']['cooling_device0']['cur_state'], '2')
        self.assertEqual(snapshot['devfreq']['memory']['cur_freq'], '800000000')
        self.assertIn('total=900', snapshot['pressure']['cpu'])
        self.assertEqual(snapshot['loadavg'], '1.00 0.50 0.25 1/100 123')

    def test_nonzero_status_has_evidence(self):
        completed = subprocess.run(self.command("print('specific failure'); raise SystemExit(9)"), env=self.environment(), capture_output=True)
        self.assertEqual(completed.returncode, 9)
        self.assertEqual(self.summary()["command_returncode"], 9)
        self.assertIn("specific failure", (self.root / "capture/command.log").read_text())
        self.assertFalse((self.root / 'capture/fast_cpu.csv').exists())
        self.assertFalse((self.root / 'capture/thread_cpu.csv').exists())

    def test_missing_profiler_output_is_not_reported_as_success(self):
        # Even a normal native exit and matching mapped identities cannot turn
        # a missing or empty flushed profile into a successful capture.
        executable = dict(path=str(self.root / "native"), sha256="native fixture")
        library = dict(path=str(self.root / "libprofiler.so"), sha256="library fixture")
        args = mock.Mock(native_target=dict(executable=executable, library=library),
                         libprofiler=Path(library["path"]))
        runtime = dict(profiler_mapped=True)
        for profiles in ([], [dict(path="empty.prof", bytes=0)]):
            with self.subTest(profiles=profiles), \
                    mock.patch.object(runner_module, "file_identity", side_effect=[executable, library]), \
                    self.assertRaisesRegex(RuntimeError, "No nonempty native profile"):
                runner_module.native_cpu_result(args, runtime, profiles, 0, [])

    def test_tree_includes_separate_child_session(self):
        code = "import subprocess,sys; c=subprocess.Popen([sys.executable,'-c','import time; time.sleep(.3)'],start_new_session=True); c.wait()"
        completed = subprocess.run(self.command(code), env=self.environment(), capture_output=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        summary = self.summary()
        self.assertGreaterEqual(len(summary["observed_processes"]), 2)
        self.assertGreater(summary["sampled_tree_peak_rss_kib"], 0)
        self.assertEqual(summary["remaining_owned_pids"], [])

    def test_exited_child_cpu_remains_in_cumulative_totals(self):
        busy = "import time; end=time.monotonic()+.15\nwhile time.monotonic()<end: pass"
        code = "import subprocess,sys,time; subprocess.run([sys.executable,'-c'," + repr(busy) + "]); time.sleep(.2)"
        completed = subprocess.run(self.command(code), env=self.environment(), capture_output=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        with (self.root / "capture/resources.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        ticks = [int(row["observed_user_ticks"]) + int(row["observed_system_ticks"]) for row in rows]
        self.assertEqual(ticks, sorted(ticks))
        self.assertGreater(ticks[-1], 0)
        self.assertGreaterEqual(self.summary()["observed_user_ticks"] + self.summary()["observed_system_ticks"], ticks[-1])
        self.assertGreaterEqual(max(int(row["live_processes"]) for row in rows), 2)

    def test_root_only_signal_does_not_double_interrupt_relay_child(self):
        ready = self.root / 'child_ready'
        count = self.root / 'child_signals'
        child_code = ("import signal,time; from pathlib import Path; "
                      "count=Path(" + repr(str(count)) + "); "
                      "signal.signal(signal.SIGINT,lambda s,f:(count.write_text('INT'),exit(0))); "
                      "Path(" + repr(str(ready)) + ").touch(); time.sleep(10)")
        leader_code = ("import subprocess,sys,signal,time; "
                       "child=subprocess.Popen([sys.executable,'-c'," + repr(child_code) + "]); "
                       "signal.signal(signal.SIGINT,lambda s,f:(child.send_signal(s),child.wait(),exit(0))); "
                       "time.sleep(10)")
        command = self.command(leader_code)
        command.insert(command.index("--"), "--signal-root-only")
        runner = subprocess.Popen(command, env=self.environment(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            deadline = time.monotonic() + 3
            while not ready.exists() and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(ready.exists())
            runner.send_signal(signal.SIGINT)
            stdout, stderr = runner.communicate(timeout=4)
            self.assertEqual(runner.returncode, 130, (stdout, stderr))
            self.assertEqual(count.read_text(), "INT")
            self.assertEqual(self.summary()["command_returncode"], 0)
            self.assertEqual([row["signal"] for row in self.summary()["cleanup_signals"]], ["SIGINT"])
            self.assertEqual(self.summary()["cleanup_signals"][0]["target"], "leader")
        finally:
            if runner.poll() is None:
                runner.terminate()
                runner.communicate(timeout=4)

    def test_gperftools_rejects_interpreter_before_elf_inspection_or_child_start(self):
        # CPU profiling belongs at the final native ELF, never a Python launcher.
        command = self.command("raise AssertionError('must not start')")[2:]
        command[command.index("resources")] = "gperftools"
        env = dict(LOCNAV_PROFILE="1", LOCNAV_PROFILE_FAST_CPU="0")
        with mock.patch.dict(os.environ, env, clear=True), \
                mock.patch.object(runner_module.platform, "platform", return_value="fixture metadata"), \
                mock.patch.object(runner_module, "elf_info", side_effect=AssertionError("ELF query")) as elf, \
                mock.patch.object(runner_module.subprocess, "Popen", side_effect=AssertionError("child started")) as child, \
                mock.patch("sys.stderr"):
            self.assertEqual(runner_module.main(command), 2)
        elf.assert_not_called()
        child.assert_not_called()
        summary = self.summary()
        self.assertEqual(summary["status"], "backend_unavailable")
        self.assertIs(summary["command_started"], False)
        self.assertIn("direct absolute native ELF", summary["error"])

    @unittest.skipUnless(os.environ.get("LOCNAV_PROFILE_LIB"), "optional private gperftools library not supplied")
    def test_gperftools_real_library_does_not_allow_interpreter_launch(self):
        marker = self.root / "unexpected_child"
        command = self.command(f"from pathlib import Path; Path({str(marker)!r}).touch()")
        command[command.index("resources")] = "gperftools"
        env = self.environment()
        for name in ("LD_PRELOAD", "LD_AUDIT", "CPUPROFILE", "CPUPROFILE_REALTIME",
                     "CPUPROFILESIGNAL", "HEAPPROFILE", "HEAPCHECK"):
            env.pop(name, None)
        self.assertTrue(Path(env["LOCNAV_PROFILE_LIB"]).is_file())
        completed = subprocess.run(command, env=env, capture_output=True, timeout=10)
        self.assertEqual(completed.returncode, 2, completed.stderr)
        self.assertIn(b"direct absolute native ELF", completed.stderr)
        self.assertIs(self.summary()["command_started"], False)
        self.assertFalse(marker.exists())
        self.assertFalse((self.root / "capture/command.log").exists())

    def test_signal_cleans_only_owned_descendants(self):
        unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(10)"])
        runner = None
        try:
            code = "import subprocess,sys,time; subprocess.Popen([sys.executable,'-c','import time; time.sleep(10)'],start_new_session=True); time.sleep(10)"
            runner = subprocess.Popen(self.command(code), env=self.environment(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                path = self.root / "capture/processes.csv"
                if path.exists() and path.read_text().count("\n") >= 5:
                    break
                time.sleep(.02)
            runner.send_signal(signal.SIGTERM)
            stdout, stderr = runner.communicate(timeout=4)
            self.assertEqual(runner.returncode, 143, (stdout, stderr))
            self.assertEqual(self.summary()["remaining_owned_pids"], [])
            self.assertIsNone(unrelated.poll())
        finally:
            if runner and runner.poll() is None:
                runner.kill()
                runner.wait()
            unrelated.terminate()
            unrelated.wait()


if __name__ == "__main__":
    unittest.main()
