#!/usr/bin/env python3
"""Opt-in CPU profiling and sampled process-tree resource evidence.

LOCNAV_PROFILE=0 (the default) execs the command without creating artifacts.
RSS/PSS are resident-memory measurements, not allocation/heap stack profiles.
"""
import argparse
from collections import deque
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import resource
import shutil
import signal
import struct
import subprocess
import sys
import threading
import tempfile
import time


def read_text(path):
    try:
        return Path(path).read_text(errors="replace").strip()
    except (OSError, PermissionError):
        return None


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def proc_stat(pid):
    raw = read_text(Path("/proc") / str(pid) / "stat")
    if not raw:
        return None
    try:
        name, tail = raw.split("(", 1)[1].rsplit(")", 1)
        fields = tail.split()
        try:
            executable = os.readlink(Path("/proc") / str(pid) / "exe")
        except OSError:
            executable = None
        return dict(pid=int(pid), name=name, executable=executable,
                    cmdline=(read_text(Path("/proc") / str(pid) / "cmdline") or "").replace("\x00", " "),
                    state=fields[0], ppid=int(fields[1]),
                    pgrp=int(fields[2]), starttime=int(fields[19]),
                    user_ticks=int(fields[11]), system_ticks=int(fields[12]))
    except (ValueError, IndexError):
        return None


def memory_kib(pid):
    result = dict(rss_kib=None, pss_kib=None, hwm_kib=None)
    status = read_text(Path("/proc") / str(pid) / "status") or ""
    rollup = read_text(Path("/proc") / str(pid) / "smaps_rollup") or ""
    for line in (status + "\n" + rollup).splitlines():
        fields = line.split()
        if len(fields) < 2:
            continue
        key = {"VmRSS:": "rss_kib", "VmHWM:": "hwm_kib", "Pss:": "pss_kib"}.get(fields[0])
        if key:
            try:
                result[key] = int(fields[1])
            except ValueError:
                pass
    return result


def identity(row):
    return row["pid"], row["starttime"]


def cpu_stat(pid, tid=None):
    """Fast path: one stat file, no executable/cmdline, PSS, or /proc inventory."""
    path = Path("/proc") / str(pid)
    if tid is not None:
        path = path / "task" / str(tid)
    raw = read_text(path / "stat")
    if not raw:
        return None
    try:
        name, tail = raw.split("(", 1)[1].rsplit(")", 1)
        fields = tail.split()
        return dict(pid=pid, name=name, state=fields[0], starttime=int(fields[19]),
                    user_ticks=int(fields[11]), system_ticks=int(fields[12]))
    except (ValueError, IndexError):
        return None


class CpuWindowStats:
    """Cumulative CPU seconds over the common capture wall-time denominator."""
    def __init__(self):
        self.points = deque([(0.0, 0.0)])
        self.maximum_sampled_cores = None
        self.maximum_rolling_1s_cores = None
        self.maximum_gap_seconds = 0.0
        self.samples = 0

    def add(self, elapsed, cpu_seconds):
        previous_time, previous_cpu = self.points[-1]
        if elapsed <= previous_time:
            return
        cpu_seconds = max(cpu_seconds, previous_cpu)
        gap = elapsed - previous_time
        self.maximum_gap_seconds = max(self.maximum_gap_seconds, gap)
        self.points.append((elapsed, cpu_seconds))
        self.samples += 1
        if elapsed >= 0.1:
            self.maximum_sampled_cores = max(self.maximum_sampled_cores or 0.0,
                                             self._window_cpu(elapsed - 0.1) / 0.1)
        if elapsed >= 1.0:
            start = elapsed - 1.0
            while len(self.points) > 2 and self.points[1][0] <= start:
                self.points.popleft()
            value = self._window_cpu(start)
            self.maximum_rolling_1s_cores = max(self.maximum_rolling_1s_cores or 0.0, value)

    def _window_cpu(self, start):
        points = list(self.points)
        for a, b in zip(points, points[1:]):
            if a[0] <= start <= b[0]:
                fraction = (start - a[0]) / (b[0] - a[0])
                return points[-1][1] - (a[1] + fraction * (b[1] - a[1]))
        raise ValueError("CPU window boundary is outside sample history")

    def summary(self):
        elapsed, cpu_seconds = self.points[-1]
        return dict(cpu_seconds=cpu_seconds, capture_elapsed_seconds=elapsed,
                    average_cores=cpu_seconds / elapsed if elapsed > 0 else None,
                    max_100ms_sampled_cores=self.maximum_sampled_cores,
                    max_rolling_1s_sampled_cores=self.maximum_rolling_1s_cores,
                    largest_sample_gap_seconds=self.maximum_gap_seconds, samples=self.samples)


class FastCpuSampler:
    """Sample only PID identities discovered by the existing slow process tree."""
    def __init__(self, output, started, clock_ticks):
        self.output, self.started, self.clock_ticks = Path(output), started, clock_ticks
        self.lock = threading.Lock()
        self.snapshot = {}
        self.latest = {}
        self.stats = {}
        self.aggregate = CpuWindowStats()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="locnav-cpu-monitor", daemon=True)
        self.error = None

    def update(self, rows):
        # The caller is the sole ProcessTree writer. Copy only lightweight stat
        # fields under this sampler lock; the fast path never scans /proc itself.
        with self.lock:
            for row in rows:
                self.snapshot[identity(row)] = {name: row[name] for name in
                    ("pid", "starttime", "name", "user_ticks", "system_ticks")}

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join()

    def _sample(self, writer, thread_writer, take_threads):
        elapsed = time.monotonic() - self.started
        with self.lock:
            known = list(self.snapshot.items())
        live = 0
        for key, baseline in known:
            current = cpu_stat(baseline["pid"])
            alive = current is not None and identity(current) == key
            if not alive:
                current = baseline
            previous = self.latest.get(key, baseline)
            row = dict(baseline, user_ticks=max(current["user_ticks"], baseline["user_ticks"], previous["user_ticks"]),
                       system_ticks=max(current["system_ticks"], baseline["system_ticks"], previous["system_ticks"]))
            self.latest[key] = row
            cpu = (row["user_ticks"] + row["system_ticks"]) / self.clock_ticks
            stats = self.stats.setdefault(key, CpuWindowStats())
            stats.add(elapsed, cpu)
            writer.writerow(dict(elapsed_seconds=elapsed, scope="process", alive=int(alive), cpu_seconds=cpu, **row))
            live += alive
            if take_threads and alive:
                self._threads(baseline, elapsed, thread_writer)
        user = sum(row["user_ticks"] for row in self.latest.values())
        system = sum(row["system_ticks"] for row in self.latest.values())
        cpu = (user + system) / self.clock_ticks
        self.aggregate.add(elapsed, cpu)
        writer.writerow(dict(elapsed_seconds=elapsed, scope="tree", pid="", starttime="", name="observed_tree",
                             user_ticks=user, system_ticks=system, cpu_seconds=cpu, alive=live))

    @staticmethod
    def _threads(process, elapsed, writer):
        current = cpu_stat(process["pid"])
        if not current or identity(current) != identity(process):
            return
        directory = Path("/proc") / str(process["pid"]) / "task"
        try:
            tids = [int(entry.name) for entry in directory.iterdir() if entry.name.isdigit()]
        except OSError:
            return
        for tid in tids:
            row = cpu_stat(process["pid"], tid)
            if row:
                writer.writerow(dict(elapsed_seconds=elapsed, pid=process["pid"], process_starttime=process["starttime"],
                    tid=tid, thread_starttime=row["starttime"], name=row["name"], state=row["state"],
                    user_ticks=row["user_ticks"], system_ticks=row["system_ticks"]))

    def _run(self):
        try:
            with (self.output / "fast_cpu.csv").open("w", newline="") as process_file, \
                    (self.output / "thread_cpu.csv").open("w", newline="") as thread_file:
                writer = csv.DictWriter(process_file, fieldnames=["elapsed_seconds", "scope", "pid", "starttime", "name", "user_ticks", "system_ticks", "cpu_seconds", "alive"])
                writer.writeheader()
                thread_writer = csv.DictWriter(thread_file, fieldnames=["elapsed_seconds", "pid", "process_starttime", "tid", "thread_starttime", "name", "state", "user_ticks", "system_ticks"])
                thread_writer.writeheader()
                deadline = time.monotonic()
                next_threads = deadline
                while True:
                    now = time.monotonic()
                    self._sample(writer, thread_writer, now >= next_threads)
                    if now >= next_threads:
                        next_threads = now + 1.0
                    process_file.flush()
                    thread_file.flush()
                    if self.stop_event.is_set():
                        break
                    deadline += 0.1
                    if self.stop_event.wait(max(0.0, deadline - time.monotonic())):
                        self._sample(writer, thread_writer, False)
                        break
        except BaseException as error:
            self.error = repr(error)

    def summary(self):
        return dict(status="failed" if self.error else "complete", error=self.error,
            clock_ticks=self.clock_ticks, tick_seconds=1.0 / self.clock_ticks,
            requested_process_interval_seconds=0.1, requested_thread_interval_seconds=1.0,
            aggregate=self.aggregate.summary(),
            processes=[dict(pid=key[0], starttime=key[1], name=self.latest[key]["name"], **value.summary())
                       for key, value in self.stats.items()],
            definitions={"cores": "CPU seconds / wall seconds; never divided by logical CPU count",
                         "max_100ms_sampled_cores": "Maximum CPU delta over 100 wall milliseconds ending at sampled timestamps; interpolated boundary, inspect sample gaps",
                         "max_rolling_1s_sampled_cores": "Maximum CPU delta over one wall second ending at sampled timestamps; linearly interpolated cumulative CPU at the start boundary",
                         "average_cores": "Observed cumulative CPU since launched-process birth / common full capture wall duration",
                         "thread_cpu": "Per-TID attribution only; do not add thread CPU to the same process CPU"},
            limitations=["Sampled peaks are not a hard real-time instantaneous CPU bound.",
                         "Tick granularity and interpolation bound precision; at HZ=100 one tick is 10 ms CPU (0.1 core in a 100 ms interval).",
                         "New PIDs are discovered only by the slow tree sampler; delayed discovery can shift attribution between fast windows, and short-lived undiscovered children are missed.",
                         "Counters of dead processes remain at their last observed value; final exit CPU may be missed if /proc disappeared before observation.",
                         "Capture extends from launch through cleanup; use CSV timestamps to trim to a separately specified active replay window before applying a playback-only budget gate.",
                         "Thread snapshots are once per second, so short-lived TIDs may be missed; comm alone does not identify a runtime."])


def matching_process(row):
    current = proc_stat(row["pid"])
    return current is not None and identity(current) == identity(row)


class ProcessTree:
    """Track descendants by ancestry and PID birth time, including child sessions."""
    def __init__(self, root_pid):
        self.root_pid = root_pid
        self.known = {}
        row = proc_stat(root_pid)
        if row:
            self.known[identity(row)] = row

    def sample(self):
        table = {}
        for entry in Path("/proc").iterdir():
            if entry.name.isdigit():
                row = proc_stat(int(entry.name))
                if row:
                    table[row["pid"]] = row
        active = {row["pid"] for key, row in self.known.items()
                  if row["pid"] in table and identity(table[row["pid"]]) == key}
        if self.root_pid in table and not self.known:
            active.add(self.root_pid)
        while True:
            discovered = {pid for pid, row in table.items() if row["ppid"] in active}
            new = discovered - active
            if not new:
                break
            active.update(new)
        rows = []
        for pid in sorted(active):
            row = table[pid]
            row.update(memory_kib(pid))
            prior = self.known.get(identity(row), {})
            for metric in ("rss_kib", "pss_kib", "hwm_kib"):
                row["peak_" + metric] = max(prior.get("peak_" + metric) or 0, row[metric] or 0)
            if not row["executable"]:
                row["executable"] = prior.get("executable")
            if not row["cmdline"]:
                row["cmdline"] = prior.get("cmdline", "")
            self.known[identity(row)] = row
            rows.append(row)
        return rows

    def signal_owned(self, number):
        self.sample()
        # Individual descendants cover child sessions without touching unrelated
        # groups. Recheck starttime immediately before signalling to avoid reuse.
        sent = []
        for row in sorted(self.known.values(), key=lambda item: item["pid"] != self.root_pid):
            if matching_process(row):
                try:
                    os.kill(row["pid"], number)
                    sent.append(row["pid"])
                except ProcessLookupError:
                    pass
        return sent

    def signal_leader(self, number, root_only):
        row = next((row for row in self.known.values() if row["pid"] == self.root_pid), None)
        if not row or not matching_process(row):
            return []
        try:
            if root_only:
                os.kill(self.root_pid, number)
            else:
                # Popen(start_new_session=True) owns this newly created group.
                # One broadcast avoids separately interrupting the same child.
                os.killpg(self.root_pid, number)
            return [self.root_pid]
        except ProcessLookupError:
            return []

    def live(self):
        self.sample()
        return [row for row in self.known.values()
                if matching_process(row) and (proc_stat(row["pid"]) or {}).get("state") != "Z"]


def cgroup_snapshot(pid):
    membership = read_text(Path("/proc") / str(pid) / "cgroup")
    result = {"membership": membership, "scope": "shared inherited cgroup; not command-exclusive"}
    for line in (membership or "").splitlines():
        parts = line.split(":", 2)
        if len(parts) == 3 and parts[0] == "0":
            root = Path("/sys/fs/cgroup").resolve()
            directory = (root / parts[2].lstrip("/")).resolve()
            if directory == root or root in directory.parents:
                result["path"] = str(directory)
                for name in ("cpu.stat", "cpu.max", "memory.current", "memory.peak",
                             "memory.max", "memory.events", "memory.stat"):
                    result[name] = read_text(directory / name)
    return result


def hardware_snapshot(sys_root=Path("/sys"), proc_root=Path("/proc")):
    result = {"frequency": {}, "thermal": {}, "thermal_throttle": None,
              "cooling_devices": None, "devfreq": None,
              "pressure": {name: read_text(proc_root / "pressure" / name)
                           for name in ("cpu", "memory", "io")},
              "loadavg": read_text(proc_root / "loadavg")}
    for policy in (sys_root / "devices/system/cpu/cpufreq").glob("policy*"):
        result["frequency"][policy.name] = {
            name: read_text(policy / name) for name in
            ("scaling_cur_freq", "scaling_min_freq", "scaling_max_freq", "scaling_governor")}
    for zone in (sys_root / "class/thermal").glob("thermal_zone*"):
        result["thermal"][zone.name] = {"type": read_text(zone / "type"), "temp_millic": read_text(zone / "temp")}
    throttle = {}
    for directory in (sys_root / "devices/system/cpu").glob("cpu[0-9]*/thermal_throttle"):
        throttle[directory.parent.name] = {
            name: read_text(directory / name) for name in
            ("core_throttle_count", "package_throttle_count")}
    result["thermal_throttle"] = throttle or None
    cooling = {}
    for directory in (sys_root / "class/thermal").glob("cooling_device*"):
        cooling[directory.name] = {name: read_text(directory / name)
                                   for name in ("type", "cur_state", "max_state")}
    result["cooling_devices"] = cooling or None
    devfreq = {}
    for directory in (sys_root / "class/devfreq").glob("*"):
        devfreq[directory.name] = {name: read_text(directory / name)
                                   for name in ("name", "cur_freq", "min_freq", "max_freq", "governor")}
    result["devfreq"] = devfreq or None
    return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="New directory; required only when enabled")
    parser.add_argument("--native-output-root", type=Path, help="gperftools native prefix only: atomically create one unique output per prefix PID")
    parser.add_argument("--native-allow-executable", action="append", default=[], help="Exact resolved native package executable allowed by this prefix")
    parser.add_argument("--backend", choices=("perf", "gperftools", "heap", "heap_sampled", "resources"), default="perf")
    parser.add_argument("--interval", type=float, default=0.5, help="Resource sampling wall seconds")
    parser.add_argument("--frequency", type=int, default=int(os.environ.get("LOCNAV_PROFILE_FREQUENCY", "99")), help="CPU profiler samples/second")
    parser.add_argument("--grace", type=float, default=5.0, help="Seconds per shutdown stage")
    parser.add_argument("--signal-root-only", action="store_true", help="Send initial cleanup signal only to a launch leader that relays it")
    parser.add_argument("--libprofiler", type=Path, default=os.environ.get("LOCNAV_PROFILE_LIB"), help="Explicit private gperftools library path")
    parser.add_argument("--heap-lib", type=Path, default=os.environ.get("LOCNAV_PROFILE_HEAP_LIB"), help="External heap library: combined tcmalloc+profiler for heap, plain tcmalloc for heap_sampled")
    parser.add_argument("--heap-sampled-helper", type=Path, default=os.environ.get("LOCNAV_PROFILE_HEAP_SAMPLED_HELPER"), help="Externally built sampled-live-heap helper; direct native ELF only")
    parser.add_argument("--heap-sampled-manifest", type=Path, default=os.environ.get("LOCNAV_PROFILE_HEAP_SAMPLED_MANIFEST"), help="Matching external helper build manifest; heap_sampled requires plain --heap-lib")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    if not args.command:
        parser.error("a command after -- is required")
    setting = os.environ.get("LOCNAV_PROFILE", "0")
    if setting not in ("0", "1"):
        parser.error("LOCNAV_PROFILE must be 0 or 1")
    args.enabled = setting == "1"
    if args.enabled and args.native_output_root is not None and (args.backend != "gperftools" or args.output is not None):
        parser.error("--native-output-root requires gperftools and no --output")
    if args.enabled and args.native_output_root is not None and not args.native_allow_executable:
        parser.error("native prefix requires explicit expected package executable allowlist")
    if args.enabled and ((args.output is None and args.native_output_root is None) or args.interval <= 0 or args.frequency <= 0 or args.grace <= 0):
        parser.error("enabled profiling requires --output and positive interval/frequency/grace")
    return args


def profiler_command(args, output, env):
    if args.backend == "resources":
        return args.command
    if args.backend == "heap_sampled":
        return sampled_heap_command(args, output, env)
    if args.backend == "perf":
        executable = shutil.which("perf")
        if not executable:
            raise RuntimeError("perf executable not found")
        probe = subprocess.run([executable, "stat", "-e", "task-clock:u", "--", "true"],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (output / "perf_preflight.log").write_text(probe.stdout)
        if probe.returncode:
            raise RuntimeError("perf preflight failed; see perf_preflight.log; command was not started")
        return [executable, "record", "-e", "cpu-clock:u", "-F", str(args.frequency),
                "--call-graph", "dwarf", "-o", str(output / "perf.data"), "--", *args.command]
    if args.backend == "gperftools":
        args.native_target = gperftools_native_target(args, env)
    # gperftools appends child PIDs to profile names. This is verified against
    # the private tool version before experiments; do not silently run unprofiled.
    env.pop("CPUPROFILE", None)
    env.pop("CPUPROFILE_REALTIME", None)
    env.pop("CPUPROFILESIGNAL", None)
    env.pop("HEAPPROFILE", None)
    if args.backend == "heap":
        if not args.heap_lib or not args.heap_lib.is_file():
            raise RuntimeError("heap backend requires existing --heap-lib (tcmalloc+profiler)")
        library = args.heap_lib
        (output / "heap_profiles").mkdir()
        env["HEAPPROFILE"] = str(output / "heap_profiles" / "heap")
        env["HEAP_PROFILE_TIME_INTERVAL"] = "10"
    else:
        if not args.libprofiler or not args.libprofiler.is_file():
            raise RuntimeError("gperftools requires existing --libprofiler")
        library = args.libprofiler
        (output / "cpu_profiles").mkdir()
        env["CPUPROFILE"] = str(output / "cpu_profiles" / "cpu.prof")
        env["CPUPROFILE_FREQUENCY"] = str(args.frequency)
    env["LD_PRELOAD"] = ":".join([str(library.resolve()), env.get("LD_PRELOAD", "")]).rstrip(":")
    return args.command


def file_identity(path):
    if not path:
        return None
    path = Path(path).resolve()
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return dict(path=str(path), sha256=digest.hexdigest())
    except OSError:
        return dict(path=str(path), sha256=None)


def elf_info(path):
    """Read ELF identity/dependencies without running the file or requiring binutils."""
    path = Path(path).resolve()
    with path.open("rb") as stream:
        header = stream.read(64)
        if len(header) < 52 or header[:4] != b"\x7fELF" or header[4] not in (1, 2) or header[5] not in (1, 2):
            raise RuntimeError("Not a supported ELF file: " + str(path))
        bits, order = (64 if header[4] == 2 else 32), ("<" if header[5] == 1 else ">")
        kind, machine = struct.unpack_from(order + "HH", header, 16)
        flags = struct.unpack_from(order + "I", header, 48 if bits == 64 else 36)[0]
        phoff = struct.unpack_from(order + ("Q" if bits == 64 else "I"), header, 32 if bits == 64 else 28)[0]
        phsize, phnum = struct.unpack_from(order + "HH", header, 54 if bits == 64 else 42)
        nominal, size = (56 if bits == 64 else 32), path.stat().st_size
        if not nominal <= phsize <= 4096 or not 0 < phnum <= 8192 or phoff + phsize * phnum > size:
            raise RuntimeError("Unsupported/truncated ELF program headers: " + str(path))
        loads, dynamic, interpreter = [], None, None
        for index in range(phnum):
            stream.seek(phoff + index * phsize)
            raw = stream.read(nominal)
            if bits == 64:
                tag, _, offset, address, _, length, _, _ = struct.unpack(order + "IIQQQQQQ", raw)
            else:
                tag, offset, address, _, length, _, _, _ = struct.unpack(order + "IIIIIIII", raw)
            if offset + length > size:
                raise RuntimeError("ELF segment outside file: " + str(path))
            if tag == 1:
                loads.append((address, offset, length))
            elif tag == 2:
                dynamic = offset, length
            elif tag == 3:
                if not 0 < length <= 4096:
                    raise RuntimeError("Invalid ELF interpreter")
                position = stream.tell()
                stream.seek(offset)
                interpreter = stream.read(length).split(b"\0", 1)[0].decode("utf-8")
                stream.seek(position)
        needed, soname = [], None
        if dynamic:
            offset, length = dynamic
            stride = 16 if bits == 64 else 8
            if length > 4 * 1024 * 1024 or length % stride:
                raise RuntimeError("Unsupported ELF dynamic table")
            stream.seek(offset)
            values = []
            for entry in struct.iter_unpack(order + ("qQ" if bits == 64 else "iI"), stream.read(length)):
                if entry[0] == 0:
                    break
                values.append(entry)
            tags = dict(values)
            if 5 in tags:
                string_size = tags.get(10, 0)
                segments = [(fileoff + tags[5] - address, available - (tags[5] - address))
                            for address, fileoff, available in loads if address <= tags[5] < address + available]
                if len(segments) != 1 or not 0 < string_size <= min(16 * 1024 * 1024, segments[0][1]):
                    raise RuntimeError("Unsupported ELF dynamic string table")
                stream.seek(segments[0][0])
                strings = stream.read(string_size)
                def string_at(index):
                    end = strings.find(b"\0", index) if 0 <= index < len(strings) else -1
                    if end < 0:
                        raise RuntimeError("Invalid ELF dynamic string")
                    return strings[index:end].decode("utf-8")
                needed = [string_at(value) for tag, value in values if tag == 1]
                soname = string_at(tags[14]) if 14 in tags else None
    return dict(path=str(path), bits=bits, byte_order=header[5], machine=machine, type=kind,
                arm_float_abi=flags & 0x600 if machine == 40 else None,
                interpreter=interpreter, soname=soname, needed=needed)


def sampled_heap_setup(args):
    """Validate externally built artifacts before launching any instrumented command."""
    selected = {}
    for name in ("heap_lib", "heap_sampled_helper", "heap_sampled_manifest"):
        path = getattr(args, name, None)
        if not path or not Path(path).is_absolute() or not Path(path).is_file():
            raise RuntimeError("heap_sampled requires an existing absolute --" + name.replace("_", "-"))
        selected[name] = Path(path).resolve()
    command = Path(args.command[0])
    if not command.is_absolute() or not command.is_file() or not os.access(command, os.X_OK):
        raise RuntimeError("heap_sampled requires an absolute executable native ELF command")
    command = command.resolve()
    launchers = {"ros2", "launch", "taskset", "env", "bash", "sh", "dash", "zsh", "fish", "busybox", "perl", "ruby", "node", "java"}
    if command.name in launchers or command.name.startswith(("python", "pypy")):
        raise RuntimeError("heap_sampled rejects launchers/interpreters; invoke the actual native ELF directly")
    try:
        manifest = json.loads(selected["heap_sampled_manifest"].read_text())
        if not isinstance(manifest, dict) or manifest.get("success") is not True or not manifest.get("finished_unix"):
            raise RuntimeError("Helper build manifest is incomplete")
        if manifest.get("schema") not in (None, "locnav.heap_sampled.build.v1"):
            raise RuntimeError("Unknown helper build manifest schema")
        headers = manifest.get("headers_sha256", {})
        if (not isinstance(headers, dict) or not headers or
                not {"malloc_extension.h", "heap-profiler.h"}.issubset({Path(name).name for name in headers}) or
                not all(isinstance(digest, str) and re.fullmatch(r"[0-9a-f]{64}", digest) for digest in headers.values())):
            raise RuntimeError("Helper manifest lacks real malloc_extension.h provenance")
        library_sha = manifest.get("library_sha256")
        artifacts = manifest.get("artifacts_sha256", {})
        if not isinstance(artifacts, dict) or not isinstance(library_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", library_sha):
            raise RuntimeError("Helper manifest lacks valid library/artifact SHA provenance")
        helper_shas = {digest for name, digest in artifacts.items() if Path(name).name == selected["heap_sampled_helper"].name}
        if (len(helper_shas) != 1 or not isinstance(next(iter(helper_shas)), str) or
                not re.fullmatch(r"[0-9a-f]{64}", next(iter(helper_shas))) or
                next(iter(helper_shas)) != file_identity(selected["heap_sampled_helper"])["sha256"]):
            raise RuntimeError("Sampled helper SHA does not match its external build manifest")
        if library_sha != file_identity(selected["heap_lib"])["sha256"]:
            raise RuntimeError("Plain tcmalloc SHA does not match the helper build manifest")
        infos = {name: elf_info(path) for name, path in dict(
            executable=command, helper=selected["heap_sampled_helper"], library=selected["heap_lib"], host=Path("/proc/self/exe")).items()}
    except (ValueError, TypeError, KeyError, struct.error, UnicodeError) as error:
        raise RuntimeError("Invalid sampled helper manifest/ELF: " + str(error)) from error
    abi = lambda item: (item["bits"], item["byte_order"], item["machine"], item["arm_float_abi"])
    if any(abi(info) != abi(infos["host"]) for info in infos.values()):
        raise RuntimeError("Native executable/helper/tcmalloc/host ELF architecture or ABI mismatch")
    if infos["executable"]["type"] not in (2, 3) or not infos["executable"]["interpreter"]:
        raise RuntimeError("heap_sampled requires a dynamic native executable; static ELF cannot preload the helper")
    library_name = infos["library"]["soname"] or ""
    if (infos["helper"]["type"] != 3 or infos["library"]["type"] != 3 or
            not library_name.startswith("libtcmalloc.so.") or
            [name for name in infos["helper"]["needed"] if "tcmalloc" in name] != [library_name]):
        raise RuntimeError("Helper must depend on the exact plain libtcmalloc SONAME; combined/minimal/second allocators are rejected")
    return dict(executable=str(command), executable_identity=file_identity(command), elf=infos,
                helper=file_identity(selected["heap_sampled_helper"]), library=file_identity(selected["heap_lib"]),
                build_manifest=file_identity(selected["heap_sampled_manifest"]),
                sample_parameter=524288, interval_seconds=10,
                meaning="Sampled live-object allocation stacks with Poisson correction; not cumulative allocations")


def sampled_heap_command(args, output, env):
    if env.get("LD_PRELOAD"):
        raise RuntimeError("heap_sampled requires no inherited LD_PRELOAD; do not mix allocators/instrumentation")
    args.sampled_setup = sampled_heap_setup(args)
    for name in list(env):
        if name.startswith(("CPUPROFILE", "HEAP_PROFILE", "TCMALLOC_", "LOCNAV_HEAP_SAMPLE")) or name in ("HEAPPROFILE", "HEAPCHECK"):
            env.pop(name)
    directory = output / "heap_sampled_profiles"
    directory.mkdir()
    env.update(LD_PRELOAD=args.sampled_setup["library"]["path"] + ":" + args.sampled_setup["helper"]["path"],
               LOCNAV_HEAP_SAMPLE="1", LOCNAV_HEAP_SAMPLE_DIR=str(directory),
               LOCNAV_HEAP_SAMPLE_EXPECT_EXE=args.sampled_setup["executable"],
               LOCNAV_HEAP_SAMPLE_INTERVAL_SECONDS="10", TCMALLOC_SAMPLE_PARAMETER="524288")
    return [args.sampled_setup["executable"], *args.command[1:]]


def sampled_heap_result(output, setup, root_pid, observed):
    """Require actual matching PID/maps, nonempty live data and a joined worker."""
    directory = output / "heap_sampled_profiles"
    manifests = list(directory.glob("*/sampler_manifest.json"))
    summaries = list(directory.glob("*/sampler_summary.json"))
    metadata_paths = sorted(directory.glob("*/sample_*.json"))
    if any(file_identity(setup[name]["path"]) != setup[name] for name in ("library", "helper", "executable_identity")):
        raise RuntimeError("Sampled native executable/helper/allocator changed during the diagnostic")
    if len(manifests) != 1 or len(summaries) != 1 or not metadata_paths or list(directory.glob("*/sampler_error.json")):
        raise RuntimeError("No complete unique sampled-live-heap collection; inspect helper errors/exit behavior")
    try:
        manifest, summary = json.loads(manifests[0].read_text()), json.loads(summaries[0].read_text())
        roots = [row for row in observed if row["pid"] == root_pid]
        if (len(roots) != 1 or len(observed) != 1 or manifest["pid"] != root_pid or
                roots[0].get("executable") != setup["executable"] or
                manifest["starttime"] != roots[0]["starttime"] or manifest["executable"] != setup["executable"] or
                summary["pid"] != root_pid or summary["starttime"] != manifest["starttime"] or
                summary["errors"] != 0 or not summary["worker_stopped"] or summary["completed"] != len(metadata_paths)):
            raise RuntimeError("Sampled heap identity/one-process scope/worker cleanup mismatch; fork/exec children are unsupported")
        if (manifest["interval_seconds"] != 10 or manifest["sample_parameter"] != 524288 or
                manifest["full_heap_profiler_running"] or
                Path(manifest["allocator_library"]).resolve() != Path(setup["library"]["path"]) or
                "TCMallocImplementation" not in manifest["malloc_extension_dynamic_type"]):
            raise RuntimeError("Unexpected sampler configuration/allocator implementation")
        nonempty, profile_files = 0, []
        for path in metadata_paths:
            row = json.loads(path.read_text())
            heap, maps = path.with_suffix(".heap"), path.with_suffix(".maps")
            profile = heap.read_text()
            if (not row["valid"] or row["kind"] != "sampled_live_heap_estimate" or
                    row["pid"] != root_pid or row["starttime"] != manifest["starttime"] or
                    row["worker_tid"] != manifest["worker_tid"] or row["worker_tid"] == root_pid or
                    row["executable"] != setup["executable"] or row["sample_parameter"] != 524288 or
                    row["full_heap_profiler_running"] or row["cumulative_allocations_available"] or
                    row["profile_bytes"] != heap.stat().st_size or not maps.is_file()):
                raise RuntimeError("Incomplete/mismatched sampled heap metadata")
            header = profile.splitlines()[0]
            own_maps = profile.partition("MAPPED_LIBRARIES:\n")[2]
            if (header != row["raw_header"] or "@ heap_v2/524288" not in header or
                    not all(name in own_maps for name in (setup["executable"], setup["library"]["path"], setup["helper"]["path"]))):
                raise RuntimeError("Sampled heap lost its own real executable/library maps or v2 period")
            fields = header.split("@", 1)[0].replace("heap profile:", "").replace("[", "").replace("]", "").replace(":", "").split()
            if len(fields) != 4 or not all(value.isdigit() for value in fields):
                raise RuntimeError("Invalid sampled-live-heap header totals")
            totals = tuple(map(int, fields))
            buckets = re.findall(r"^\s*(\d+):\s*(\d+)\s*\[\s*(\d+):\s*(\d+)\]\s*@\s+0x", profile, re.M)
            if (totals[:2] != totals[2:] or
                    tuple(sum(int(row[index]) for row in buckets) for index in range(4)) != totals):
                raise RuntimeError("Incomplete sampled-live-heap buckets; do not treat truncated data as a complete sample")
            nonempty += int(fields[1]) > 0
            profile_files.append(dict(path=str(heap.relative_to(output)), bytes=heap.stat().st_size))
        if not nonempty:
            raise RuntimeError("All sampled live heaps are empty; do not fabricate a nonempty flame graph")
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        raise RuntimeError("Invalid sampled heap result: " + str(error)) from error
    return dict(valid=True, snapshots=len(metadata_paths), nonempty_snapshots=nonempty,
                worker_stopped=True, profile_files=profile_files, cumulative_allocations_available=False,
                limitation="Only this directly launched native process; sampled live estimates are not RSS/PSS or cumulative allocation counts")


def shutdown(tree, child, grace, first_signal, root_only=False):
    audit = []
    for stage, number in enumerate((first_signal, signal.SIGTERM, signal.SIGKILL)):
        if not tree.live():
            return audit
        pids = tree.signal_leader(number, root_only) if stage == 0 else tree.signal_owned(number)
        audit.append(dict(signal=signal.Signals(number).name,
                          target="leader" if stage == 0 and root_only else "launched_group" if stage == 0 else "remaining_descendants",
                          pids=pids))
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            child.poll()  # Reap the root so an exited zombie is never a live child.
            if not tree.live():
                return audit
            time.sleep(min(.05, grace))
    child.poll()
    return audit



def gperftools_native_target(args, env):
    """Validate without executing: never start a CPU timer in an exec launcher."""
    if any(env.get(k) for k in ("LD_PRELOAD", "LD_AUDIT", "CPUPROFILE", "CPUPROFILE_REALTIME", "CPUPROFILESIGNAL", "HEAPPROFILE", "HEAPCHECK")):
        raise RuntimeError("gperftools requires a clean intermediate environment")
    executable = Path(args.command[0])
    excluded = {"env", "taskset", "ros2", "launch", "bash", "sh", "dash", "zsh", "fish", "busybox", "perl", "ruby", "node", "java", "run_budget", "profile_workload"}
    if not executable.is_absolute() or executable.name in excluded or executable.name.startswith(("python", "pypy")):
        raise RuntimeError("gperftools accepts the direct absolute native ELF only, not a launcher/interpreter")
    executable = executable.resolve(strict=True)
    if executable.name in excluded or executable.name.startswith(("python", "pypy")):
        raise RuntimeError("Resolved native CPU target is a launcher/interpreter alias")
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError("Native profiling target is missing/not executable")
    allowed = [Path(value).resolve(strict=True) for value in args.native_allow_executable]
    if allowed and executable not in allowed:
        raise RuntimeError("Resolved native executable is outside the explicit package allowlist")
    if not args.libprofiler or not args.libprofiler.is_file():
        raise RuntimeError("gperftools requires the existing explicit plain --libprofiler")
    info = elf_info(executable); host = elf_info(Path('/proc/self/exe')); library = elf_info(args.libprofiler)
    abi = lambda v: (v['bits'],v['byte_order'],v['machine'],v['arm_float_abi'])
    if info['type'] not in (2,3) or not info['interpreter'] or abi(info)!=abi(host) or abi(library)!=abi(host):
        raise RuntimeError("Native profiling requires a dynamic executable and matching native profiler ABI")
    if library['type'] != 3 or not (library['soname'] or '').startswith('libprofiler.so.'):
        raise RuntimeError("CPU boundary requires plain libprofiler; do not replace the allocator")
    return dict(schema=1,requested_command=list(args.command),executable=file_identity(executable),elf=info,
                library=file_identity(args.libprofiler),prefix_pid=os.getpid(),injection_boundary='Single Popen of actual native ELF; all Python/ROS/taskset parents remain clean',
                later_exec='Unsupported: this target must not exec another image after profiler start')


def native_cpu_snapshot(args, child, output):
    target=args.native_target;row=proc_stat(child.pid)
    if not row:return None
    if row.get('executable') != target['executable']['path']:
        raise RuntimeError('Actual native PID executable differs from validated target')
    maps=read_text(Path('/proc')/str(child.pid)/'maps')
    if not maps:return None
    mapped={line.split(None,5)[5] for line in maps.splitlines() if len(line.split(None,5))==6 and line.split(None,5)[5].startswith('/')}
    if target['library']['path'] not in mapped:return None
    path=output/'native_maps.txt'
    with path.open('x') as f:f.write(maps+'\n')
    return dict(pid=row['pid'],starttime=row['starttime'],executable=row['executable'],command=row['cmdline'],maps=file_identity(path),profiler_mapped=True)


def native_cpu_result(args, runtime, profiles, returncode, received):
    if not runtime or runtime.get('profiler_mapped') is not True:
        raise RuntimeError('No actual native PID/starttime/executable/profiler maps proof')
    if file_identity(args.native_target['executable']['path']) != args.native_target['executable'] or file_identity(args.libprofiler) != args.native_target['library']:
        raise RuntimeError('Native executable/profiler identity changed during capture')
    if not any(p['bytes']>0 for p in profiles):raise RuntimeError('No nonempty native profile flushed')
    if returncode != 0 and not (received and received[0]==signal.SIGINT and returncode in (-signal.SIGINT,128+signal.SIGINT)):
        raise RuntimeError('Native exited abnormally; signal exits including SIGPROF remain failures: '+str(returncode))
    return dict(valid=True,target=args.native_target,actual=runtime,nonempty_profile_flushed=True,
                scope='This actual native target only; mapped identities are sampled, not lifetime coverage or a resource acceptance gate')

def verified_prefix_shutdown(args, received, returncode, failure, native_result, cleanup_signals, remaining):
    # A native prefix is a launch child. Report clean native shutdown as zero
    # only after every CPU-profile proof succeeded; keep SIGINT in its summary.
    return bool(args.backend == 'gperftools' and args.native_output_root is not None
                and received == [signal.SIGINT] and returncode == 0 and failure is None
                and native_result is not None and native_result.get('valid') is True
                and not remaining and len(cleanup_signals) == 1
                and cleanup_signals[0].get('signal') == 'SIGINT'
                and cleanup_signals[0].get('target') == 'launched_group'
                and cleanup_signals[0].get('pids'))


def main(argv=None):
    args = arguments(argv)
    if not args.enabled:
        os.execvpe(args.command[0], args.command, os.environ.copy())
    native_claimed = False
    if args.native_output_root is not None:
        # Validation precedes allocation or Popen; the parent environment is never mutated.
        args.native_target = gperftools_native_target(args, os.environ)
        root = args.native_output_root.resolve();root.mkdir(parents=True, exist_ok=True)
        args.output = Path(tempfile.mkdtemp(prefix='native-'+str(os.getpid())+'-',dir=root))
        native_claimed = True
    output = args.output.resolve()
    if not native_claimed:output.mkdir(parents=True, exist_ok=False)
    fast_cpu_enabled = os.environ.get("LOCNAV_PROFILE_FAST_CPU", "0") == "1"
    manifest = dict(schema="locnav.performance.v1", command=args.command, cwd=os.getcwd(),
                    backend=args.backend, enabled=True, hostname=platform.node(),
                    platform=platform.platform(), architecture=platform.machine(),
                    cpu_count=os.cpu_count(), clock_ticks=os.sysconf("SC_CLK_TCK"),
                    started_unix=time.time(), interval_seconds=args.interval,
                    fast_cpu_enabled=fast_cpu_enabled,
                    fast_cpu_interval_seconds=0.1 if fast_cpu_enabled else None,
                    thread_cpu_interval_seconds=1.0 if fast_cpu_enabled else None,
                    requested_cpu_frequency_hz=args.frequency if args.backend in ("perf", "gperftools") else None,
                    cpu_sample_clock="ITIMER_PROF (user+system CPU timer, user stacks)" if args.backend == "gperftools" else "cpu-clock:u" if args.backend == "perf" else None,
                    ros={key: os.environ.get(key) for key in
                         ("ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "AMENT_PREFIX_PATH")},
                    perf_event_paranoid=read_text("/proc/sys/kernel/perf_event_paranoid"),
                    memory_measurement="tcmalloc allocation stack profiles plus RSS/PSS/HWM" if args.backend == "heap" else "sampled live-heap estimates plus RSS/PSS/HWM; no cumulative allocations" if args.backend == "heap_sampled" else "sampled RSS/PSS/HWM; no heap allocation stacks",
                    allocator_changed=args.backend in ("heap", "heap_sampled"),
                    profiler_library=file_identity(args.heap_lib if args.backend in ("heap", "heap_sampled") else args.libprofiler if args.backend == "gperftools" else None),
                    limitations=["Short-lived processes between samples may be missed; sampled CPU totals can undercount.",
                                 "Sum of RSS double-counts shared pages; PSS is proportional resident memory.",
                                 "Cgroup counters can include unrelated workloads in the inherited cgroup.",
                                 "Profiler overhead, CPU frequency and thermal state affect timings.",
                                 "Frequency changes alone do not prove thermal throttling; inspect available counter deltas and cooling states. Package counters may be repeated across CPUs and must not be summed.",
                                 "Heap mode replaces the allocator; its CPU timings cannot establish production CPU improvement.",
                                 "gperftools child coverage and profile flushes depend on child exit behavior; verify actual profile files."])
    write_json(output / "manifest.json", manifest)
    env = os.environ.copy()
    try:
        command = profiler_command(args, output, env)
    except (OSError, RuntimeError) as error:
        write_json(output / "summary.json", dict(status="backend_unavailable", error=str(error), command_started=False))
        print(str(error), file=sys.stderr)
        return 2
    if args.backend == "heap":
        manifest["heap_snapshot_environment"] = {name: env.get(name) for name in (
            "HEAP_PROFILE_TIME_INTERVAL", "HEAP_PROFILE_ALLOCATION_INTERVAL", "HEAP_PROFILE_INUSE_INTERVAL")}
        manifest["heap_unset_intervals"] = "Unset values use the selected gperftools library defaults"
    if args.backend == "heap_sampled":
        manifest["sampled_heap"] = args.sampled_setup
        manifest["limitations"].extend([
            "heap_sampled supports one directly launched dynamic native ELF, not launcher/interpreter trees or fork/exec children.",
            "Sampled live objects require Poisson correction; do not use alloc_space or sum snapshots as cumulative allocations."])
    if args.backend == "gperftools":
        manifest["native_cpu_boundary"] = args.native_target
    native_runtime = None
    received = []
    previous = {}
    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[number] = signal.signal(number, lambda signum, frame: received.append(signum))
    usage_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    started = time.monotonic()
    manifest["started_monotonic_ns"] = round(started * 1e9)
    manifest["sampling_started_unix"] = time.time()
    write_json(output / "manifest.json", manifest)
    child = None
    tree = None
    peak_rss = peak_pss = 0
    samples = 0
    last_cgroup = None
    failure = None
    cleanup_signals = []
    fast_cpu = None
    try:
        with (output / "command.log").open("w") as log, \
                (output / "processes.csv").open("w", newline="") as process_file, \
                (output / "resources.csv").open("w", newline="") as aggregate_file, \
                (output / "system.jsonl").open("w") as system_file:
            child = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            tree = ProcessTree(child.pid)
            if fast_cpu_enabled:
                fast_cpu = FastCpuSampler(output, started, manifest["clock_ticks"])
                fast_cpu.update(tree.known.values())
                fast_cpu.start()
            manifest.update(root_pid=child.pid, executed_command=command)
            write_json(output / "manifest.json", manifest)
            process_writer = csv.DictWriter(process_file, fieldnames=["elapsed_seconds", "pid", "starttime", "ppid", "pgrp", "name", "executable", "cmdline", "state", "user_ticks", "system_ticks", "rss_kib", "pss_kib", "hwm_kib", "peak_rss_kib", "peak_pss_kib", "peak_hwm_kib"])
            process_writer.writeheader()
            aggregate_writer = csv.DictWriter(aggregate_file, fieldnames=["elapsed_seconds", "live_processes", "observed_user_ticks", "observed_system_ticks", "rss_kib", "pss_kib", "pss_processes"])
            aggregate_writer.writeheader()
            while True:
                elapsed = time.monotonic() - started
                rows = tree.sample()
                if args.backend == "gperftools" and native_runtime is None:
                    native_runtime = native_cpu_snapshot(args, child, output)
                if fast_cpu:
                    fast_cpu.update(tree.known.values())
                for row in rows:
                    process_writer.writerow(dict(elapsed_seconds=elapsed, **row))
                rss = sum(row["rss_kib"] or 0 for row in rows)
                pss = sum(row["pss_kib"] or 0 for row in rows)
                peak_rss, peak_pss = max(peak_rss, rss), max(peak_pss, pss)
                aggregate_writer.writerow(dict(elapsed_seconds=elapsed, live_processes=len(rows),
                    observed_user_ticks=sum(row["user_ticks"] for row in tree.known.values()),
                    observed_system_ticks=sum(row["system_ticks"] for row in tree.known.values()),
                    rss_kib=rss, pss_kib=pss, pss_processes=sum(row["pss_kib"] is not None for row in rows)))
                last_cgroup = cgroup_snapshot(child.pid) if child.poll() is None else last_cgroup
                system_file.write(json.dumps(dict(elapsed_seconds=elapsed, hardware=hardware_snapshot(), cgroup=last_cgroup)) + "\n")
                samples += 1
                for stream in (process_file, aggregate_file, system_file):
                    stream.flush()
                if received or child.poll() is not None:
                    break
                time.sleep(args.interval)
    except BaseException as error:
        failure = repr(error)
    finally:
        if child is not None and tree is not None:
            cleanup_signals = shutdown(tree, child, args.grace,
                                       received[0] if received else signal.SIGINT, args.signal_root_only)
            child.wait()
        if fast_cpu:
            fast_cpu.update(tree.known.values())
            fast_cpu.stop()
            if fast_cpu.error:
                failure = failure or "Fast CPU sampler failed: " + fast_cpu.error
        for number, handler in previous.items():
            signal.signal(number, handler)
    usage_after = resource.getrusage(resource.RUSAGE_CHILDREN)
    returncode = child.returncode if child is not None else None
    exitcode = 128 + received[0] if received else (returncode if returncode is not None and returncode >= 0 else 128 - returncode if returncode is not None else 1)
    if failure:
        exitcode = exitcode or 1
    profile_files = [dict(path=str(path.relative_to(output)), bytes=path.stat().st_size)
                     for directory in (output / "cpu_profiles", output / "heap_profiles")
                     if directory.is_dir() for path in sorted(directory.iterdir()) if path.is_file()]
    if args.backend in ("gperftools", "heap") and not any(row["bytes"] for row in profile_files):
        failure = failure or "No nonempty profile was flushed; check command.log and the profiler library/exit behavior"
        exitcode = exitcode or 1
    native_result = None
    if args.backend == "gperftools":
        try:native_result = native_cpu_result(args, native_runtime, profile_files, returncode, received)
        except (OSError, RuntimeError) as error:
            failure = failure or str(error);exitcode = exitcode or 1
    sampled_heap = None
    if args.backend == "heap_sampled":
        try:
            sampled_heap = sampled_heap_result(output, args.sampled_setup, child.pid if child else None,
                                                list(tree.known.values()) if tree else [])
            profile_files = sampled_heap["profile_files"]
        except RuntimeError as error:
            failure = failure or str(error)
            exitcode = exitcode or 1
    summary = dict(status="runner_error" if failure else "interrupted" if received else "completed",
        command_started=child is not None, command_returncode=returncode, returncode=exitcode,
        received_signal=received[0] if received else None, error=failure,
        elapsed_seconds=time.monotonic() - started, samples=samples,
        cleanup_signals=cleanup_signals,
        observed_user_ticks=sum(row["user_ticks"] for row in tree.known.values()) if tree else 0,
        observed_system_ticks=sum(row["system_ticks"] for row in tree.known.values()) if tree else 0,
        sampled_tree_peak_rss_kib=peak_rss, sampled_tree_peak_pss_kib=peak_pss,
        waited_child_user_seconds=usage_after.ru_utime - usage_before.ru_utime,
        waited_child_system_seconds=usage_after.ru_stime - usage_before.ru_stime,
        waited_child_maxrss_kib=usage_after.ru_maxrss,
        waited_child_maxrss_scope="largest single waited child, not simultaneous tree memory; includes backend preflight",
        profile_files=profile_files,
        observed_processes=list(tree.known.values()) if tree else [],
        remaining_owned_pids=[row["pid"] for row in tree.live()] if tree else [])
    if args.backend == "gperftools":summary["native_cpu_boundary"] = native_result
    if verified_prefix_shutdown(args, received, returncode, failure, native_result,
                                cleanup_signals, summary['remaining_owned_pids']):
        exitcode = 0
        summary['returncode'] = 0
        summary['verified_graceful_prefix_shutdown'] = True
        summary['prefix_signal_policy'] = 'Only explicit native prefix with unique SIGINT, native exit0, valid maps/profile and no force/residual is clean; original received signal is retained'
    if sampled_heap is not None:
        summary["sampled_heap"] = sampled_heap
    if fast_cpu:
        summary["fast_cpu"] = fast_cpu.summary()
        write_json(output / "fast_cpu_summary.json", summary["fast_cpu"])
    write_json(output / "summary.json", summary)
    print(str(output / "summary.json"))
    return exitcode


if __name__ == "__main__":
    raise SystemExit(main())
