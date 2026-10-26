// Private opt-in diagnostic only. Uses the matching real gperftools 2.15 API.
#include <gperftools/heap-profiler.h>
#include <gperftools/malloc_extension.h>

#include <pthread.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <fcntl.h>
#include <dlfcn.h>

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>

namespace {
struct State {
  pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t condition;
  pthread_t thread{};
  pid_t owner = 0;
  bool started = false;
  bool stop = false;
  bool request = false;
  char label[96]{};
  const char* root = nullptr;  // Environment storage remains alive until exit.
  double interval = 10.;
  uint64_t completed = 0;
  uint64_t errors = 0;
};
State state;

std::string Quote(const std::string& value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
    else if (c < 0x20) out << "\\u00" << "0123456789abcdef"[c >> 4]
                           << "0123456789abcdef"[c & 15];
    else out << static_cast<char>(c);
  }
  out << '"';
  return out.str();
}

uint64_t Clock(clockid_t clock) {
  timespec value{};
  if (clock_gettime(clock, &value) != 0) throw std::runtime_error("clock_gettime failed");
  return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL + value.tv_nsec;
}

std::string Read(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("Cannot read " + path);
  std::ostringstream out;
  out << input.rdbuf();
  if (!input.eof() && input.fail()) throw std::runtime_error("Cannot finish reading " + path);
  return out.str();
}

void WriteNew(const std::string& path, const std::string& data) {
  int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
  if (fd < 0) throw std::runtime_error("Cannot create " + path + ": " + std::strerror(errno));
  size_t offset = 0;
  while (offset < data.size()) {
    ssize_t count = write(fd, data.data() + offset, data.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      int error = errno;
      close(fd);
      throw std::runtime_error("Cannot write " + path + ": " + std::strerror(error));
    }
    offset += static_cast<size_t>(count);
  }
  if (close(fd) != 0) throw std::runtime_error("Cannot close " + path);
}

std::string Executable() {
  char path[PATH_MAX + 1];
  ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
  if (count <= 0 || count == PATH_MAX) throw std::runtime_error("Cannot resolve native executable");
  return std::string(path, static_cast<size_t>(count));
}

uint64_t Starttime() {
  const std::string stat = Read("/proc/self/stat");
  const size_t end = stat.rfind(") ");
  if (end == std::string::npos) throw std::runtime_error("Malformed self stat");
  std::istringstream fields(stat.substr(end + 2));
  std::string token;
  for (int i = 0; i <= 19; ++i) {
    if (!(fields >> token)) throw std::runtime_error("Missing self starttime");
  }
  return std::stoull(token);
}

void Warn(const char* message) {
  const char prefix[] = "[LOCNAV_HEAP_SAMPLE] ";
  (void)!write(STDERR_FILENO, prefix, sizeof(prefix) - 1);
  (void)!write(STDERR_FILENO, message, std::strlen(message));
  (void)!write(STDERR_FILENO, "\n", 1);
}

std::string AllocatorLibrary() {
  Dl_info info{};
  void* symbol = dlsym(RTLD_DEFAULT, "tc_malloc");
  if (!symbol || dladdr(symbol, &info) == 0 || !info.dli_fname)
    throw std::runtime_error("Cannot identify tcmalloc library");
  return info.dli_fname;
}

void Capture(const std::string& directory, uint64_t index, const std::string& reason,
             uint64_t starttime, const std::string& exe) {
  const uint64_t begin = Clock(CLOCK_MONOTONIC);
  const uint64_t cpu_begin = Clock(CLOCK_THREAD_CPUTIME_ID);
  std::string sample;
  MallocExtension* extension = MallocExtension::instance();
  size_t allocated = 0;
  if (!extension->GetNumericProperty("generic.current_allocated_bytes", &allocated))
    throw std::runtime_error("MallocExtension is not the real tcmalloc implementation");
  if (IsHeapProfilerRunning()) throw std::runtime_error("Full allocation heap profiler is unexpectedly active");
  // Virtual call through the actual 2.15 header and implementation, never a stub.
  extension->GetHeapSample(&sample);
  const uint64_t captured_cpu = Clock(CLOCK_THREAD_CPUTIME_ID);
  const uint64_t captured_wall = Clock(CLOCK_MONOTONIC);
  const std::string expected = "@ heap_v2/524288";
  const size_t newline = sample.find('\n');
  const std::string header = sample.substr(0, newline);
  if (header.rfind("heap profile:", 0) != 0 || header.find(expected) == std::string::npos)
    throw std::runtime_error("Missing real sampled heap_v2/524288 header: " + header);
  if (sample.find("MAPPED_LIBRARIES:") == std::string::npos)
    throw std::runtime_error("Sample is missing its own address map");
  const std::string stem = directory + "/sample_" + std::to_string(index);
  WriteNew(stem + ".heap", sample);  // No fixed 1 MiB buffer/truncation.
  WriteNew(stem + ".maps", Read("/proc/self/maps"));
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "{\"valid\":true,\"kind\":\"sampled_live_heap_estimate\",\"pid\":" << getpid()
      << ",\"starttime\":" << starttime << ",\"worker_tid\":" << syscall(SYS_gettid)
      << ",\"executable\":" << Quote(exe) << ",\"index\":" << index
      << ",\"reason\":" << Quote(reason) << ",\"monotonic_begin_ns\":" << begin
      << ",\"monotonic_captured_ns\":" << captured_wall
      << ",\"collection_wall_ns\":" << captured_wall - begin
      << ",\"collection_thread_cpu_ns\":" << captured_cpu - cpu_begin
      << ",\"collection_and_io_wall_ns\":" << Clock(CLOCK_MONOTONIC) - begin
      << ",\"collection_and_io_thread_cpu_ns\":" << Clock(CLOCK_THREAD_CPUTIME_ID) - cpu_begin
      << ",\"profile_bytes\":" << sample.size() << ",\"raw_header\":" << Quote(header)
      << ",\"allocator_current_allocated_bytes_before_collection\":" << allocated
      << ",\"sample_parameter\":524288,\"full_heap_profiler_running\":false"
      << ",\"cumulative_allocations_available\":false}\n";
  WriteNew(stem + ".json", out.str());
}

void* Worker(void*) noexcept {
  std::string directory;
  uint64_t starttime = 0;
  try {
    starttime = Starttime();
    const std::string exe = Executable();
    directory = std::string(state.root) + "/" + std::to_string(getpid()) + "-" + std::to_string(starttime);
    if (mkdir(directory.c_str(), 0755) != 0) throw std::runtime_error("Cannot create unique PID/starttime output");
    const char* expected_exe = std::getenv("LOCNAV_HEAP_SAMPLE_EXPECT_EXE");
    if (!expected_exe || exe != expected_exe) throw std::runtime_error("Native executable identity mismatch");
    const char* parameter = std::getenv("TCMALLOC_SAMPLE_PARAMETER");
    if (!parameter || std::strcmp(parameter, "524288") != 0)
      throw std::runtime_error("TCMALLOC_SAMPLE_PARAMETER must be exactly 524288");
    for (const char* name : {"HEAPPROFILE", "HEAPCHECK", "CPUPROFILE"}) {
      const char* value = std::getenv(name);
      if (value && *value) throw std::runtime_error(std::string("Conflicting environment: ") + name);
    }
    MallocExtension* extension = MallocExtension::instance();
    size_t allocated = 0;
    if (IsHeapProfilerRunning() || !extension->GetNumericProperty("generic.current_allocated_bytes", &allocated))
      throw std::runtime_error("Full heap profiler active or allocator API is a stub");
    std::ostringstream manifest;
    manifest.imbue(std::locale::classic());
    manifest << "{\"enabled\":true,\"pid\":" << getpid() << ",\"starttime\":" << starttime
             << ",\"worker_tid\":" << syscall(SYS_gettid) << ",\"executable\":" << Quote(exe)
             << ",\"allocator_library\":" << Quote(AllocatorLibrary())
             << ",\"malloc_extension_dynamic_type\":" << Quote(typeid(*extension).name())
             << ",\"interval_seconds\":" << std::setprecision(17) << state.interval
             << ",\"sample_parameter\":524288,\"full_heap_profiler_running\":false"
             << ",\"api\":\"real-header virtual MallocExtension::instance()->GetHeapSample\"}\n";
    WriteNew(directory + "/sampler_manifest.json", manifest.str());
    for (;;) {
      const uint64_t deadline_ns = Clock(CLOCK_MONOTONIC) + static_cast<uint64_t>(state.interval * 1e9);
      const timespec deadline{static_cast<time_t>(deadline_ns / 1000000000ULL),
                              static_cast<long>(deadline_ns % 1000000000ULL)};
      pthread_mutex_lock(&state.mutex);
      int waited = 0;
      while (!state.stop && !state.request && waited != ETIMEDOUT) {
        waited = pthread_cond_timedwait(&state.condition, &state.mutex, &deadline);
        if (waited != 0 && waited != ETIMEDOUT) {
          pthread_mutex_unlock(&state.mutex);
          throw std::runtime_error("pthread_cond_timedwait failed");
        }
      }
      const bool stop = state.stop;
      const bool explicit_request = state.request;
      char requested_label[sizeof(state.label)];
      std::memcpy(requested_label, state.label, sizeof(requested_label));
      state.request = false;
      const uint64_t index = state.completed + 1;
      pthread_mutex_unlock(&state.mutex);
      if (stop) break;
      // Constructing a long label can throw. Never allocate while holding the
      // mutex: the exception collector acquires it and exit must remain joinable.
      Capture(directory, index, explicit_request ? std::string(requested_label) : std::string("periodic"), starttime, exe);
      pthread_mutex_lock(&state.mutex);
      ++state.completed;
      pthread_mutex_unlock(&state.mutex);
    }
  } catch (const std::exception& error) {
    Warn(error.what());
    pthread_mutex_lock(&state.mutex);
    ++state.errors;
    pthread_mutex_unlock(&state.mutex);
    if (!directory.empty()) {
      try { WriteNew(directory + "/sampler_error.json", "{\"valid\":false,\"error\":" + Quote(error.what()) + "}\n"); }
      catch (...) { Warn("Cannot persist sampler error"); }
    }
  } catch (...) {
    Warn("Unknown sampler failure");
    pthread_mutex_lock(&state.mutex);
    ++state.errors;
    pthread_mutex_unlock(&state.mutex);
  }
  if (!directory.empty()) {
    try {
      std::ostringstream out;
      out << "{\"pid\":" << getpid() << ",\"starttime\":" << starttime
          << ",\"completed\":" << state.completed << ",\"errors\":" << state.errors
          << ",\"worker_stopped\":true,\"final_empty_exit_heap_not_collected\":true}\n";
      WriteNew(directory + "/sampler_summary.json", out.str());
    } catch (...) { Warn("Cannot persist sampler summary"); }
  }
  return nullptr;
}

struct Lifetime {
  Lifetime() noexcept {
    const char* enabled = std::getenv("LOCNAV_HEAP_SAMPLE");
    if (!enabled || std::strcmp(enabled, "1") != 0) return;
    state.root = std::getenv("LOCNAV_HEAP_SAMPLE_DIR");
    if (!state.root || state.root[0] != '/') { Warn("Absolute sample output directory required"); return; }
    const char* interval = std::getenv("LOCNAV_HEAP_SAMPLE_INTERVAL_SECONDS");
    if (interval) {
      char* end = nullptr;
      const double value = std::strtod(interval, &end);
      if (!end || *end || !std::isfinite(value) || value < .02 || value > 3600.) {
        Warn("Invalid sample interval"); return;
      }
      state.interval = value;
    }
    pthread_condattr_t attr;
    if (pthread_condattr_init(&attr) != 0) { Warn("Condition attribute initialization failed"); return; }
    const int clock_error = pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    const int cond_error = clock_error ? clock_error : pthread_cond_init(&state.condition, &attr);
    pthread_condattr_destroy(&attr);
    if (cond_error) { Warn("Monotonic condition initialization failed"); return; }
    state.owner = getpid();
    if (pthread_create(&state.thread, nullptr, Worker, nullptr) != 0) {
      pthread_cond_destroy(&state.condition); Warn("Cannot start sample worker"); return;
    }
    state.started = true;
  }
  ~Lifetime() noexcept {
    if (!state.started || state.owner != getpid()) return;  // No inherited join after fork.
    pthread_mutex_lock(&state.mutex);
    state.stop = true;
    pthread_cond_signal(&state.condition);
    pthread_mutex_unlock(&state.mutex);
    if (pthread_join(state.thread, nullptr) != 0) Warn("Sampler worker join failed");
    pthread_cond_destroy(&state.condition);
  }
};
Lifetime lifetime;
}  // namespace

// Test/explicit ordinary-thread trigger. These functions are NOT async-signal-safe.
extern "C" int LocnavHeapSampleRequest(const char* label) noexcept {
  if (!state.started || state.owner != getpid()) return ENODEV;
  pthread_mutex_lock(&state.mutex);
  if (state.stop || state.request || state.errors) {
    pthread_mutex_unlock(&state.mutex);
    return EBUSY;
  }
  std::strncpy(state.label, label ? label : "explicit", sizeof(state.label) - 1);
  state.label[sizeof(state.label) - 1] = '\0';
  state.request = true;
  pthread_cond_signal(&state.condition);
  pthread_mutex_unlock(&state.mutex);
  return 0;
}

extern "C" uint64_t LocnavHeapSampleCompleted() noexcept {
  if (!state.started || state.owner != getpid()) return 0;
  pthread_mutex_lock(&state.mutex);
  const uint64_t value = state.completed;
  pthread_mutex_unlock(&state.mutex);
  return value;
}
