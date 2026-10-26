#include <dlfcn.h>
#include <unistd.h>
#include <dirent.h>
#include <cerrno>
#include <csignal>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
using Request = int (*)(const char*);
using Completed = uint64_t (*)();
constexpr size_t kBlock = 8 * 1024 * 1024;
volatile std::sig_atomic_t interrupted = 0;
void NoteSignal(int) { interrupted = 1; }  // No allocation/collection in a handler.

void Capture(const char* label, uint64_t expected) {
  const auto request = reinterpret_cast<Request>(dlsym(RTLD_DEFAULT, "LocnavHeapSampleRequest"));
  const auto completed = reinterpret_cast<Completed>(dlsym(RTLD_DEFAULT, "LocnavHeapSampleCompleted"));
  if (!request || !completed || request(label) != 0) throw std::runtime_error("Cannot request sample");
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (completed() < expected) {
    if (std::chrono::steady_clock::now() > end) throw std::runtime_error("Sample did not complete");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

int Tasks() {
  DIR* directory = opendir("/proc/self/task");
  if (!directory) return -1;
  int count = 0;
  while (dirent* entry = readdir(directory)) if (entry->d_name[0] != '.') ++count;
  closedir(directory);
  return count;
}
}  // namespace

__attribute__((noinline)) void* AllocateRetained() {
  void* p = std::malloc(kBlock);
  if (!p) throw std::bad_alloc();
  static_cast<char*>(p)[0] = 1;
  static_cast<char*>(p)[kBlock - 1] = 1;
  return p;
}
__attribute__((noinline)) void* AllocateReleasable() {
  void* p = std::malloc(kBlock);
  if (!p) throw std::bad_alloc();
  static_cast<char*>(p)[0] = 2;
  static_cast<char*>(p)[kBlock - 1] = 2;
  return p;
}
__attribute__((noinline)) void ConcurrentAllocations() {
  for (int round = 0; round < 40; ++round) {
    std::vector<void*> objects;
    for (int i = 0; i < 128; ++i) objects.push_back(std::malloc(4096));
    for (void* object : objects) std::free(object);
    std::this_thread::yield();
  }
}

int main(int argc, char** argv) {
  try {
    const bool wait_signal = argc == 2 && std::strcmp(argv[1], "--wait-signal") == 0;
    if (wait_signal) std::signal(SIGINT, NoteSignal);
    if (argc == 2 && std::strcmp(argv[1], "--disabled-probe") == 0) {
      std::cout << "{\"pid\":" << getpid() << ",\"tasks\":" << Tasks() << "}\n";
      return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--quick-exit") == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
      return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "--periodic-probe") == 0) {
      void* retained = AllocateRetained();
      std::this_thread::sleep_for(std::chrono::milliseconds(180));
      std::free(retained);
      return 0;
    }
    std::vector<void*> retained, releasable;
    for (int i = 0; i < 4; ++i) retained.push_back(AllocateRetained());
    for (int i = 0; i < 4; ++i) releasable.push_back(AllocateReleasable());
    std::vector<std::thread> workers;
    for (int i = 0; i < 4; ++i) workers.emplace_back(ConcurrentAllocations);
    Capture("retained_and_releasable\nquoted\"label", 1);
    for (auto& worker : workers) worker.join();
    for (void* object : releasable) std::free(object);
    Capture("after_free_releasable", 2);
    for (void* object : retained) std::free(object);
    Capture("all_large_objects_freed", 3);
    while (wait_signal && !interrupted) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (argc == 2 && std::strcmp(argv[1], "--nonzero") == 0) return 7;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
