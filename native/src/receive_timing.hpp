#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <ostream>
#include <time.h>

struct ReceiveTiming {
  static std::int64_t time_ns(clockid_t clock) {
    timespec t{};
    clock_gettime(clock, &t);
    return t.tv_sec*1'000'000'000LL+t.tv_nsec;
  }
  struct Sample {
    std::size_t slot = 0;
    std::array<std::size_t, 4> frames{};
    std::size_t rms_updates = 0;
    std::int64_t wall_ns = 0, cpu_ns = 0, read_ns = 0, monitor_ns = 0;
  } current, maximum;
  std::array<Sample, 16> slow{};
  std::size_t slow_count = 0, total_frames = 0, total_rms_updates = 0;
  std::int64_t wall_start = 0, cpu_start = 0;
  void begin(std::size_t slot) {
    current = {};
    current.slot = slot;
    wall_start = time_ns(CLOCK_MONOTONIC);
    cpu_start = time_ns(CLOCK_THREAD_CPUTIME_ID);
  }
  void finish() {
    current.cpu_ns = time_ns(CLOCK_THREAD_CPUTIME_ID)-cpu_start;
    current.wall_ns = time_ns(CLOCK_MONOTONIC)-wall_start;
    for (auto n : current.frames) total_frames += n;
    total_rms_updates += current.rms_updates;
    if (current.wall_ns > maximum.wall_ns) maximum = current;
    if (current.wall_ns >= 200'000) {
      if (slow_count < slow.size()) slow[slow_count] = current;
      ++slow_count;
    }
  }
  static void sample_json(std::ostream& out, const Sample& s) {
    out << "{\"slot\":" << s.slot << ",\"frames_by_bus\":[";
    for (std::size_t i=0;i<4;++i) { if (i) out << ','; out << s.frames[i]; }
    out << "],\"rms_updates\":" << s.rms_updates << ",\"wall_ms\":" << s.wall_ns/1e6
        << ",\"thread_cpu_ms\":" << s.cpu_ns/1e6 << ",\"read_syscalls_ms\":" << s.read_ns/1e6
        << ",\"rms_temperature_ms\":" << s.monitor_ns/1e6 << '}';
  }
  void json_fields(std::ostream& out) const {
    out << "  \"receive_diagnostics\":{\"total_frames\":" << total_frames
        << ",\"total_rms_updates\":" << total_rms_updates << ",\"maximum\":";
    sample_json(out, maximum);
    out << ",\"slow_samples\":[";
    for (std::size_t i=0;i<std::min(slow_count,slow.size());++i) {
      if (i) out << ',';
      sample_json(out, slow[i]);
    }
    out << "],\"slow_count\":" << slow_count << "},\n";
  }
};
