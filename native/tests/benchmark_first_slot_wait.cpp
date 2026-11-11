// Timing-only diagnostic. No CAN, SPI, motor, or policy access.
#include "../src/first_slot_wait.hpp"
#include <algorithm>
#include <cerrno>
#include <iostream>
#include <sched.h>
#include <time.h>
#include <vector>

std::int64_t now_ns() {
  timespec t{};
  if (clock_gettime(CLOCK_MONOTONIC, &t)) throw std::runtime_error("clock_gettime failed");
  return t.tv_sec*1'000'000'000LL+t.tv_nsec;
}
void sleep_at(std::int64_t ns) {
  timespec t{ns/1'000'000'000LL, ns%1'000'000'000LL};
  int status;
  do status = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, nullptr);
  while (status == EINTR);
  if (status) throw std::runtime_error("sleep failed");
}
int main() {
  cpu_set_t cpus;
  CPU_ZERO(&cpus);
  CPU_SET(5, &cpus);
  if (sched_setaffinity(0, sizeof(cpus), &cpus)) throw std::runtime_error("CPU5 affinity failed");
  std::vector<double> samples[2];
  // Interleave treatments, with 100ms idle before every first deadline.
  for (int trial=0; trial<100; ++trial) {
    for (int j=0; j<2; ++j) {
      const int lead = (trial+j)%2;
      const auto deadline = now_ns()+100'000'000;
      if (lead) wait_first_slot(deadline, now_ns, sleep_at, [] { return false; });
      else sleep_at(deadline);
      samples[lead].push_back((now_ns()-deadline)/1e6);
    }
  }
  std::cout << "{\"no_hardware_access\":true,\"cpu\":5,\"scheduler_id\":"
            << sched_getscheduler(0) << ",\"measure\":\"wakeup_only_not_CAN_work\",\"results\":[";
  for (int lead=0; lead<2; ++lead) {
    auto& v = samples[lead];
    std::sort(v.begin(), v.end());
    if (lead) std::cout << ',';
    std::cout << "{\"lead_ns\":" << (lead ? kFirstSlotWakeLeadNs : 0)
              << ",\"trials\":" << v.size() << ",\"median_ms\":" << v[v.size()/2]
              << ",\"p99_ms\":" << v[98] << ",\"max_ms\":" << v.back() << '}';
  }
  std::cout << "]}\n";
}
