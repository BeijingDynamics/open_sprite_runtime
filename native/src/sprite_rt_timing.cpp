#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

namespace {

struct Options {
  double duration_s = 10.0;
  int rate_hz = 500;
  int cpu = -1;
  double maximum_p99_lateness_ms = 0.5;
  double maximum_lateness_ms = 2.0;
  std::string output;
};

struct TimingReport {
  std::size_t ticks = 0;
  std::size_t deadline_misses = 0;
  double elapsed_s = 0.0;
  double lateness_p50_ms = 0.0;
  double lateness_p95_ms = 0.0;
  double lateness_p99_ms = 0.0;
  double lateness_max_ms = 0.0;
  double work_p99_ms = 0.0;
  double work_max_ms = 0.0;
  double maximum_p99_lateness_ms = 0.0;
  double maximum_lateness_ms = 0.0;
  int requested_cpu = -1;
  int running_cpu = -1;
  int scheduler_policy = 0;
  int scheduler_priority = 0;
  bool affinity_applied = false;
  bool memory_locked = false;
  double checksum = 0.0;
  bool passed = false;
};

std::int64_t to_ns(const timespec& value) {
  return static_cast<std::int64_t>(value.tv_sec) * 1'000'000'000LL + value.tv_nsec;
}

timespec from_ns(std::int64_t value) {
  timespec result{};
  result.tv_sec = value / 1'000'000'000LL;
  result.tv_nsec = value % 1'000'000'000LL;
  return result;
}

std::int64_t monotonic_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::runtime_error(std::string("clock_gettime failed: ") + std::strerror(errno));
  }
  return to_ns(value);
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto index = std::min(
      values.size() - 1,
      static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(values.size()))) - 1);
  return values[index];
}

int parse_int(const char* value, const char* name) {
  std::size_t consumed = 0;
  const int result = std::stoi(value, &consumed);
  if (consumed != std::strlen(value)) {
    throw std::runtime_error(std::string("invalid ") + name);
  }
  return result;
}

double parse_double(const char* value, const char* name) {
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != std::strlen(value) || !std::isfinite(result)) {
    throw std::runtime_error(std::string("invalid ") + name);
  }
  return result;
}

Options parse_options(int argc, char** argv) {
  Options result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto require_value = [&]() -> const char* {
      if (++index >= argc) {
        throw std::runtime_error("missing value after " + argument);
      }
      return argv[index];
    };
    if (argument == "--duration") {
      result.duration_s = parse_double(require_value(), "duration");
    } else if (argument == "--rate") {
      result.rate_hz = parse_int(require_value(), "rate");
    } else if (argument == "--cpu") {
      result.cpu = parse_int(require_value(), "cpu");
    } else if (argument == "--maximum-p99-lateness-ms") {
      result.maximum_p99_lateness_ms = parse_double(require_value(), "P99 limit");
    } else if (argument == "--maximum-lateness-ms") {
      result.maximum_lateness_ms = parse_double(require_value(), "maximum lateness limit");
    } else if (argument == "--output") {
      result.output = require_value();
    } else if (argument == "--help") {
      std::cout
          << "usage: sprite_rt_timing [--duration SEC] [--rate HZ] [--cpu N] "
             "[--maximum-p99-lateness-ms MS] [--maximum-lateness-ms MS] "
             "[--output FILE]\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  if (!(result.duration_s > 0.0 && result.duration_s <= 3600.0)) {
    throw std::runtime_error("duration must be in (0, 3600]");
  }
  if (result.rate_hz <= 0 || result.rate_hz > 5000) {
    throw std::runtime_error("rate must be in (0, 5000]");
  }
  if (result.maximum_p99_lateness_ms < 0.0 || result.maximum_lateness_ms < 0.0) {
    throw std::runtime_error("lateness limits must be non-negative");
  }
  return result;
}

bool apply_affinity(int cpu) {
  if (cpu < 0) {
    return false;
  }
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
    throw std::runtime_error("failed to apply CPU affinity");
  }
  return true;
}

double representative_ankle_work(std::size_t tick) {
  const double phase = static_cast<double>(tick % 1000) * 0.001;
  double checksum = 0.0;
  for (int pair = 0; pair < 2; ++pair) {
    const double pitch_error = 0.12 * std::sin(phase + pair);
    const double roll_error = 0.08 * std::cos(phase - pair);
    const double pitch_velocity = 0.6 * std::cos(phase + pair);
    const double roll_velocity = -0.4 * std::sin(phase - pair);
    const double pitch_torque = 35.0 * pitch_error - 1.5 * pitch_velocity;
    const double roll_torque = 25.0 * roll_error - 1.2 * roll_velocity;
    const double a00 = pair == 0 ? -1.30469867345466 : 1.3055003739519502;
    const double a01 = pair == 0 ? -1.0115688924461703 : -1.011683046199467;
    const double a10 = pair == 0 ? 1.3049296051854669 : -1.3051579126920603;
    const double a11 = pair == 0 ? -1.0113392728274706 : -1.012140973324761;
    const double determinant = a00 * a11 - a01 * a10;
    const double motor_a = (a11 * pitch_torque - a10 * roll_torque) / determinant;
    const double motor_b = (-a01 * pitch_torque + a00 * roll_torque) / determinant;
    checksum += motor_a + motor_b;
  }
  return checksum;
}

TimingReport run(const Options& options) {
  const std::int64_t period_ns = 1'000'000'000LL / options.rate_hz;
  const std::size_t requested_ticks =
      static_cast<std::size_t>(std::llround(options.duration_s * options.rate_hz));
  std::vector<double> lateness_ms(requested_ticks);
  std::vector<double> work_ms(requested_ticks);
  const bool affinity_applied = apply_affinity(options.cpu);
  const bool memory_locked = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;

  const std::int64_t start_ns = monotonic_ns() + 100'000'000LL;
  double checksum = 0.0;
  std::size_t deadline_misses = 0;
  for (std::size_t tick = 0; tick < requested_ticks; ++tick) {
    const std::int64_t deadline_ns = start_ns + static_cast<std::int64_t>(tick) * period_ns;
    const timespec deadline = from_ns(deadline_ns);
    int status = 0;
    do {
      status = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    } while (status == EINTR);
    if (status != 0) {
      throw std::runtime_error(std::string("clock_nanosleep failed: ") + std::strerror(status));
    }
    const std::int64_t woke_ns = monotonic_ns();
    lateness_ms[tick] = std::max(0.0, static_cast<double>(woke_ns - deadline_ns) / 1.0e6);
    deadline_misses += static_cast<std::size_t>(woke_ns - deadline_ns >= period_ns);
    const std::int64_t work_start_ns = monotonic_ns();
    checksum += representative_ankle_work(tick);
    work_ms[tick] = static_cast<double>(monotonic_ns() - work_start_ns) / 1.0e6;
  }
  const std::int64_t finish_ns = monotonic_ns();
  sched_param current_scheduler{};
  int current_policy = 0;
  if (pthread_getschedparam(pthread_self(), &current_policy, &current_scheduler) != 0) {
    throw std::runtime_error("pthread_getschedparam failed");
  }

  TimingReport report;
  report.ticks = requested_ticks;
  report.deadline_misses = deadline_misses;
  report.elapsed_s = static_cast<double>(finish_ns - start_ns) / 1.0e9;
  report.lateness_p50_ms = percentile(lateness_ms, 0.50);
  report.lateness_p95_ms = percentile(lateness_ms, 0.95);
  report.lateness_p99_ms = percentile(lateness_ms, 0.99);
  report.lateness_max_ms = *std::max_element(lateness_ms.begin(), lateness_ms.end());
  report.work_p99_ms = percentile(work_ms, 0.99);
  report.work_max_ms = *std::max_element(work_ms.begin(), work_ms.end());
  report.maximum_p99_lateness_ms = options.maximum_p99_lateness_ms;
  report.maximum_lateness_ms = options.maximum_lateness_ms;
  report.requested_cpu = options.cpu;
  report.running_cpu = sched_getcpu();
  report.scheduler_policy = current_policy;
  report.scheduler_priority = current_scheduler.sched_priority;
  report.affinity_applied = affinity_applied;
  report.memory_locked = memory_locked;
  report.checksum = checksum;
  report.passed = report.deadline_misses == 0 &&
                  report.lateness_p99_ms <= options.maximum_p99_lateness_ms &&
                  report.lateness_max_ms <= options.maximum_lateness_ms;
  return report;
}

std::string to_json(const TimingReport& report) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(6);
  output << "{\n"
         << "  \"mode\": \"native_500hz_timing_no_hardware_tx\",\n"
         << "  \"ticks\": " << report.ticks << ",\n"
         << "  \"deadline_misses\": " << report.deadline_misses << ",\n"
         << "  \"elapsed_s\": " << report.elapsed_s << ",\n"
         << "  \"lateness_p50_ms\": " << report.lateness_p50_ms << ",\n"
         << "  \"lateness_p95_ms\": " << report.lateness_p95_ms << ",\n"
         << "  \"lateness_p99_ms\": " << report.lateness_p99_ms << ",\n"
         << "  \"lateness_max_ms\": " << report.lateness_max_ms << ",\n"
         << "  \"work_p99_ms\": " << report.work_p99_ms << ",\n"
         << "  \"work_max_ms\": " << report.work_max_ms << ",\n"
         << "  \"maximum_p99_lateness_ms\": " << report.maximum_p99_lateness_ms << ",\n"
         << "  \"maximum_lateness_ms\": " << report.maximum_lateness_ms << ",\n"
         << "  \"requested_cpu\": " << report.requested_cpu << ",\n"
         << "  \"running_cpu\": " << report.running_cpu << ",\n"
         << "  \"scheduler_policy\": " << report.scheduler_policy << ",\n"
         << "  \"scheduler_priority\": " << report.scheduler_priority << ",\n"
         << "  \"affinity_applied\": " << (report.affinity_applied ? "true" : "false") << ",\n"
         << "  \"memory_locked\": " << (report.memory_locked ? "true" : "false") << ",\n"
         << "  \"checksum\": " << report.checksum << ",\n"
         << "  \"hardware_tx_attempts\": 0,\n"
         << "  \"passed\": " << (report.passed ? "true" : "false") << "\n"
         << "}\n";
  return output.str();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const TimingReport report = run(options);
    const std::string json = to_json(report);
    std::cout << json;
    if (!options.output.empty()) {
      std::ofstream file(options.output);
      if (!file) {
        throw std::runtime_error("cannot open output file: " + options.output);
      }
      file << json;
    }
    return report.passed ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}
