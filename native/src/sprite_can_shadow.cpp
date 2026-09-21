#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace {

constexpr std::int64_t kTransportPeriodNs = 500'000LL;
constexpr int kTransportRateHz = 2000;

struct Motor {
  std::string name;
  std::string interface;
  int can_id = 0;
  int master_id = 0;
  double position_min = 0.0;
  double position_max = 0.0;
  double velocity_min = 0.0;
  double velocity_max = 0.0;
  double torque_min = 0.0;
  double torque_max = 0.0;
  int poll_rate_hz = 0;
  int poll_phase = 0;
  double last_position = 0.0;
  std::size_t tx_count = 0;
  std::size_t rx_count = 0;
  bool seen = false;
};

struct Options {
  std::string config;
  std::string output;
  double duration_s = 10.0;
  int cpu = 5;
  std::string acknowledgement;
  bool all_disabled = false;
};

struct Socket {
  std::string interface;
  int fd = -1;
};

std::int64_t monotonic_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::runtime_error("clock_gettime failed");
  }
  return static_cast<std::int64_t>(value.tv_sec) * 1'000'000'000LL + value.tv_nsec;
}

timespec from_ns(std::int64_t value) {
  return timespec{value / 1'000'000'000LL, value % 1'000'000'000LL};
}

std::vector<std::string> split(const std::string& value, char separator) {
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, separator)) {
    result.push_back(item);
  }
  return result;
}

double number(const std::string& value) {
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result)) {
    throw std::runtime_error("invalid numeric TSV field");
  }
  return result;
}

std::vector<Motor> load_motors(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open native motor config: " + path);
  }
  std::string line;
  if (!std::getline(file, line)) {
    throw std::runtime_error("native motor config is empty");
  }
  if (!line.empty() && line.back() == '\r') line.pop_back();
  const std::vector<std::string> expected_header = {
      "motor_name", "interface", "can_id", "master_id", "position_min_rad",
      "position_max_rad", "velocity_min_rad_s", "velocity_max_rad_s",
      "torque_min_nm", "torque_max_nm", "poll_rate_hz"};
  if (split(line, '\t') != expected_header) {
    throw std::runtime_error("native motor config header mismatch");
  }
  std::vector<Motor> motors;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) {
      continue;
    }
    const auto fields = split(line, '\t');
    if (fields.size() != expected_header.size()) {
      throw std::runtime_error("native motor config row has wrong field count");
    }
    Motor motor;
    motor.name = fields[0];
    motor.interface = fields[1];
    motor.can_id = static_cast<int>(number(fields[2]));
    motor.master_id = static_cast<int>(number(fields[3]));
    motor.position_min = number(fields[4]);
    motor.position_max = number(fields[5]);
    motor.velocity_min = number(fields[6]);
    motor.velocity_max = number(fields[7]);
    motor.torque_min = number(fields[8]);
    motor.torque_max = number(fields[9]);
    motor.poll_rate_hz = static_cast<int>(number(fields[10]));
    if (motor.can_id < 1 || motor.can_id > 8 || motor.master_id != motor.can_id + 0x10 ||
        (motor.poll_rate_hz != 50 && motor.poll_rate_hz != 500)) {
      throw std::runtime_error("native motor config endpoint/rate invariant failed");
    }
    motors.push_back(motor);
  }
  if (motors.size() != 31) {
    throw std::runtime_error("native motor config must contain 31 motors");
  }
  std::map<std::string, int> next_ankle_phase;
  std::map<std::string, int> next_other_phase;
  for (auto& motor : motors) {
    if (motor.poll_rate_hz == 500) {
      motor.poll_phase = 2 * next_ankle_phase[motor.interface]++;
      if (motor.poll_phase >= 4) {
        throw std::runtime_error("more than two 500 Hz motors configured on one CAN bus");
      }
    } else {
      motor.poll_phase = 1 + 2 * next_other_phase[motor.interface]++;
      if (motor.poll_phase >= 40) {
        throw std::runtime_error("too many 50 Hz motors for one 40-slot CAN schedule");
      }
    }
  }
  return motors;
}

Options parse_options(int argc, char** argv) {
  Options result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto value = [&]() -> std::string {
      if (++index >= argc) {
        throw std::runtime_error("missing value after " + argument);
      }
      return argv[index];
    };
    if (argument == "--config") result.config = value();
    else if (argument == "--output") result.output = value();
    else if (argument == "--duration") result.duration_s = number(value());
    else if (argument == "--cpu") result.cpu = static_cast<int>(number(value()));
    else if (argument == "--acknowledge-hardware-tx") result.acknowledgement = value();
    else if (argument == "--all-motors-disabled-confirmed") result.all_disabled = true;
    else throw std::runtime_error("unknown argument: " + argument);
  }
  if (result.config.empty() || result.output.empty()) {
    throw std::runtime_error("--config and --output are required");
  }
  if (!(result.duration_s > 0.0 && result.duration_s <= 120.0)) {
    throw std::runtime_error("duration must be in (0, 120]");
  }
  if (result.acknowledgement != "ZERO_GAIN_NATIVE_SHADOW" || !result.all_disabled) {
    throw std::runtime_error("explicit zero-gain TX acknowledgement and disabled confirmation required");
  }
  return result;
}

std::uint32_t encode_uint(double value, double low, double high, int bits) {
  if (!(low <= value && value <= high)) {
    throw std::runtime_error("MIT value outside configured range");
  }
  return static_cast<std::uint32_t>((value - low) / (high - low) * ((1U << bits) - 1U));
}

double decode_uint(std::uint32_t value, double low, double high, int bits) {
  return static_cast<double>(value) * (high - low) / ((1U << bits) - 1U) + low;
}

std::array<std::uint8_t, 8> zero_gain_payload(const Motor& motor) {
  const auto p = encode_uint(motor.last_position, motor.position_min, motor.position_max, 16);
  const auto v = encode_uint(0.0, motor.velocity_min, motor.velocity_max, 12);
  const auto torque = encode_uint(0.0, motor.torque_min, motor.torque_max, 12);
  const std::uint32_t kp = 0;
  const std::uint32_t kd = 0;
  std::array<std::uint8_t, 8> data{
      static_cast<std::uint8_t>(p >> 8), static_cast<std::uint8_t>(p),
      static_cast<std::uint8_t>(v >> 4), static_cast<std::uint8_t>((v & 0xF) << 4 | kp >> 8),
      static_cast<std::uint8_t>(kp), static_cast<std::uint8_t>(kd >> 4),
      static_cast<std::uint8_t>((kd & 0xF) << 4 | torque >> 8),
      static_cast<std::uint8_t>(torque)};
  const auto decoded_v = (static_cast<unsigned>(data[2]) << 4) | (data[3] >> 4);
  const auto decoded_kp = ((data[3] & 0xF) << 8) | data[4];
  const auto decoded_kd = (static_cast<unsigned>(data[5]) << 4) | (data[6] >> 4);
  const auto decoded_torque = ((data[6] & 0xF) << 8) | data[7];
  if ((decoded_v != 2047 && decoded_v != 2048) || decoded_kp != 0 || decoded_kd != 0 ||
      (decoded_torque != 2047 && decoded_torque != 2048)) {
    throw std::runtime_error("restricted writer rejected nonzero v/Kp/Kd/torque payload");
  }
  return data;
}

Socket open_socket(const std::string& interface) {
  const int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd < 0) throw std::runtime_error("failed to open CAN socket");
  int enable_fd = 1;
  if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) != 0) {
    close(fd);
    throw std::runtime_error("failed to enable CAN-FD");
  }
  ifreq request{};
  std::strncpy(request.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd, SIOCGIFINDEX, &request) != 0) {
    close(fd);
    throw std::runtime_error("CAN interface not found: " + interface);
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    close(fd);
    throw std::runtime_error("failed to bind CAN interface: " + interface);
  }
  return Socket{interface, fd};
}

void send_poll(Socket& socket, Motor& motor) {
  const auto data = zero_gain_payload(motor);
  canfd_frame frame{};
  frame.can_id = static_cast<canid_t>(motor.can_id);
  frame.len = 8;
  frame.flags = CANFD_BRS;
  std::copy(data.begin(), data.end(), frame.data);
  const ssize_t sent = write(socket.fd, &frame, CANFD_MTU);
  if (sent != CANFD_MTU) {
    throw std::runtime_error(
        "CAN-FD zero-gain write failed for " + motor.name + ": " + std::strerror(errno));
  }
  ++motor.tx_count;
}

void apply_affinity(int cpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
    throw std::runtime_error("failed to apply CPU affinity");
  }
}

double percentile(std::vector<double> values, double fraction) {
  std::sort(values.begin(), values.end());
  const auto index = std::min(values.size() - 1,
      static_cast<std::size_t>(std::ceil(fraction * values.size())) - 1);
  return values[index];
}

std::string json_report(const std::vector<Motor>& motors, const std::vector<double>& lateness,
                        double elapsed_s, std::size_t deadline_misses, bool memory_locked,
                        int cpu) {
  const double p99 = percentile(lateness, 0.99);
  const double maximum = *std::max_element(lateness.begin(), lateness.end());
  bool coverage_passed = true;
  std::ostringstream tx_counts;
  std::ostringstream rx_counts;
  std::ostringstream coverage_values;
  tx_counts << "  \"tx_count_by_motor\": {\n";
  rx_counts << "  \"rx_count_by_motor\": {\n";
  coverage_values << "  \"sample_coverage_by_motor\": {\n";
  for (std::size_t i = 0; i < motors.size(); ++i) {
    const auto& motor = motors[i];
    const double coverage = motor.tx_count > 0
        ? static_cast<double>(motor.rx_count) / static_cast<double>(motor.tx_count)
        : 0.0;
    coverage_passed = coverage_passed && motor.seen && coverage >= 0.95;
    const char* suffix = i + 1 == motors.size() ? "\n" : ",\n";
    tx_counts << "    \"" << motor.name << "\": " << motor.tx_count << suffix;
    rx_counts << "    \"" << motor.name << "\": " << motor.rx_count << suffix;
    coverage_values << "    \"" << motor.name << "\": " << coverage << suffix;
  }
  tx_counts << "  },\n";
  rx_counts << "  },\n";
  coverage_values << "  },\n";
  const bool passed = deadline_misses == 0 && p99 <= 0.5 && maximum <= 2.0 && coverage_passed;
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "{\n  \"mode\": \"native_mixed_rate_zero_gain_can_shadow\",\n"
      << "  \"elapsed_s\": " << elapsed_s << ",\n"
      << "  \"state_ticks\": " << lateness.size() << ",\n"
      << "  \"deadline_misses\": " << deadline_misses << ",\n"
      << "  \"lateness_p99_ms\": " << p99 << ",\n"
      << "  \"lateness_max_ms\": " << maximum << ",\n"
      << "  \"cpu\": " << cpu << ",\n"
      << "  \"memory_locked\": " << (memory_locked ? "true" : "false") << ",\n"
      << "  \"nonzero_gain_or_torque_tx_attempts\": 0,\n"
      << "  \"automatic_enable_attempts\": 0,\n"
      << "  \"automatic_mode_switch_attempts\": 0,\n"
      << tx_counts.str()
      << rx_counts.str()
      << coverage_values.str()
      << "  \"coverage_passed\": " << (coverage_passed ? "true" : "false") << ",\n"
      << "  \"passed\": " << (passed ? "true" : "false") << "\n}\n";
  return out.str();
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<Socket> sockets;
  try {
    const Options options = parse_options(argc, argv);
    auto motors = load_motors(options.config);
    std::map<std::string, std::size_t> socket_index;
    for (const auto& motor : motors) {
      if (socket_index.count(motor.interface) == 0) {
        socket_index[motor.interface] = sockets.size();
        sockets.push_back(open_socket(motor.interface));
      }
    }
    if (sockets.size() != 4) throw std::runtime_error("exactly four CAN interfaces required");
    std::map<std::pair<std::string, int>, std::size_t> feedback_map;
    for (std::size_t i = 0; i < motors.size(); ++i) {
      if (!feedback_map.emplace(std::make_pair(motors[i].interface, motors[i].master_id), i).second) {
        throw std::runtime_error("duplicate feedback endpoint");
      }
    }
    const std::size_t transport_slots =
        static_cast<std::size_t>(std::llround(options.duration_s * kTransportRateHz));
    const std::size_t state_ticks = transport_slots / 4;
    std::vector<double> lateness(state_ticks);
    apply_affinity(options.cpu);
    const bool memory_locked = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;
    const std::int64_t start_ns = monotonic_ns() + 100'000'000LL;
    std::size_t deadline_misses = 0;
    std::size_t state_tick = 0;
    for (std::size_t slot = 0; slot < transport_slots; ++slot) {
      const std::int64_t deadline_ns =
          start_ns + static_cast<std::int64_t>(slot) * kTransportPeriodNs;
      const timespec deadline = from_ns(deadline_ns);
      int status = 0;
      do status = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
      while (status == EINTR);
      if (status != 0) throw std::runtime_error("clock_nanosleep failed");
      const std::int64_t woke_ns = monotonic_ns();
      if (slot % 4 == 0) {
        lateness[state_tick++] =
            std::max(0.0, static_cast<double>(woke_ns - deadline_ns) / 1.0e6);
        deadline_misses += static_cast<std::size_t>(woke_ns - deadline_ns >= 2'000'000LL);
      }
      for (auto& motor : motors) {
        const bool due = motor.poll_rate_hz == 500
            ? static_cast<int>(slot % 4) == motor.poll_phase
            : static_cast<int>(slot % 40) == motor.poll_phase;
        if (due) {
          send_poll(sockets[socket_index.at(motor.interface)], motor);
        }
      }
      for (auto& can_socket : sockets) {
        while (true) {
          canfd_frame frame{};
          const ssize_t received = read(can_socket.fd, &frame, CANFD_MTU);
          if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
          if (received != CANFD_MTU || frame.len != 8 || (frame.flags & CANFD_BRS) == 0) {
            throw std::runtime_error("invalid CAN-FD feedback frame");
          }
          const int can_id = static_cast<int>(frame.can_id & CAN_SFF_MASK);
          const auto found = feedback_map.find({can_socket.interface, can_id});
          if (found == feedback_map.end()) continue;
          auto& motor = motors[found->second];
          const int controller_id = frame.data[0] & 0x0F;
          const int status_code = (frame.data[0] >> 4) & 0x0F;
          if (controller_id != motor.can_id || status_code != 0) {
            throw std::runtime_error("motor identity/status invariant failed for " + motor.name);
          }
          const auto raw_position = (static_cast<unsigned>(frame.data[1]) << 8) | frame.data[2];
          motor.last_position = decode_uint(raw_position, motor.position_min, motor.position_max, 16);
          ++motor.rx_count;
          motor.seen = true;
        }
      }
    }
    // Account for responses to the final scheduled slot before reporting coverage.
    usleep(2000);
    for (auto& can_socket : sockets) {
      while (true) {
        canfd_frame frame{};
        const ssize_t received = read(can_socket.fd, &frame, CANFD_MTU);
        if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        if (received != CANFD_MTU || frame.len != 8 || (frame.flags & CANFD_BRS) == 0) {
          throw std::runtime_error("invalid final CAN-FD feedback frame");
        }
        const int can_id = static_cast<int>(frame.can_id & CAN_SFF_MASK);
        const auto found = feedback_map.find({can_socket.interface, can_id});
        if (found == feedback_map.end()) continue;
        auto& motor = motors[found->second];
        const int controller_id = frame.data[0] & 0x0F;
        const int status_code = (frame.data[0] >> 4) & 0x0F;
        if (controller_id != motor.can_id || status_code != 0) {
          throw std::runtime_error("final motor identity/status invariant failed for " + motor.name);
        }
        const auto raw_position = (static_cast<unsigned>(frame.data[1]) << 8) | frame.data[2];
        motor.last_position = decode_uint(raw_position, motor.position_min, motor.position_max, 16);
        ++motor.rx_count;
        motor.seen = true;
      }
    }
    const double elapsed_s = static_cast<double>(monotonic_ns() - start_ns) / 1.0e9;
    const std::string report = json_report(
        motors, lateness, elapsed_s, deadline_misses, memory_locked, options.cpu);
    std::cout << report;
    std::ofstream output(options.output);
    if (!output) throw std::runtime_error("cannot open report output");
    output << report;
    for (auto& can_socket : sockets) close(can_socket.fd);
    return report.find("\"passed\": true") != std::string::npos ? 0 : 2;
  } catch (const std::exception& error) {
    for (auto& can_socket : sockets) if (can_socket.fd >= 0) close(can_socket.fd);
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}
