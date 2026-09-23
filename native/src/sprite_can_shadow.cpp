#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
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
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/un.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace {

constexpr std::int64_t kTransportPeriodNs = 500'000LL;
constexpr int kTransportRateHz = 2000;
constexpr std::uint16_t kIpcVersion = 1;
constexpr std::uint16_t kStateKind = 1;
constexpr std::uint16_t kTargetKind = 2;
constexpr std::size_t kJointCount = 31;
constexpr std::int64_t kTargetTimeoutNs = 100'000'000LL;
volatile std::sig_atomic_t g_shutdown_requested = 0;

void request_shutdown(int) {
  g_shutdown_requested = 1;
}

#pragma pack(push, 1)
struct StatePacket {
  char magic[4];
  std::uint16_t version;
  std::uint16_t kind;
  std::uint64_t sequence;
  std::int64_t monotonic_ns;
  std::uint64_t motor_order_hash;
  double position_rad[kJointCount];
  double velocity_rad_s[kJointCount];
};

struct TargetPacket {
  char magic[4];
  std::uint16_t version;
  std::uint16_t kind;
  std::uint64_t sequence;
  std::int64_t monotonic_ns;
  std::uint64_t joint_order_hash;
  std::uint64_t source_state_sequence;
  double position_rad[kJointCount];
  double velocity_rad_s[kJointCount];
  double kp[kJointCount];
  double kd[kJointCount];
  double feedforward_torque_nm[kJointCount];
};
#pragma pack(pop)

static_assert(sizeof(StatePacket) == 528, "state IPC ABI drift");
static_assert(sizeof(TargetPacket) == 1280, "target IPC ABI drift");

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
  double soft_position_min = 0.0;
  double soft_position_max = 0.0;
  double hard_position_min = 0.0;
  double hard_position_max = 0.0;
  double deployment_velocity_max = 0.0;
  double rated_torque = 0.0;
  double mechanical_peak_torque = 0.0;
  double commissioning_torque_cap = 0.0;
  double feedback_torque_cap = 0.0;
  double mos_temperature_limit = 0.0;
  double rotor_temperature_limit = 0.0;
  double filtered_mos_temperature_limit = 0.0;
  double filtered_rotor_temperature_limit = 0.0;
  double temperature_filter_time_constant_s = 0.0;
  int poll_rate_hz = 0;
  int poll_phase = 0;
  double last_position = 0.0;
  double last_velocity = 0.0;
  int maximum_mos_temperature = 0;
  int maximum_rotor_temperature = 0;
  double filtered_mos_temperature = 0.0;
  double filtered_rotor_temperature = 0.0;
  double maximum_filtered_mos_temperature = 0.0;
  double maximum_filtered_rotor_temperature = 0.0;
  bool temperature_filter_initialized = false;
  std::size_t tx_count = 0;
  std::size_t rx_count = 0;
  bool seen = false;
  double preview_maximum_abs_position = 0.0;
  double preview_maximum_abs_velocity = 0.0;
  double preview_maximum_kp = 0.0;
  double preview_maximum_kd = 0.0;
  double preview_maximum_abs_feedforward_torque = 0.0;
  double preview_maximum_abs_estimated_torque = 0.0;
  double preview_maximum_abs_unsaturated_torque = 0.0;
  std::size_t policy_torque_saturation_count = 0;
  double preview_position = 0.0;
  double preview_velocity = 0.0;
  double preview_kp = 0.0;
  double preview_kd = 0.0;
  double preview_feedforward_torque = 0.0;
  bool preview_initialized = false;
  double captured_position = 0.0;
  double maximum_abs_position_drift = 0.0;
  double maximum_abs_speed = 0.0;
  double maximum_abs_commanded_torque = 0.0;
  double maximum_abs_feedback_torque = 0.0;
  std::deque<double> short_torque_squared_samples;
  std::deque<double> long_torque_squared_samples;
  double short_torque_squared_sum = 0.0;
  double long_torque_squared_sum = 0.0;
  double maximum_short_torque_rms = 0.0;
  double maximum_long_torque_rms = 0.0;
  bool final_disabled = false;
  int last_status_code = -1;
};

struct NativeKinematics {
  std::array<std::string, kJointCount> joint_names{};
  std::array<double, kJointCount> motor_offset{};
  std::array<double, kJointCount> joint_effort_limit{};
  std::array<std::array<double, kJointCount>, kJointCount> joint_to_motor{};
  std::array<std::array<double, kJointCount>, kJointCount> motor_to_joint{};
};

struct JointSafety {
  std::string name;
  double position_min = 0.0;
  double position_max = 0.0;
  double velocity_max = 0.0;
  double kp_max = 0.0;
  double kd_max = 0.0;
  double feedforward_torque_max = 0.0;
};

struct Options {
  std::string config;
  std::string output;
  double duration_s = 10.0;
  int cpu = 5;
  std::string acknowledgement;
  std::string extended_policy_actuation_acknowledgement;
  bool all_disabled = false;
  std::string ipc_socket;
  std::string joint_safety_config;
  std::string kinematics_config;
  std::uint64_t policy_joint_hash = 0;
  int realtime_priority = 0;
  bool measured_pose_hold = false;
  bool policy_actuation = false;
  bool saturate_policy_torque = false;
};

struct Socket {
  std::string interface;
  int fd = -1;
};

struct IpcSocket {
  std::string path;
  int server_fd = -1;
  int client_fd = -1;
  std::uint64_t state_sequence = 0;
  std::uint64_t last_target_sequence = 0;
  std::int64_t last_target_ns = 0;
  std::size_t target_count = 0;
  double maximum_target_age_ms = 0.0;
  TargetPacket latest_target{};
  bool has_target = false;
  std::size_t motor_preview_count = 0;
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

std::uint64_t unsigned_number(const std::string& value) {
  std::size_t consumed = 0;
  const auto result = std::stoull(value, &consumed, 0);
  if (consumed != value.size()) {
    throw std::runtime_error("invalid unsigned integer argument");
  }
  return result;
}

std::uint64_t ordered_name_hash(const std::vector<Motor>& motors) {
  std::uint64_t result = 0xCBF29CE484222325ULL;
  for (const auto& motor : motors) {
    for (const unsigned char byte : motor.name) {
      result ^= byte;
      result *= 0x100000001B3ULL;
    }
    result ^= 0;
    result *= 0x100000001B3ULL;
  }
  return result;
}

std::uint64_t ordered_joint_name_hash(const std::vector<JointSafety>& limits) {
  std::uint64_t result = 0xCBF29CE484222325ULL;
  for (const auto& limit : limits) {
    for (const unsigned char byte : limit.name) {
      result ^= byte;
      result *= 0x100000001B3ULL;
    }
    result ^= 0;
    result *= 0x100000001B3ULL;
  }
  return result;
}

std::uint64_t ordered_joint_name_hash(
    const std::array<std::string, kJointCount>& names) {
  std::uint64_t result = 0xCBF29CE484222325ULL;
  for (const auto& name : names) {
    for (const unsigned char byte : name) {
      result ^= byte;
      result *= 0x100000001B3ULL;
    }
    result ^= 0;
    result *= 0x100000001B3ULL;
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
      "torque_min_nm", "torque_max_nm", "soft_position_min_rad",
      "soft_position_max_rad", "hard_position_min_rad", "hard_position_max_rad",
      "deployment_velocity_max_rad_s", "rated_torque_nm", "mechanical_peak_torque_nm",
      "commissioning_torque_cap_nm", "feedback_torque_cap_nm",
      "mos_temperature_limit_c", "rotor_temperature_limit_c",
      "filtered_mos_temperature_limit_c", "filtered_rotor_temperature_limit_c",
      "temperature_filter_time_constant_s", "poll_rate_hz"};
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
    motor.soft_position_min = number(fields[10]);
    motor.soft_position_max = number(fields[11]);
    motor.hard_position_min = number(fields[12]);
    motor.hard_position_max = number(fields[13]);
    motor.deployment_velocity_max = number(fields[14]);
    motor.rated_torque = number(fields[15]);
    motor.mechanical_peak_torque = number(fields[16]);
    motor.commissioning_torque_cap = number(fields[17]);
    motor.feedback_torque_cap = number(fields[18]);
    motor.mos_temperature_limit = number(fields[19]);
    motor.rotor_temperature_limit = number(fields[20]);
    motor.filtered_mos_temperature_limit = number(fields[21]);
    motor.filtered_rotor_temperature_limit = number(fields[22]);
    motor.temperature_filter_time_constant_s = number(fields[23]);
    motor.poll_rate_hz = static_cast<int>(number(fields[24]));
    if (motor.can_id < 1 || motor.can_id > 8 || motor.master_id != motor.can_id + 0x10 ||
        motor.position_min >= motor.soft_position_min ||
        motor.soft_position_min >= motor.soft_position_max ||
        motor.soft_position_max >= motor.position_max ||
        motor.hard_position_min >= motor.soft_position_min ||
        motor.hard_position_max <= motor.soft_position_max ||
        motor.deployment_velocity_max <= 0.0 ||
        motor.deployment_velocity_max > motor.velocity_max ||
        motor.rated_torque <= 0.0 ||
        motor.rated_torque > motor.mechanical_peak_torque ||
        motor.mechanical_peak_torque <= 0.0 ||
        motor.commissioning_torque_cap <= 0.0 ||
        motor.commissioning_torque_cap > motor.mechanical_peak_torque ||
        motor.feedback_torque_cap <= 0.0 ||
        motor.feedback_torque_cap > motor.mechanical_peak_torque ||
        motor.mos_temperature_limit <= 0.0 || motor.rotor_temperature_limit <= 0.0 ||
        motor.filtered_mos_temperature_limit <= 0.0 ||
        motor.filtered_mos_temperature_limit >= motor.mos_temperature_limit ||
        motor.filtered_rotor_temperature_limit <= 0.0 ||
        motor.filtered_rotor_temperature_limit >= motor.rotor_temperature_limit ||
        motor.temperature_filter_time_constant_s <= 0.0 ||
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

NativeKinematics load_kinematics(const std::string& path,
                                 const std::vector<Motor>& motors) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open native kinematics config: " + path);
  std::string line;
  if (!std::getline(file, line)) throw std::runtime_error("native kinematics config is empty");
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::vector<std::string> expected{"kind", "name", "offset_or_effort"};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    expected.push_back("c" + std::to_string(index));
  }
  if (split(line, '\t') != expected) {
    throw std::runtime_error("native kinematics config header mismatch");
  }
  NativeKinematics result;
  std::size_t row = 0;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    const auto fields = split(line, '\t');
    if (fields.size() != expected.size() || row >= 2 * kJointCount) {
      throw std::runtime_error("native kinematics config row mismatch");
    }
    const bool motor_row = row < kJointCount;
    const std::size_t index = motor_row ? row : row - kJointCount;
    if (fields[0] != (motor_row ? "motor" : "joint")) {
      throw std::runtime_error("native kinematics row kind/order mismatch");
    }
    if (motor_row) {
      if (fields[1] != motors[index].name) {
        throw std::runtime_error("native kinematics motor order mismatch");
      }
      result.motor_offset[index] = number(fields[2]);
    } else {
      result.joint_names[index] = fields[1];
      result.joint_effort_limit[index] = number(fields[2]);
      if (result.joint_effort_limit[index] <= 0.0) {
        throw std::runtime_error("native kinematics joint effort must be positive");
      }
    }
    for (std::size_t column = 0; column < kJointCount; ++column) {
      (motor_row ? result.joint_to_motor[index][column]
                 : result.motor_to_joint[index][column]) = number(fields[3 + column]);
    }
    ++row;
  }
  if (row != 2 * kJointCount) {
    throw std::runtime_error("native kinematics config must contain 62 rows");
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    for (std::size_t j = 0; j < kJointCount; ++j) {
      double product = 0.0;
      for (std::size_t k = 0; k < kJointCount; ++k) {
        product += result.motor_to_joint[i][k] * result.joint_to_motor[k][j];
      }
      const double expected_value = i == j ? 1.0 : 0.0;
      if (std::abs(product - expected_value) > 1.0e-9) {
        throw std::runtime_error("native kinematics inverse invariant failed");
      }
    }
  }
  return result;
}

std::vector<JointSafety> load_joint_safety(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open native joint safety config: " + path);
  }
  std::string line;
  if (!std::getline(file, line)) {
    throw std::runtime_error("native joint safety config is empty");
  }
  if (!line.empty() && line.back() == '\r') line.pop_back();
  const std::vector<std::string> expected_header = {
      "joint_name", "position_min_rad", "position_max_rad", "velocity_max_rad_s",
      "kp_max", "kd_max", "feedforward_torque_max_nm"};
  if (split(line, '\t') != expected_header) {
    throw std::runtime_error("native joint safety config header mismatch");
  }
  std::vector<JointSafety> result;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    const auto fields = split(line, '\t');
    if (fields.size() != expected_header.size()) {
      throw std::runtime_error("native joint safety row has wrong field count");
    }
    JointSafety limit;
    limit.name = fields[0];
    limit.position_min = number(fields[1]);
    limit.position_max = number(fields[2]);
    limit.velocity_max = number(fields[3]);
    limit.kp_max = number(fields[4]);
    limit.kd_max = number(fields[5]);
    limit.feedforward_torque_max = number(fields[6]);
    if (limit.name.empty() || limit.position_min >= limit.position_max ||
        limit.velocity_max <= 0.0 || limit.kp_max < 0.0 || limit.kd_max < 0.0 ||
        limit.kd_max > 3.0 || limit.feedforward_torque_max < 0.0) {
      throw std::runtime_error("native joint safety limit invariant failed");
    }
    result.push_back(limit);
  }
  if (result.size() != kJointCount) {
    throw std::runtime_error("native joint safety config must contain 31 joints");
  }
  std::vector<std::string> names;
  names.reserve(result.size());
  for (const auto& limit : result) names.push_back(limit.name);
  std::sort(names.begin(), names.end());
  if (std::adjacent_find(names.begin(), names.end()) != names.end()) {
    throw std::runtime_error("native joint safety names must be unique");
  }
  return result;
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
    else if (argument == "--extended-policy-actuation-acknowledgement") {
      result.extended_policy_actuation_acknowledgement = value();
    }
    else if (argument == "--all-motors-disabled-confirmed") result.all_disabled = true;
    else if (argument == "--ipc-socket") result.ipc_socket = value();
    else if (argument == "--joint-safety-config") result.joint_safety_config = value();
    else if (argument == "--kinematics-config") result.kinematics_config = value();
    else if (argument == "--policy-joint-hash") result.policy_joint_hash = unsigned_number(value());
    else if (argument == "--realtime-priority") {
      result.realtime_priority = static_cast<int>(number(value()));
    }
    else if (argument == "--measured-pose-hold") result.measured_pose_hold = true;
    else if (argument == "--policy-actuation") result.policy_actuation = true;
    else if (argument == "--saturate-policy-torque-to-commissioning-cap") {
      result.saturate_policy_torque = true;
    }
    else throw std::runtime_error("unknown argument: " + argument);
  }
  if (result.config.empty() || result.output.empty()) {
    throw std::runtime_error("--config and --output are required");
  }
  if (!(result.duration_s > 0.0 && result.duration_s <= 120.0)) {
    throw std::runtime_error("duration must be in (0, 120]");
  }
  if (result.measured_pose_hold && result.policy_actuation) {
    throw std::runtime_error("measured-pose hold and policy actuation are mutually exclusive");
  }
  const std::string required_ack = result.measured_pose_hold
      ? "ENABLE_NATIVE_FULL_BODY_MEASURED_POSE_HOLD"
      : result.policy_actuation ? "ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION"
                                : "ZERO_GAIN_NATIVE_SHADOW";
  if (result.acknowledgement != required_ack || !result.all_disabled) {
    throw std::runtime_error("explicit hardware TX acknowledgement and disabled confirmation required");
  }
  if (result.ipc_socket.empty() != (result.policy_joint_hash == 0)) {
    throw std::runtime_error("--ipc-socket and nonzero --policy-joint-hash must be provided together");
  }
  if (!result.joint_safety_config.empty() && result.ipc_socket.empty()) {
    throw std::runtime_error("--joint-safety-config requires policy IPC");
  }
  if (!result.measured_pose_hold &&
      result.kinematics_config.empty() != result.ipc_socket.empty()) {
    throw std::runtime_error("--kinematics-config is required exactly when policy IPC is enabled");
  }
  if (result.measured_pose_hold) {
    if (!result.ipc_socket.empty() || result.kinematics_config.empty()) {
      throw std::runtime_error("measured-pose hold requires kinematics and forbids policy IPC");
    }
    if (result.duration_s > 2.0) {
      throw std::runtime_error("first measured-pose hold duration must not exceed 2 seconds");
    }
  }
  if (result.policy_actuation) {
    if (result.ipc_socket.empty() || result.kinematics_config.empty() ||
        result.joint_safety_config.empty() || result.policy_joint_hash == 0) {
      throw std::runtime_error(
          "policy actuation requires IPC, kinematics, joint safety, and joint hash");
    }
    const bool acknowledged_20_seconds =
        result.extended_policy_actuation_acknowledgement ==
        "ENABLE_20_SECOND_SUSPENDED_BALANCE_TEST";
    const bool acknowledged_40_seconds =
        result.extended_policy_actuation_acknowledgement ==
        "ENABLE_40_SECOND_GROUNDED_BALANCE_TEST";
    if (result.duration_s > 10.0 &&
        !acknowledged_20_seconds && !acknowledged_40_seconds) {
      throw std::runtime_error(
          "protected policy actuation over 10 seconds requires the exact extended-test acknowledgement");
    }
    if (result.duration_s > 20.0 && !acknowledged_40_seconds) {
      throw std::runtime_error(
          "protected policy actuation over 20 seconds requires the exact 40-second acknowledgement");
    }
    if (result.duration_s > 40.0) {
      throw std::runtime_error("extended protected policy actuation must not exceed 40 seconds");
    }
  }
  if (result.saturate_policy_torque &&
      (!result.policy_actuation ||
       result.extended_policy_actuation_acknowledgement !=
           "ENABLE_40_SECOND_GROUNDED_BALANCE_TEST")) {
    throw std::runtime_error(
        "policy torque saturation requires the exact 40-second grounded-test acknowledgement");
  }
  if (result.realtime_priority < 0 || result.realtime_priority > 80) {
    throw std::runtime_error("realtime priority must be in [0, 80]");
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

IpcSocket open_ipc_server(const std::string& path) {
  if (path.empty() || path.size() >= sizeof(sockaddr_un::sun_path)) {
    throw std::runtime_error("invalid IPC socket path");
  }
  IpcSocket result;
  result.path = path;
  result.server_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
  if (result.server_fd < 0) throw std::runtime_error("failed to open IPC server socket");
  unlink(path.c_str());
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);
  if (bind(result.server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
      listen(result.server_fd, 1) != 0) {
    close(result.server_fd);
    unlink(path.c_str());
    throw std::runtime_error("failed to bind/listen on IPC socket");
  }
  pollfd event{result.server_fd, POLLIN, 0};
  const int ready = poll(&event, 1, 10'000);
  if (ready <= 0 || (event.revents & POLLIN) == 0) {
    close(result.server_fd);
    unlink(path.c_str());
    throw std::runtime_error("policy IPC client did not connect within 10 seconds");
  }
  result.client_fd = accept4(result.server_fd, nullptr, nullptr, SOCK_NONBLOCK);
  if (result.client_fd < 0) {
    close(result.server_fd);
    unlink(path.c_str());
    throw std::runtime_error("failed to accept policy IPC client");
  }
  return result;
}

void close_ipc(IpcSocket& ipc) {
  if (ipc.client_fd >= 0) close(ipc.client_fd);
  if (ipc.server_fd >= 0) close(ipc.server_fd);
  if (!ipc.path.empty()) unlink(ipc.path.c_str());
  ipc.client_fd = -1;
  ipc.server_fd = -1;
}

void send_state_packet(IpcSocket& ipc, const std::vector<Motor>& motors,
                       std::uint64_t motor_hash, std::int64_t now_ns) {
  StatePacket packet{};
  std::memcpy(packet.magic, "SPRT", 4);
  packet.version = kIpcVersion;
  packet.kind = kStateKind;
  packet.sequence = ++ipc.state_sequence;
  packet.monotonic_ns = now_ns;
  packet.motor_order_hash = motor_hash;
  for (std::size_t index = 0; index < motors.size(); ++index) {
    packet.position_rad[index] = motors[index].last_position;
    packet.velocity_rad_s[index] = motors[index].last_velocity;
  }
  const ssize_t sent = send(ipc.client_fd, &packet, sizeof(packet), MSG_DONTWAIT | MSG_NOSIGNAL);
  if (sent != static_cast<ssize_t>(sizeof(packet))) {
    throw std::runtime_error("failed to publish native state IPC packet");
  }
}

void drain_target_packets(IpcSocket& ipc, std::uint64_t policy_joint_hash,
                          const std::vector<JointSafety>& joint_safety,
                          std::int64_t now_ns) {
  while (true) {
    TargetPacket packet{};
    const ssize_t received = recv(ipc.client_fd, &packet, sizeof(packet), MSG_DONTWAIT);
    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
    if (received == 0) throw std::runtime_error("policy IPC client disconnected");
    if (received != static_cast<ssize_t>(sizeof(packet)) ||
        std::memcmp(packet.magic, "SPRT", 4) != 0 || packet.version != kIpcVersion ||
        packet.kind != kTargetKind || packet.joint_order_hash != policy_joint_hash) {
      throw std::runtime_error("policy IPC target ABI/order invariant failed");
    }
    if (packet.sequence <= ipc.last_target_sequence ||
        packet.source_state_sequence > ipc.state_sequence ||
        ipc.state_sequence - packet.source_state_sequence > 5) {
      throw std::runtime_error("policy IPC target sequence invariant failed");
    }
    const std::int64_t age_ns = now_ns - packet.monotonic_ns;
    if (age_ns < -5'000'000LL || age_ns > kTargetTimeoutNs) {
      throw std::runtime_error("policy IPC target timestamp is future or stale");
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const std::array<double, 5> values{packet.position_rad[index], packet.velocity_rad_s[index],
          packet.kp[index], packet.kd[index], packet.feedforward_torque_nm[index]};
      if (!std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); }) ||
          packet.kp[index] < 0.0 || packet.kd[index] < 0.0 || packet.kd[index] > 3.0) {
        throw std::runtime_error("policy IPC target numeric/gain invariant failed");
      }
      if (!joint_safety.empty()) {
        constexpr double tolerance = 1.0e-9;
        const auto& limit = joint_safety[index];
        if (packet.position_rad[index] < limit.position_min - tolerance ||
            packet.position_rad[index] > limit.position_max + tolerance ||
            std::abs(packet.velocity_rad_s[index]) > limit.velocity_max + tolerance ||
            packet.kp[index] > limit.kp_max + tolerance ||
            packet.kd[index] > limit.kd_max + tolerance ||
            std::abs(packet.feedforward_torque_nm[index]) >
                limit.feedforward_torque_max + tolerance) {
          throw std::runtime_error(
              "policy IPC protected target envelope failed at joint " + limit.name);
        }
      }
    }
    ipc.last_target_sequence = packet.sequence;
    ipc.last_target_ns = packet.monotonic_ns;
    ipc.latest_target = packet;
    ipc.has_target = true;
    ++ipc.target_count;
    ipc.maximum_target_age_ms = std::max(
        ipc.maximum_target_age_ms, static_cast<double>(age_ns) / 1.0e6);
  }
}

void preview_final_motor_commands(std::vector<Motor>& motors, IpcSocket& ipc,
                                  const NativeKinematics& kinematics,
                                  bool saturate_to_commissioning_cap = false) {
  if (!ipc.has_target) return;
  std::array<double, kJointCount> joint_position{};
  std::array<double, kJointCount> joint_velocity{};
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    for (std::size_t motor = 0; motor < kJointCount; ++motor) {
      joint_position[joint] += kinematics.motor_to_joint[joint][motor] *
          (motors[motor].last_position - kinematics.motor_offset[motor]);
      joint_velocity[joint] +=
          kinematics.motor_to_joint[joint][motor] * motors[motor].last_velocity;
    }
  }
  std::array<double, kJointCount> desired_motor_position{};
  std::array<double, kJointCount> desired_motor_velocity{};
  std::array<double, kJointCount> joint_torque{};
  for (std::size_t motor = 0; motor < kJointCount; ++motor) {
    desired_motor_position[motor] = kinematics.motor_offset[motor];
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      desired_motor_position[motor] += kinematics.joint_to_motor[motor][joint] *
          ipc.latest_target.position_rad[joint];
      desired_motor_velocity[motor] += kinematics.joint_to_motor[motor][joint] *
          ipc.latest_target.velocity_rad_s[joint];
    }
  }
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    const double raw = ipc.latest_target.kp[joint] *
            (ipc.latest_target.position_rad[joint] - joint_position[joint]) +
        ipc.latest_target.kd[joint] *
            (ipc.latest_target.velocity_rad_s[joint] - joint_velocity[joint]) +
        ipc.latest_target.feedforward_torque_nm[joint];
    joint_torque[joint] = std::clamp(
        raw, -kinematics.joint_effort_limit[joint], kinematics.joint_effort_limit[joint]);
  }
  const bool update_other_motors = ipc.motor_preview_count % 10 == 0;
  for (std::size_t motor_index = 0; motor_index < kJointCount; ++motor_index) {
    auto& motor = motors[motor_index];
    const bool ankle = motor.name.find("ankle_motor") != std::string::npos;
    if (ankle) {
      motor.preview_position = motor.last_position;
      motor.preview_velocity = 0.0;
      motor.preview_kp = 0.0;
      motor.preview_kd = 0.0;
      motor.preview_feedforward_torque = 0.0;
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        if (kinematics.joint_names[joint].find("ankle_") != std::string::npos) {
          motor.preview_feedforward_torque +=
              kinematics.motor_to_joint[joint][motor_index] * joint_torque[joint];
        }
      }
      motor.preview_initialized = true;
    } else if (update_other_motors) {
      motor.preview_position = desired_motor_position[motor_index];
      motor.preview_velocity = desired_motor_velocity[motor_index];
      motor.preview_kp = 0.0;
      motor.preview_kd = 0.0;
      motor.preview_feedforward_torque = 0.0;
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        const double coefficient = kinematics.motor_to_joint[joint][motor_index];
        motor.preview_kp += ipc.latest_target.kp[joint] * coefficient * coefficient;
        motor.preview_kd += ipc.latest_target.kd[joint] * coefficient * coefficient;
        motor.preview_feedforward_torque +=
            coefficient * ipc.latest_target.feedforward_torque_nm[joint];
      }
      for (std::size_t other = 0; other < kJointCount; ++other) {
        if (other == motor_index) continue;
        double coupled_kp = 0.0;
        double coupled_kd = 0.0;
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
          const double left = kinematics.motor_to_joint[joint][motor_index];
          const double right = kinematics.motor_to_joint[joint][other];
          coupled_kp += ipc.latest_target.kp[joint] * left * right;
          coupled_kd += ipc.latest_target.kd[joint] * left * right;
        }
        motor.preview_feedforward_torque +=
            coupled_kp * (desired_motor_position[other] - motors[other].last_position) +
            coupled_kd * (desired_motor_velocity[other] - motors[other].last_velocity);
      }
      motor.preview_initialized = true;
    }
    if (!motor.preview_initialized) continue;
    const double estimated_torque = motor.preview_kp *
            (motor.preview_position - motor.last_position) +
        motor.preview_kd * (motor.preview_velocity - motor.last_velocity) +
        motor.preview_feedforward_torque;
    motor.preview_maximum_abs_unsaturated_torque = std::max(
        motor.preview_maximum_abs_unsaturated_torque, std::abs(estimated_torque));
    if (saturate_to_commissioning_cap &&
        std::abs(estimated_torque) > 0.98 * motor.commissioning_torque_cap) {
      const double limited_torque =
          std::copysign(0.98 * motor.commissioning_torque_cap, estimated_torque);
      if (motor.preview_kp > 1.0e-9) {
        motor.preview_position +=
            (limited_torque - estimated_torque) / motor.preview_kp;
      } else {
        motor.preview_feedforward_torque += limited_torque - estimated_torque;
      }
      ++motor.policy_torque_saturation_count;
    }
    const double protected_estimated_torque = motor.preview_kp *
            (motor.preview_position - motor.last_position) +
        motor.preview_kd * (motor.preview_velocity - motor.last_velocity) +
        motor.preview_feedforward_torque;
    constexpr double tolerance = 1.0e-9;
    if (motor.preview_position < motor.soft_position_min - tolerance ||
        motor.preview_position > motor.soft_position_max + tolerance ||
        motor.preview_position < motor.hard_position_min - tolerance ||
        motor.preview_position > motor.hard_position_max + tolerance ||
        motor.preview_position < motor.position_min - tolerance ||
        motor.preview_position > motor.position_max + tolerance ||
        std::abs(motor.preview_velocity) > motor.deployment_velocity_max + tolerance ||
        motor.preview_kp < 0.0 || motor.preview_kp > 500.0 + tolerance ||
        motor.preview_kd < 0.0 || motor.preview_kd > 3.0 + tolerance ||
        std::abs(motor.preview_feedforward_torque) >
            std::min(std::abs(motor.torque_min), motor.torque_max) + tolerance ||
        std::abs(protected_estimated_torque) >
            std::min(std::abs(motor.torque_min), motor.torque_max) + tolerance ||
        std::abs(protected_estimated_torque) > motor.mechanical_peak_torque + tolerance) {
      throw std::runtime_error("final motor command preview invariant failed for " + motor.name);
    }
    motor.preview_maximum_abs_position =
        std::max(motor.preview_maximum_abs_position, std::abs(motor.preview_position));
    motor.preview_maximum_abs_velocity =
        std::max(motor.preview_maximum_abs_velocity, std::abs(motor.preview_velocity));
    motor.preview_maximum_kp = std::max(motor.preview_maximum_kp, motor.preview_kp);
    motor.preview_maximum_kd = std::max(motor.preview_maximum_kd, motor.preview_kd);
    motor.preview_maximum_abs_feedforward_torque = std::max(
        motor.preview_maximum_abs_feedforward_torque,
        std::abs(motor.preview_feedforward_torque));
    motor.preview_maximum_abs_estimated_torque = std::max(
        motor.preview_maximum_abs_estimated_torque,
        std::abs(protected_estimated_torque));
  }
  ++ipc.motor_preview_count;
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

std::array<std::uint8_t, 8> hold_payload(
    Motor& motor, double position, double kp, double kd, double feedforward,
    double maximum_output_torque) {
  if (position < motor.soft_position_min || position > motor.soft_position_max ||
      std::abs(feedforward) > maximum_output_torque || kp < 0.0 || kp > 0.2 ||
      kd < 0.0 || kd > 0.05) {
    throw std::runtime_error("measured-hold command envelope failed for " + motor.name);
  }
  const double estimated = kp * (position - motor.last_position) -
      kd * motor.last_velocity + feedforward;
  if (!std::isfinite(estimated) || std::abs(estimated) > maximum_output_torque) {
    throw std::runtime_error("measured-hold torque guard failed for " + motor.name);
  }
  motor.maximum_abs_commanded_torque =
      std::max(motor.maximum_abs_commanded_torque, std::abs(estimated));
  const auto p = encode_uint(position, motor.position_min, motor.position_max, 16);
  const auto v = encode_uint(0.0, motor.velocity_min, motor.velocity_max, 12);
  const auto kp_raw = encode_uint(kp, 0.0, 500.0, 12);
  const auto kd_raw = encode_uint(kd, 0.0, 5.0, 12);
  const auto torque = encode_uint(feedforward, motor.torque_min, motor.torque_max, 12);
  return {
      static_cast<std::uint8_t>(p >> 8), static_cast<std::uint8_t>(p),
      static_cast<std::uint8_t>(v >> 4),
      static_cast<std::uint8_t>((v & 0xF) << 4 | kp_raw >> 8),
      static_cast<std::uint8_t>(kp_raw), static_cast<std::uint8_t>(kd_raw >> 4),
      static_cast<std::uint8_t>((kd_raw & 0xF) << 4 | torque >> 8),
      static_cast<std::uint8_t>(torque)};
}

std::array<std::uint8_t, 8> protected_policy_payload(Motor& motor) {
  constexpr double tolerance = 1.0e-9;
  const double position = motor.preview_position;
  const double velocity = motor.preview_velocity;
  const double kp = motor.preview_kp;
  const double kd = motor.preview_kd;
  const double feedforward = motor.preview_feedforward_torque;
  if (!motor.preview_initialized || !std::isfinite(position) ||
      !std::isfinite(velocity) || !std::isfinite(kp) || !std::isfinite(kd) ||
      !std::isfinite(feedforward) ||
      position < motor.soft_position_min - tolerance ||
      position > motor.soft_position_max + tolerance ||
      position < motor.hard_position_min - tolerance ||
      position > motor.hard_position_max + tolerance ||
      position < motor.position_min - tolerance ||
      position > motor.position_max + tolerance ||
      std::abs(velocity) > motor.deployment_velocity_max + tolerance ||
      kp < 0.0 || kp > 500.0 || kd < 0.0 || kd > 3.0 ||
      std::abs(feedforward) > motor.commissioning_torque_cap + tolerance) {
    throw std::runtime_error("protected policy command envelope failed for " + motor.name);
  }
  const auto p = encode_uint(position, motor.position_min, motor.position_max, 16);
  const auto v = encode_uint(velocity, motor.velocity_min, motor.velocity_max, 12);
  const auto kp_raw = encode_uint(kp, 0.0, 500.0, 12);
  const auto kd_raw = encode_uint(kd, 0.0, 5.0, 12);
  const auto torque = encode_uint(feedforward, motor.torque_min, motor.torque_max, 12);
  const double encoded_position = decode_uint(p, motor.position_min, motor.position_max, 16);
  const double encoded_velocity = decode_uint(v, motor.velocity_min, motor.velocity_max, 12);
  const double encoded_kp = decode_uint(kp_raw, 0.0, 500.0, 12);
  const double encoded_kd = decode_uint(kd_raw, 0.0, 5.0, 12);
  const double encoded_feedforward =
      decode_uint(torque, motor.torque_min, motor.torque_max, 12);
  const double estimated = encoded_kp * (encoded_position - motor.last_position) +
      encoded_kd * (encoded_velocity - motor.last_velocity) + encoded_feedforward;
  if (!std::isfinite(estimated) ||
      std::abs(estimated) > motor.commissioning_torque_cap + tolerance) {
    throw std::runtime_error("protected policy torque cap failed for " + motor.name);
  }
  motor.maximum_abs_commanded_torque =
      std::max(motor.maximum_abs_commanded_torque, std::abs(estimated));
  return {
      static_cast<std::uint8_t>(p >> 8), static_cast<std::uint8_t>(p),
      static_cast<std::uint8_t>(v >> 4),
      static_cast<std::uint8_t>((v & 0xF) << 4 | kp_raw >> 8),
      static_cast<std::uint8_t>(kp_raw), static_cast<std::uint8_t>(kd_raw >> 4),
      static_cast<std::uint8_t>((kd_raw & 0xF) << 4 | torque >> 8),
      static_cast<std::uint8_t>(torque)};
}

void send_payload(Socket& socket, Motor& motor,
                  const std::array<std::uint8_t, 8>& data) {
  canfd_frame frame{};
  frame.can_id = static_cast<canid_t>(motor.can_id);
  frame.len = 8;
  frame.flags = CANFD_BRS;
  std::copy(data.begin(), data.end(), frame.data);
  if (write(socket.fd, &frame, CANFD_MTU) != CANFD_MTU) {
    throw std::runtime_error("CAN-FD hold write failed for " + motor.name);
  }
  ++motor.tx_count;
}

void send_special(Socket& socket, Motor& motor, std::uint8_t opcode) {
  if (opcode != 0xFC && opcode != 0xFD) {
    throw std::runtime_error("native hold special opcode is not enable/disable");
  }
  std::array<std::uint8_t, 8> data{};
  data.fill(0xFF);
  data[7] = opcode;
  send_payload(socket, motor, data);
}

void drain_hold_feedback(
    std::vector<Socket>& sockets, std::vector<Motor>& motors,
    const std::map<std::pair<std::string, int>, std::size_t>& feedback_map,
    int expected_status, bool enforce_motion_guards) {
  for (auto& can_socket : sockets) {
    while (true) {
      canfd_frame frame{};
      const ssize_t received = read(can_socket.fd, &frame, CANFD_MTU);
      if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (received != CANFD_MTU || frame.len != 8 || (frame.flags & CANFD_BRS) == 0) {
        throw std::runtime_error("invalid measured-hold CAN-FD feedback frame");
      }
      const int frame_id = static_cast<int>(frame.can_id & CAN_SFF_MASK);
      const auto found = feedback_map.find({can_socket.interface, frame_id});
      if (found == feedback_map.end()) continue;
      auto& motor = motors[found->second];
      const int controller_id = frame.data[0] & 0x0F;
      const int status_code = (frame.data[0] >> 4) & 0x0F;
      const bool status_valid = expected_status >= 0
          ? status_code == expected_status
          : status_code == 0 || status_code == 1;
      if (controller_id != motor.can_id || !status_valid) {
        throw std::runtime_error("measured-hold identity/status failed for " + motor.name);
      }
      const auto raw_position =
          (static_cast<unsigned>(frame.data[1]) << 8) | frame.data[2];
      const auto raw_velocity =
          (static_cast<unsigned>(frame.data[3]) << 4) | (frame.data[4] >> 4);
      const auto raw_torque =
          (static_cast<unsigned>(frame.data[4] & 0x0F) << 8) | frame.data[5];
      motor.last_position =
          decode_uint(raw_position, motor.position_min, motor.position_max, 16);
      motor.last_velocity =
          decode_uint(raw_velocity, motor.velocity_min, motor.velocity_max, 12);
      const double feedback_torque =
          decode_uint(raw_torque, motor.torque_min, motor.torque_max, 12);
      motor.maximum_abs_position_drift = std::max(
          motor.maximum_abs_position_drift,
          std::abs(motor.last_position - motor.captured_position));
      motor.maximum_abs_speed =
          std::max(motor.maximum_abs_speed, std::abs(motor.last_velocity));
      motor.maximum_abs_feedback_torque = std::max(
          motor.maximum_abs_feedback_torque, std::abs(feedback_torque));
      motor.maximum_mos_temperature =
          std::max(motor.maximum_mos_temperature, static_cast<int>(frame.data[6]));
      motor.maximum_rotor_temperature =
          std::max(motor.maximum_rotor_temperature, static_cast<int>(frame.data[7]));
      if (frame.data[6] >= motor.mos_temperature_limit ||
          frame.data[7] >= motor.rotor_temperature_limit) {
        throw std::runtime_error("measured-hold temperature failed for " + motor.name);
      }
      if (enforce_motion_guards) {
        const bool differential = motor.name.find("ankle_motor") != std::string::npos ||
            motor.name == "head_motor_a" || motor.name == "head_motor_b";
        double torque_guard = differential ? 0.10 : 0.50;
        if (motor.name == "waist_roll_motor") torque_guard = 2.50;
        if (motor.maximum_abs_position_drift > 0.05 ||
            motor.maximum_abs_speed > 0.20 ||
            std::abs(feedback_torque) > torque_guard) {
          throw std::runtime_error("measured-hold motion/torque guard failed for " + motor.name);
        }
      }
      ++motor.rx_count;
      motor.seen = true;
      if (expected_status == 0) {
        motor.final_disabled = true;
      } else if (expected_status < 0) {
        // During shutdown, consume stale enabled replies until the later disabled
        // poll replies become the final observed state for every endpoint.
        motor.final_disabled = status_code == 0;
      }
    }
  }
}

void reset_torque_rms_monitor(Motor& motor) {
  motor.short_torque_squared_samples.clear();
  motor.long_torque_squared_samples.clear();
  motor.short_torque_squared_sum = 0.0;
  motor.long_torque_squared_sum = 0.0;
  motor.maximum_short_torque_rms = 0.0;
  motor.maximum_long_torque_rms = 0.0;
}

void update_torque_rms_monitor(Motor& motor, double torque_nm) {
  const double squared = torque_nm * torque_nm;
  const auto update_window = [&](std::deque<double>& samples, double& sum,
                                 std::size_t maximum_samples) {
    samples.push_back(squared);
    sum += squared;
    while (samples.size() > maximum_samples) {
      sum -= samples.front();
      samples.pop_front();
    }
    return std::sqrt(std::max(0.0, sum) / static_cast<double>(samples.size()));
  };
  const auto rate = static_cast<std::size_t>(motor.poll_rate_hz);
  const double short_rms = update_window(
      motor.short_torque_squared_samples, motor.short_torque_squared_sum, rate);
  const double long_rms = update_window(
      motor.long_torque_squared_samples, motor.long_torque_squared_sum, 10 * rate);
  if (motor.short_torque_squared_samples.size() == rate) {
    motor.maximum_short_torque_rms =
        std::max(motor.maximum_short_torque_rms, short_rms);
  }
  if (motor.long_torque_squared_samples.size() == 10 * rate) {
    motor.maximum_long_torque_rms =
        std::max(motor.maximum_long_torque_rms, long_rms);
  }
}

void update_temperature_filter(Motor& motor, double mos_c, double rotor_c) {
  if (!motor.temperature_filter_initialized) {
    motor.filtered_mos_temperature = mos_c;
    motor.filtered_rotor_temperature = rotor_c;
    motor.temperature_filter_initialized = true;
  } else {
    const double sample_period_s = 1.0 / static_cast<double>(motor.poll_rate_hz);
    const double alpha = 1.0 - std::exp(
        -sample_period_s / motor.temperature_filter_time_constant_s);
    motor.filtered_mos_temperature +=
        alpha * (mos_c - motor.filtered_mos_temperature);
    motor.filtered_rotor_temperature +=
        alpha * (rotor_c - motor.filtered_rotor_temperature);
  }
  motor.maximum_filtered_mos_temperature = std::max(
      motor.maximum_filtered_mos_temperature, motor.filtered_mos_temperature);
  motor.maximum_filtered_rotor_temperature = std::max(
      motor.maximum_filtered_rotor_temperature, motor.filtered_rotor_temperature);
}

void drain_policy_feedback(
    std::vector<Socket>& sockets, std::vector<Motor>& motors,
    const std::map<std::pair<std::string, int>, std::size_t>& feedback_map,
    int expected_status, bool enforce_dynamic_guards) {
  for (auto& can_socket : sockets) {
    while (true) {
      canfd_frame frame{};
      const ssize_t received = read(can_socket.fd, &frame, CANFD_MTU);
      if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (received != CANFD_MTU || frame.len != 8 || (frame.flags & CANFD_BRS) == 0) {
        throw std::runtime_error("invalid protected-policy CAN-FD feedback frame");
      }
      const int frame_id = static_cast<int>(frame.can_id & CAN_SFF_MASK);
      const auto found = feedback_map.find({can_socket.interface, frame_id});
      if (found == feedback_map.end()) continue;
      auto& motor = motors[found->second];
      const int controller_id = frame.data[0] & 0x0F;
      const int status_code = (frame.data[0] >> 4) & 0x0F;
      const bool status_valid = expected_status >= 0
          ? status_code == expected_status
          : status_code == 0 || status_code == 1;
      if (controller_id != motor.can_id || !status_valid) {
        throw std::runtime_error("protected-policy identity/status failed for " + motor.name);
      }
      const auto raw_position =
          (static_cast<unsigned>(frame.data[1]) << 8) | frame.data[2];
      const auto raw_velocity =
          (static_cast<unsigned>(frame.data[3]) << 4) | (frame.data[4] >> 4);
      const auto raw_torque =
          (static_cast<unsigned>(frame.data[4] & 0x0F) << 8) | frame.data[5];
      motor.last_position =
          decode_uint(raw_position, motor.position_min, motor.position_max, 16);
      motor.last_velocity =
          decode_uint(raw_velocity, motor.velocity_min, motor.velocity_max, 12);
      const double feedback_torque =
          decode_uint(raw_torque, motor.torque_min, motor.torque_max, 12);
      motor.last_status_code = status_code;
      motor.maximum_abs_speed =
          std::max(motor.maximum_abs_speed, std::abs(motor.last_velocity));
      motor.maximum_abs_feedback_torque = std::max(
          motor.maximum_abs_feedback_torque, std::abs(feedback_torque));
      if (enforce_dynamic_guards && status_code == 1) {
        update_torque_rms_monitor(motor, feedback_torque);
      }
      motor.maximum_mos_temperature =
          std::max(motor.maximum_mos_temperature, static_cast<int>(frame.data[6]));
      motor.maximum_rotor_temperature =
          std::max(motor.maximum_rotor_temperature, static_cast<int>(frame.data[7]));
      update_temperature_filter(
          motor, static_cast<double>(frame.data[6]), static_cast<double>(frame.data[7]));
      if (frame.data[6] >= motor.mos_temperature_limit ||
          frame.data[7] >= motor.rotor_temperature_limit) {
        throw std::runtime_error("protected-policy temperature failed for " + motor.name);
      }
      if (enforce_dynamic_guards &&
          (motor.filtered_mos_temperature >= motor.filtered_mos_temperature_limit ||
           motor.filtered_rotor_temperature >= motor.filtered_rotor_temperature_limit)) {
        throw std::runtime_error(
            "protected-policy filtered temperature guard failed for " + motor.name);
      }
      if (enforce_dynamic_guards &&
          (motor.last_position < motor.hard_position_min ||
           motor.last_position > motor.hard_position_max ||
           std::abs(motor.last_velocity) > motor.deployment_velocity_max ||
           std::abs(feedback_torque) > motor.feedback_torque_cap)) {
        throw std::runtime_error("protected-policy dynamic guard failed for " + motor.name);
      }
      ++motor.rx_count;
      motor.seen = true;
      if (expected_status == 0) {
        motor.final_disabled = true;
      } else if (expected_status < 0) {
        motor.final_disabled = status_code == 0;
      }
    }
  }
}

int run_native_measured_pose_hold(
    const Options& options, std::vector<Socket>& sockets, std::vector<Motor>& motors,
    const std::map<std::string, std::size_t>& socket_index,
    const std::map<std::pair<std::string, int>, std::size_t>& feedback_map,
    const NativeKinematics& kinematics, bool memory_locked) {
  std::string failure;
  std::size_t deadline_misses = 0;
  double maximum_lateness_ms = 0.0;
  std::array<double, kJointCount> target_joint{};
  std::array<double, kJointCount> joint_torque{};
  std::vector<std::size_t> active_tx(kJointCount, 0);
  std::vector<std::size_t> active_rx(kJointCount, 0);
  auto due = [](const Motor& motor, std::size_t slot) {
    return motor.poll_rate_hz == 500
        ? static_cast<int>(slot % 4) == motor.poll_phase
        : static_cast<int>(slot % 40) == motor.poll_phase;
  };
  auto sleep_until = [](std::int64_t deadline_ns) {
    const timespec deadline = from_ns(deadline_ns);
    int status = 0;
    do status = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    while (status == EINTR);
    if (status != 0) throw std::runtime_error("clock_nanosleep failed");
  };
  auto joint_index = [&](const std::string& name) -> std::size_t {
    const auto found = std::find(
        kinematics.joint_names.begin(), kinematics.joint_names.end(), name);
    if (found == kinematics.joint_names.end()) {
      throw std::runtime_error("missing measured-hold joint: " + name);
    }
    return static_cast<std::size_t>(found - kinematics.joint_names.begin());
  };
  const std::array<std::size_t, 6> controlled_joints{
      joint_index("left_ankle_pitch_joint"), joint_index("left_ankle_roll_joint"),
      joint_index("right_ankle_pitch_joint"), joint_index("right_ankle_roll_joint"),
      joint_index("head_pitch_joint"), joint_index("head_roll_joint")};

  try {
    // Capture every endpoint while all drives are still disabled.
    constexpr std::size_t warmup_slots = 400;
    const std::int64_t warmup_start = monotonic_ns() + 50'000'000LL;
    for (std::size_t slot = 0; slot < warmup_slots; ++slot) {
      sleep_until(warmup_start + static_cast<std::int64_t>(slot) * kTransportPeriodNs);
      for (auto& motor : motors) {
        if (due(motor, slot)) send_poll(sockets[socket_index.at(motor.interface)], motor);
      }
      drain_hold_feedback(sockets, motors, feedback_map, 0, false);
    }
    usleep(2000);
    drain_hold_feedback(sockets, motors, feedback_map, 0, false);
    for (auto& motor : motors) {
      if (!motor.seen || motor.rx_count == 0 ||
          motor.last_position < motor.soft_position_min + 0.05 ||
          motor.last_position > motor.soft_position_max - 0.05 ||
          std::abs(motor.last_velocity) > 0.20) {
        throw std::runtime_error("disabled capture invariant failed for " + motor.name);
      }
      motor.captured_position = motor.last_position;
      motor.tx_count = 0;
      motor.rx_count = 0;
      motor.seen = false;
      motor.maximum_abs_position_drift = 0.0;
      motor.maximum_abs_speed = 0.0;
      motor.maximum_abs_commanded_torque = 0.0;
      motor.maximum_abs_feedback_torque = 0.0;
      reset_torque_rms_monitor(motor);
      motor.final_disabled = false;
    }
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      for (std::size_t motor = 0; motor < kJointCount; ++motor) {
        target_joint[joint] += kinematics.motor_to_joint[joint][motor] *
            (motors[motor].captured_position - kinematics.motor_offset[motor]);
      }
    }

    for (auto& motor : motors) {
      send_special(sockets[socket_index.at(motor.interface)], motor, 0xFC);
    }
    for (auto& motor : motors) motor.tx_count = 0;

    const std::size_t transport_slots =
        static_cast<std::size_t>(std::llround(options.duration_s * kTransportRateHz));
    const std::int64_t start_ns = monotonic_ns() + 20'000'000LL;
    for (std::size_t slot = 0; slot < transport_slots; ++slot) {
      const std::int64_t deadline_ns =
          start_ns + static_cast<std::int64_t>(slot) * kTransportPeriodNs;
      sleep_until(deadline_ns);
      const std::int64_t woke_ns = monotonic_ns();
      if (slot % 4 == 0) {
        const double lateness_ms =
            std::max(0.0, static_cast<double>(woke_ns - deadline_ns) / 1.0e6);
        maximum_lateness_ms = std::max(maximum_lateness_ms, lateness_ms);
        deadline_misses += static_cast<std::size_t>(lateness_ms >= 2.0);

        std::array<double, kJointCount> position{};
        std::array<double, kJointCount> velocity{};
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
          for (std::size_t motor = 0; motor < kJointCount; ++motor) {
            position[joint] += kinematics.motor_to_joint[joint][motor] *
                (motors[motor].last_position - kinematics.motor_offset[motor]);
            velocity[joint] += kinematics.motor_to_joint[joint][motor] *
                motors[motor].last_velocity;
          }
        }
        joint_torque.fill(0.0);
        for (std::size_t offset = 0; offset < controlled_joints.size(); ++offset) {
          const std::size_t joint = controlled_joints[offset];
          const bool head = offset >= 4;
          const double kp = head ? 0.2 : 0.5;
          const double kd = head ? 0.03 : 0.05;
          const double cap = head ? 0.05 : 0.15;
          const double error = target_joint[joint] - position[joint];
          if (std::abs(error) > 0.05) {
            throw std::runtime_error(
                "measured-hold joint error failed for " + kinematics.joint_names[joint]);
          }
          joint_torque[joint] = std::clamp(kp * error - kd * velocity[joint], -cap, cap);
        }
      }

      for (std::size_t index = 0; index < motors.size(); ++index) {
        auto& motor = motors[index];
        if (!due(motor, slot)) continue;
        const bool differential = motor.name.find("ankle_motor") != std::string::npos ||
            motor.name == "head_motor_a" || motor.name == "head_motor_b";
        double feedforward = 0.0;
        if (differential) {
          for (std::size_t joint = 0; joint < kJointCount; ++joint) {
            feedforward += kinematics.motor_to_joint[joint][index] * joint_torque[joint];
          }
          if (std::abs(feedforward) > 0.10) {
            throw std::runtime_error("differential motor torque cap failed for " + motor.name);
          }
        }
        const auto payload = differential
            ? hold_payload(motor, motor.last_position, 0.0, 0.0, feedforward, 0.10)
            : hold_payload(motor, motor.captured_position, 0.2, 0.05, 0.0, 0.50);
        send_payload(sockets[socket_index.at(motor.interface)], motor, payload);
      }
      drain_hold_feedback(sockets, motors, feedback_map, 1, true);
    }
    usleep(2000);
    drain_hold_feedback(sockets, motors, feedback_map, 1, true);
    for (std::size_t index = 0; index < motors.size(); ++index) {
      active_tx[index] = motors[index].tx_count;
      active_rx[index] = motors[index].rx_count;
      const double coverage = active_tx[index] > 0
          ? static_cast<double>(active_rx[index]) / active_tx[index] : 0.0;
      if (coverage < 0.95) {
        throw std::runtime_error("measured-hold coverage failed for " + motors[index].name);
      }
    }
    if (deadline_misses != 0 || maximum_lateness_ms >= 2.0) {
      throw std::runtime_error("measured-hold realtime deadline gate failed");
    }
  } catch (const std::exception& error) {
    failure = error.what();
  }

  // Preserve partial active-run counts even when a guard aborts the hold.
  for (std::size_t index = 0; index < motors.size(); ++index) {
    active_tx[index] = motors[index].tx_count;
    active_rx[index] = motors[index].rx_count;
  }

  // This block runs after success and every failure. 0xFD is disable, never set-zero.
  try {
    for (int attempt = 0; attempt < 3; ++attempt) {
      for (auto& motor : motors) {
        send_special(sockets[socket_index.at(motor.interface)], motor, 0xFD);
      }
      usleep(10000);
    }
    for (auto& motor : motors) {
      motor.seen = false;
      motor.final_disabled = false;
    }
    const std::int64_t verify_start = monotonic_ns() + 10'000'000LL;
    for (std::size_t slot = 0; slot < 400; ++slot) {
      sleep_until(verify_start + static_cast<std::int64_t>(slot) * kTransportPeriodNs);
      for (auto& motor : motors) {
        if (due(motor, slot)) send_poll(sockets[socket_index.at(motor.interface)], motor);
      }
      drain_hold_feedback(sockets, motors, feedback_map, -1, false);
    }
    usleep(2000);
    drain_hold_feedback(sockets, motors, feedback_map, -1, false);
    for (const auto& motor : motors) {
      if (!motor.final_disabled) {
        throw std::runtime_error("final disabled verification failed for " + motor.name);
      }
    }
  } catch (const std::exception& error) {
    if (!failure.empty()) failure += "; ";
    failure += error.what();
  }

  std::ostringstream report;
  report << std::fixed << std::setprecision(6)
      << "{\n  \"mode\": \"native_suspended_31_motor_measured_pose_hold\",\n"
      << "  \"duration_s\": " << options.duration_s << ",\n"
      << "  \"scheduler\": \""
      << (options.realtime_priority > 0 ? "SCHED_FIFO" : "SCHED_OTHER") << "\",\n"
      << "  \"realtime_priority\": " << options.realtime_priority << ",\n"
      << "  \"memory_locked\": " << (memory_locked ? "true" : "false") << ",\n"
      << "  \"deadline_misses\": " << deadline_misses << ",\n"
      << "  \"maximum_lateness_ms\": " << maximum_lateness_ms << ",\n"
      << "  \"automatic_mode_switch_attempts\": 0,\n"
      << "  \"automatic_zero_reset_attempts\": 0,\n"
      << "  \"motors\": {\n";
  for (std::size_t index = 0; index < motors.size(); ++index) {
    const auto& motor = motors[index];
    report << "    \"" << motor.name << "\": {"
        << "\"tx\": " << active_tx[index] << ", \"rx\": " << active_rx[index]
        << ", \"max_drift_rad\": " << motor.maximum_abs_position_drift
        << ", \"max_speed_rad_s\": " << motor.maximum_abs_speed
        << ", \"max_command_torque_nm\": " << motor.maximum_abs_commanded_torque
        << ", \"max_feedback_torque_nm\": " << motor.maximum_abs_feedback_torque
        << ", \"final_disabled\": " << (motor.final_disabled ? "true" : "false")
        << "}" << (index + 1 == motors.size() ? "\n" : ",\n");
  }
  const bool passed = failure.empty();
  report << "  },\n  \"errors\": [";
  if (!failure.empty()) report << "\"" << failure << "\"";
  report << "],\n  \"passed\": " << (passed ? "true" : "false") << "\n}\n";
  std::cout << report.str();
  std::ofstream output(options.output);
  if (!output) throw std::runtime_error("cannot open measured-hold report output");
  output << report.str();
  return passed ? 0 : 2;
}

int run_native_protected_policy(
    const Options& options, std::vector<Socket>& sockets, std::vector<Motor>& motors,
    const std::map<std::string, std::size_t>& socket_index,
    const std::map<std::pair<std::string, int>, std::size_t>& feedback_map,
    const NativeKinematics& kinematics, const std::vector<JointSafety>& joint_safety,
    IpcSocket& ipc, std::uint64_t motor_hash, bool memory_locked) {
  std::string failure;
  std::size_t deadline_misses = 0;
  double maximum_lateness_ms = 0.0;
  std::vector<std::size_t> active_tx(kJointCount, 0);
  std::vector<std::size_t> active_rx(kJointCount, 0);
  auto due = [](const Motor& motor, std::size_t slot) {
    return motor.poll_rate_hz == 500
        ? static_cast<int>(slot % 4) == motor.poll_phase
        : static_cast<int>(slot % 40) == motor.poll_phase;
  };
  auto sleep_until = [](std::int64_t deadline_ns) {
    const timespec deadline = from_ns(deadline_ns);
    int status = 0;
    do status = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    while (status == EINTR);
    if (status != 0) throw std::runtime_error("clock_nanosleep failed");
  };
  auto all_seen = [&]() {
    return std::all_of(motors.begin(), motors.end(), [](const Motor& motor) {
      return motor.seen;
    });
  };

  try {
    // Keep every drive disabled while obtaining complete state, live IMU policy
    // output, and the first zero-gain measured-pose startup target.
    constexpr std::size_t preparation_slots = 4000;
    const std::int64_t preparation_start = monotonic_ns() + 50'000'000LL;
    bool prepared = false;
    for (std::size_t slot = 0; slot < preparation_slots; ++slot) {
      if (g_shutdown_requested) {
        throw std::runtime_error("protected-policy shutdown signal received during preparation");
      }
      sleep_until(preparation_start + static_cast<std::int64_t>(slot) * kTransportPeriodNs);
      for (auto& motor : motors) {
        if (due(motor, slot)) send_poll(sockets[socket_index.at(motor.interface)], motor);
      }
      drain_policy_feedback(sockets, motors, feedback_map, 0, false);
      drain_target_packets(ipc, options.policy_joint_hash, joint_safety, monotonic_ns());
      if (slot % 40 == 39 && all_seen()) {
        send_state_packet(ipc, motors, motor_hash, monotonic_ns());
      }
      if (!ipc.has_target || !all_seen()) continue;

      std::array<double, kJointCount> measured_joint{};
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        for (std::size_t motor = 0; motor < kJointCount; ++motor) {
          measured_joint[joint] += kinematics.motor_to_joint[joint][motor] *
              (motors[motor].last_position - kinematics.motor_offset[motor]);
        }
        const double position_delta =
            ipc.latest_target.position_rad[joint] - measured_joint[joint];
        auto startup_error = [&](const std::string& field, double value) {
          std::ostringstream message;
          message << "first protected-policy target failed for "
                  << joint_safety[joint].name << " field=" << field
                  << " value=" << value;
          throw std::runtime_error(message.str());
        };
        if (std::abs(position_delta) > 0.03) {
          startup_error("position_delta_rad", position_delta);
        }
        if (std::abs(ipc.latest_target.velocity_rad_s[joint]) > 1.0e-9) {
          startup_error("velocity_rad_s", ipc.latest_target.velocity_rad_s[joint]);
        }
        if (std::abs(ipc.latest_target.kp[joint]) > 1.0e-9) {
          startup_error("kp", ipc.latest_target.kp[joint]);
        }
        if (std::abs(ipc.latest_target.kd[joint]) > 1.0e-9) {
          startup_error("kd", ipc.latest_target.kd[joint]);
        }
        if (std::abs(ipc.latest_target.feedforward_torque_nm[joint]) > 1.0e-9) {
          startup_error(
              "feedforward_torque_nm",
              ipc.latest_target.feedforward_torque_nm[joint]);
        }
      }
      prepared = true;
      break;
    }
    if (!prepared) {
      throw std::runtime_error("protected-policy startup target was not ready within 2 seconds");
    }
    for (auto& motor : motors) {
      motor.captured_position = motor.last_position;
      motor.tx_count = 0;
      motor.rx_count = 0;
      motor.seen = false;
      motor.preview_initialized = false;
      motor.maximum_abs_speed = 0.0;
      motor.maximum_abs_commanded_torque = 0.0;
      motor.maximum_abs_feedback_torque = 0.0;
      reset_torque_rms_monitor(motor);
      motor.final_disabled = false;
      motor.last_status_code = 0;
    }
    ipc.motor_preview_count = 0;

    for (auto& motor : motors) {
      send_special(sockets[socket_index.at(motor.interface)], motor, 0xFC);
    }
    for (auto& motor : motors) motor.tx_count = 0;

    const std::size_t transport_slots =
        static_cast<std::size_t>(std::llround(options.duration_s * kTransportRateHz));
    const std::int64_t start_ns = monotonic_ns() + 20'000'000LL;
    for (std::size_t slot = 0; slot < transport_slots; ++slot) {
      if (g_shutdown_requested) {
        throw std::runtime_error("protected-policy shutdown signal received");
      }
      const std::int64_t deadline_ns =
          start_ns + static_cast<std::int64_t>(slot) * kTransportPeriodNs;
      sleep_until(deadline_ns);
      const std::int64_t woke_ns = monotonic_ns();
      if (slot % 4 == 0) {
        const double lateness_ms =
            std::max(0.0, static_cast<double>(woke_ns - deadline_ns) / 1.0e6);
        maximum_lateness_ms = std::max(maximum_lateness_ms, lateness_ms);
        deadline_misses += static_cast<std::size_t>(lateness_ms >= 2.0);
      }

      drain_target_packets(ipc, options.policy_joint_hash, joint_safety, woke_ns);
      if (!ipc.has_target || woke_ns - ipc.last_target_ns > kTargetTimeoutNs) {
        throw std::runtime_error("protected-policy target watchdog expired");
      }
      if (slot % 4 == 0) {
        preview_final_motor_commands(
            motors, ipc, kinematics, options.saturate_policy_torque);
      }
      for (auto& motor : motors) {
        if (!due(motor, slot)) continue;
        const auto payload = protected_policy_payload(motor);
        send_payload(sockets[socket_index.at(motor.interface)], motor, payload);
      }
      // Permit only the short enable transition; require all endpoints enabled
      // after 100 ms and throughout the remaining active interval.
      drain_policy_feedback(
          sockets, motors, feedback_map, slot < 200 ? -1 : 1, true);
      if (slot == 200) {
        for (const auto& motor : motors) {
          if (motor.last_status_code != 1) {
            throw std::runtime_error("protected-policy enable verification failed for " +
                                     motor.name);
          }
        }
      }
      if (slot % 40 == 39 && all_seen()) {
        send_state_packet(ipc, motors, motor_hash, monotonic_ns());
      }
    }
    usleep(2000);
    drain_policy_feedback(sockets, motors, feedback_map, 1, true);
    for (std::size_t index = 0; index < motors.size(); ++index) {
      active_tx[index] = motors[index].tx_count;
      active_rx[index] = motors[index].rx_count;
      const double coverage = active_tx[index] > 0
          ? static_cast<double>(active_rx[index]) / active_tx[index] : 0.0;
      if (coverage < 0.95) {
        throw std::runtime_error("protected-policy coverage failed for " + motors[index].name);
      }
    }
    if (deadline_misses != 0 || maximum_lateness_ms >= 2.0) {
      throw std::runtime_error("protected-policy realtime deadline gate failed");
    }
  } catch (const std::exception& error) {
    failure = error.what();
  }

  for (std::size_t index = 0; index < motors.size(); ++index) {
    active_tx[index] = motors[index].tx_count;
    active_rx[index] = motors[index].rx_count;
  }

  try {
    for (int attempt = 0; attempt < 3; ++attempt) {
      for (auto& motor : motors) {
        send_special(sockets[socket_index.at(motor.interface)], motor, 0xFD);
      }
      usleep(10000);
    }
    for (auto& motor : motors) {
      motor.seen = false;
      motor.final_disabled = false;
    }
    const std::int64_t verify_start = monotonic_ns() + 10'000'000LL;
    for (std::size_t slot = 0; slot < 400; ++slot) {
      sleep_until(verify_start + static_cast<std::int64_t>(slot) * kTransportPeriodNs);
      for (auto& motor : motors) {
        if (due(motor, slot)) send_poll(sockets[socket_index.at(motor.interface)], motor);
      }
      drain_policy_feedback(sockets, motors, feedback_map, -1, false);
    }
    usleep(2000);
    drain_policy_feedback(sockets, motors, feedback_map, -1, false);
    for (const auto& motor : motors) {
      if (!motor.final_disabled) {
        throw std::runtime_error("final disabled verification failed for " + motor.name);
      }
    }
  } catch (const std::exception& error) {
    if (!failure.empty()) failure += "; ";
    failure += error.what();
  }

  std::ostringstream report;
  report << std::fixed << std::setprecision(6)
      << "{\n  \"mode\": \"native_suspended_protected_policy_actuation\",\n"
      << "  \"duration_s\": " << options.duration_s << ",\n"
      << "  \"memory_locked\": " << (memory_locked ? "true" : "false") << ",\n"
      << "  \"deadline_misses\": " << deadline_misses << ",\n"
      << "  \"maximum_lateness_ms\": " << maximum_lateness_ms << ",\n"
      << "  \"policy_state_count\": " << ipc.state_sequence << ",\n"
      << "  \"policy_target_count\": " << ipc.target_count << ",\n"
      << "  \"policy_torque_saturation_enabled\": "
      << (options.saturate_policy_torque ? "true" : "false") << ",\n"
      << "  \"automatic_mode_switch_attempts\": 0,\n"
      << "  \"automatic_zero_reset_attempts\": 0,\n"
      << "  \"motors\": {\n";
  for (std::size_t index = 0; index < motors.size(); ++index) {
    const auto& motor = motors[index];
    report << "    \"" << motor.name << "\": {"
        << "\"tx\": " << active_tx[index] << ", \"rx\": " << active_rx[index]
        << ", \"commissioning_torque_cap_nm\": " << motor.commissioning_torque_cap
        << ", \"feedback_torque_cap_nm\": " << motor.feedback_torque_cap
        << ", \"rated_torque_nm\": " << motor.rated_torque
        << ", \"max_command_torque_nm\": " << motor.maximum_abs_commanded_torque
        << ", \"max_feedback_torque_nm\": " << motor.maximum_abs_feedback_torque
        << ", \"max_1s_feedback_torque_rms_nm\": "
        << motor.maximum_short_torque_rms
        << ", \"max_10s_feedback_torque_rms_nm\": "
        << motor.maximum_long_torque_rms
        << ", \"max_1s_rated_torque_ratio\": "
        << motor.maximum_short_torque_rms / motor.rated_torque
        << ", \"max_10s_rated_torque_ratio\": "
        << motor.maximum_long_torque_rms / motor.rated_torque
        << ", \"raw_mos_temperature_limit_c\": " << motor.mos_temperature_limit
        << ", \"raw_rotor_temperature_limit_c\": " << motor.rotor_temperature_limit
        << ", \"filtered_mos_temperature_limit_c\": "
        << motor.filtered_mos_temperature_limit
        << ", \"filtered_rotor_temperature_limit_c\": "
        << motor.filtered_rotor_temperature_limit
        << ", \"temperature_filter_time_constant_s\": "
        << motor.temperature_filter_time_constant_s
        << ", \"max_raw_mos_temperature_c\": " << motor.maximum_mos_temperature
        << ", \"max_raw_rotor_temperature_c\": " << motor.maximum_rotor_temperature
        << ", \"max_filtered_mos_temperature_c\": "
        << motor.maximum_filtered_mos_temperature
        << ", \"max_filtered_rotor_temperature_c\": "
        << motor.maximum_filtered_rotor_temperature
        << ", \"torque_saturation_count\": " << motor.policy_torque_saturation_count
        << ", \"max_unsaturated_torque_nm\": "
        << motor.preview_maximum_abs_unsaturated_torque
        << ", \"max_speed_rad_s\": " << motor.maximum_abs_speed
        << ", \"final_disabled\": " << (motor.final_disabled ? "true" : "false")
        << "}" << (index + 1 == motors.size() ? "\n" : ",\n");
  }
  const bool passed = failure.empty();
  report << "  },\n  \"errors\": [";
  if (!failure.empty()) report << "\"" << failure << "\"";
  report << "],\n  \"passed\": " << (passed ? "true" : "false") << "\n}\n";
  std::cout << report.str();
  std::ofstream output(options.output);
  if (!output) throw std::runtime_error("cannot open protected-policy report output");
  output << report.str();
  return passed ? 0 : 2;
}

void apply_affinity(int cpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
    throw std::runtime_error("failed to apply CPU affinity");
  }
}

void apply_realtime_priority(int priority) {
  if (priority == 0) return;
  sched_param parameters{};
  parameters.sched_priority = priority;
  const int status = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
  if (status != 0) {
    throw std::runtime_error(
        "failed to apply SCHED_FIFO priority " + std::to_string(priority) + ": " +
        std::strerror(status));
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
                        int cpu, int realtime_priority, const IpcSocket* ipc,
                        bool protected_target_envelope_enabled,
                        std::int64_t finished_ns) {
  const double p99 = percentile(lateness, 0.99);
  const double maximum = *std::max_element(lateness.begin(), lateness.end());
  bool coverage_passed = true;
  std::ostringstream tx_counts;
  std::ostringstream rx_counts;
  std::ostringstream coverage_values;
  std::ostringstream mos_temperatures;
  std::ostringstream rotor_temperatures;
  std::ostringstream preview_torque;
  tx_counts << "  \"tx_count_by_motor\": {\n";
  rx_counts << "  \"rx_count_by_motor\": {\n";
  coverage_values << "  \"sample_coverage_by_motor\": {\n";
  mos_temperatures << "  \"maximum_mos_temperature_c_by_motor\": {\n";
  rotor_temperatures << "  \"maximum_rotor_temperature_c_by_motor\": {\n";
  preview_torque << "  \"preview_maximum_abs_estimated_torque_nm_by_motor\": {\n";
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
    mos_temperatures << "    \"" << motor.name << "\": "
                     << motor.maximum_mos_temperature << suffix;
    rotor_temperatures << "    \"" << motor.name << "\": "
                       << motor.maximum_rotor_temperature << suffix;
    preview_torque << "    \"" << motor.name << "\": "
                   << motor.preview_maximum_abs_estimated_torque << suffix;
  }
  tx_counts << "  },\n";
  rx_counts << "  },\n";
  coverage_values << "  },\n";
  mos_temperatures << "  },\n";
  rotor_temperatures << "  },\n";
  preview_torque << "  },\n";
  const bool ipc_enabled = ipc != nullptr;
  double preview_position_max = 0.0;
  double preview_velocity_max = 0.0;
  double preview_kp_max = 0.0;
  double preview_kd_max = 0.0;
  double preview_feedforward_max = 0.0;
  double preview_estimated_torque_max = 0.0;
  for (const auto& motor : motors) {
    preview_position_max = std::max(preview_position_max, motor.preview_maximum_abs_position);
    preview_velocity_max = std::max(preview_velocity_max, motor.preview_maximum_abs_velocity);
    preview_kp_max = std::max(preview_kp_max, motor.preview_maximum_kp);
    preview_kd_max = std::max(preview_kd_max, motor.preview_maximum_kd);
    preview_feedforward_max = std::max(
        preview_feedforward_max, motor.preview_maximum_abs_feedforward_torque);
    preview_estimated_torque_max = std::max(
        preview_estimated_torque_max, motor.preview_maximum_abs_estimated_torque);
  }
  const double target_coverage = ipc_enabled && ipc->state_sequence > 0
      ? static_cast<double>(ipc->target_count) / static_cast<double>(ipc->state_sequence)
      : 0.0;
  const double final_target_age_ms = ipc_enabled && ipc->last_target_ns > 0
      ? static_cast<double>(finished_ns - ipc->last_target_ns) / 1.0e6
      : 0.0;
  const bool ipc_passed = !ipc_enabled ||
      (ipc->state_sequence > 0 && target_coverage >= 0.90 &&
       ipc->last_target_ns > 0 && final_target_age_ms <= 100.0 &&
       ipc->motor_preview_count > 0);
  const bool passed = deadline_misses == 0 && p99 <= 0.5 && maximum <= 2.0 &&
      coverage_passed && ipc_passed;
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "{\n  \"mode\": \"native_mixed_rate_zero_gain_can_shadow\",\n"
      << "  \"elapsed_s\": " << elapsed_s << ",\n"
      << "  \"state_ticks\": " << lateness.size() << ",\n"
      << "  \"deadline_misses\": " << deadline_misses << ",\n"
      << "  \"lateness_p99_ms\": " << p99 << ",\n"
      << "  \"lateness_max_ms\": " << maximum << ",\n"
      << "  \"cpu\": " << cpu << ",\n"
      << "  \"scheduler\": \"" << (realtime_priority > 0 ? "SCHED_FIFO" : "SCHED_OTHER")
      << "\",\n"
      << "  \"realtime_priority\": " << realtime_priority << ",\n"
      << "  \"memory_locked\": " << (memory_locked ? "true" : "false") << ",\n"
      << "  \"policy_ipc_enabled\": " << (ipc_enabled ? "true" : "false") << ",\n"
      << "  \"policy_ipc_state_count\": " << (ipc_enabled ? ipc->state_sequence : 0) << ",\n"
      << "  \"policy_ipc_target_count\": " << (ipc_enabled ? ipc->target_count : 0) << ",\n"
      << "  \"policy_ipc_target_coverage\": " << target_coverage << ",\n"
      << "  \"policy_ipc_maximum_target_age_ms\": "
      << (ipc_enabled ? ipc->maximum_target_age_ms : 0.0) << ",\n"
      << "  \"policy_ipc_final_target_age_ms\": " << final_target_age_ms << ",\n"
      << "  \"policy_ipc_passed\": " << (ipc_passed ? "true" : "false") << ",\n"
      << "  \"final_motor_command_preview_enabled\": "
      << (ipc_enabled ? "true" : "false") << ",\n"
      << "  \"final_motor_command_preview_count\": "
      << (ipc_enabled ? ipc->motor_preview_count : 0) << ",\n"
      << "  \"preview_maximum_abs_position_rad\": " << preview_position_max << ",\n"
      << "  \"preview_maximum_abs_velocity_rad_s\": " << preview_velocity_max << ",\n"
      << "  \"preview_maximum_embedded_kp\": " << preview_kp_max << ",\n"
      << "  \"preview_maximum_embedded_kd\": " << preview_kd_max << ",\n"
      << "  \"preview_maximum_abs_feedforward_torque_nm\": "
      << preview_feedforward_max << ",\n"
      << "  \"preview_maximum_abs_estimated_torque_nm\": "
      << preview_estimated_torque_max << ",\n"
      << "  \"protected_target_envelope_enabled\": "
      << (protected_target_envelope_enabled ? "true" : "false") << ",\n"
      << "  \"nonzero_gain_or_torque_tx_attempts\": 0,\n"
      << "  \"automatic_enable_attempts\": 0,\n"
      << "  \"automatic_mode_switch_attempts\": 0,\n"
      << tx_counts.str()
      << rx_counts.str()
      << coverage_values.str()
      << mos_temperatures.str()
      << rotor_temperatures.str()
      << preview_torque.str()
      << "  \"coverage_passed\": " << (coverage_passed ? "true" : "false") << ",\n"
      << "  \"passed\": " << (passed ? "true" : "false") << "\n}\n";
  return out.str();
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<Socket> sockets;
  IpcSocket ipc;
  try {
    const Options options = parse_options(argc, argv);
    auto motors = load_motors(options.config);
    const auto joint_safety = options.joint_safety_config.empty()
        ? std::vector<JointSafety>{}
        : load_joint_safety(options.joint_safety_config);
    const auto kinematics = options.kinematics_config.empty()
        ? NativeKinematics{}
        : load_kinematics(options.kinematics_config, motors);
    if (!joint_safety.empty() &&
        ordered_joint_name_hash(joint_safety) != options.policy_joint_hash) {
      throw std::runtime_error("native joint safety order/hash mismatch");
    }
    if (!options.kinematics_config.empty() && !options.measured_pose_hold) {
      if (ordered_joint_name_hash(kinematics.joint_names) != options.policy_joint_hash) {
        throw std::runtime_error("native kinematics joint order/hash mismatch");
      }
    }
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
    const std::uint64_t motor_hash = ordered_name_hash(motors);
    if (!options.ipc_socket.empty()) {
      ipc = open_ipc_server(options.ipc_socket);
    }
    const std::size_t transport_slots =
        static_cast<std::size_t>(std::llround(options.duration_s * kTransportRateHz));
    const std::size_t state_ticks = transport_slots / 4;
    std::vector<double> lateness(state_ticks);
    apply_affinity(options.cpu);
    apply_realtime_priority(options.realtime_priority);
    const bool memory_locked = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;
    if (options.measured_pose_hold) {
      const int result = run_native_measured_pose_hold(
          options, sockets, motors, socket_index, feedback_map, kinematics,
          memory_locked);
      for (auto& can_socket : sockets) close(can_socket.fd);
      close_ipc(ipc);
      return result;
    }
    if (options.policy_actuation) {
      std::signal(SIGINT, request_shutdown);
      std::signal(SIGTERM, request_shutdown);
      const int result = run_native_protected_policy(
          options, sockets, motors, socket_index, feedback_map, kinematics,
          joint_safety, ipc, motor_hash, memory_locked);
      for (auto& can_socket : sockets) close(can_socket.fd);
      close_ipc(ipc);
      return result;
    }
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
          const auto raw_velocity = (static_cast<unsigned>(frame.data[3]) << 4) | (frame.data[4] >> 4);
          motor.last_position = decode_uint(raw_position, motor.position_min, motor.position_max, 16);
          motor.last_velocity = decode_uint(raw_velocity, motor.velocity_min, motor.velocity_max, 12);
          motor.maximum_mos_temperature =
              std::max(motor.maximum_mos_temperature, static_cast<int>(frame.data[6]));
          motor.maximum_rotor_temperature =
              std::max(motor.maximum_rotor_temperature, static_cast<int>(frame.data[7]));
          if (frame.data[6] >= motor.mos_temperature_limit ||
              frame.data[7] >= motor.rotor_temperature_limit) {
            throw std::runtime_error("motor temperature invariant failed for " + motor.name);
          }
          ++motor.rx_count;
          motor.seen = true;
        }
      }
      if (ipc.client_fd >= 0) {
        drain_target_packets(ipc, options.policy_joint_hash, joint_safety, monotonic_ns());
        if (slot % 4 == 0 &&
            std::all_of(motors.begin(), motors.end(), [](const Motor& motor) { return motor.seen; })) {
          preview_final_motor_commands(motors, ipc, kinematics, false);
        }
        if (slot % 40 == 39 &&
            std::all_of(motors.begin(), motors.end(), [](const Motor& motor) { return motor.seen; })) {
          send_state_packet(ipc, motors, motor_hash, monotonic_ns());
          if (ipc.target_count == 0 && ipc.state_sequence >= 5) {
            throw std::runtime_error("policy IPC initial target watchdog expired");
          }
          if (ipc.last_target_ns > 0 && monotonic_ns() - ipc.last_target_ns > kTargetTimeoutNs) {
            throw std::runtime_error("policy IPC target watchdog expired");
          }
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
        const auto raw_velocity = (static_cast<unsigned>(frame.data[3]) << 4) | (frame.data[4] >> 4);
        motor.last_position = decode_uint(raw_position, motor.position_min, motor.position_max, 16);
        motor.last_velocity = decode_uint(raw_velocity, motor.velocity_min, motor.velocity_max, 12);
        motor.maximum_mos_temperature =
            std::max(motor.maximum_mos_temperature, static_cast<int>(frame.data[6]));
        motor.maximum_rotor_temperature =
            std::max(motor.maximum_rotor_temperature, static_cast<int>(frame.data[7]));
        if (frame.data[6] >= motor.mos_temperature_limit ||
            frame.data[7] >= motor.rotor_temperature_limit) {
          throw std::runtime_error("final motor temperature invariant failed for " + motor.name);
        }
        ++motor.rx_count;
        motor.seen = true;
      }
    }
    const std::int64_t finished_ns = monotonic_ns();
    if (ipc.client_fd >= 0) {
      drain_target_packets(ipc, options.policy_joint_hash, joint_safety, finished_ns);
    }
    const double elapsed_s = static_cast<double>(finished_ns - start_ns) / 1.0e9;
    const std::string report = json_report(
        motors, lateness, elapsed_s, deadline_misses, memory_locked, options.cpu,
        options.realtime_priority, ipc.client_fd >= 0 ? &ipc : nullptr,
        !joint_safety.empty(), finished_ns);
    std::cout << report;
    std::ofstream output(options.output);
    if (!output) throw std::runtime_error("cannot open report output");
    output << report;
    for (auto& can_socket : sockets) close(can_socket.fd);
    close_ipc(ipc);
    return report.find("\"passed\": true") != std::string::npos ? 0 : 2;
  } catch (const std::exception& error) {
    for (auto& can_socket : sockets) if (can_socket.fd >= 0) close(can_socket.fd);
    close_ipc(ipc);
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}
