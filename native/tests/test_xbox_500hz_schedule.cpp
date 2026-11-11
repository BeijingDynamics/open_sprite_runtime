#define main sprite_transport_main
#include "../src/sprite_can_shadow.cpp"
#undef main

void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

std::vector<Motor> fixture() {
  std::vector<Motor> motors;
  for (int bus = 1; bus <= 4; ++bus) {
    for (int id = 1; id <= (bus == 4 ? 7 : 8); ++id) {
      Motor motor;
      motor.interface = "kcan" + std::to_string(bus);
      motor.can_id = id;
      motor.master_id = id + 0x10;
      const bool ankle = bus <= 2 && (id == 5 || id == 6);
      motor.poll_rate_hz = ankle ? 500 : 50;
      motor.poll_phase = ankle ? 2 * (id - 5) : 1;
      motors.push_back(motor);
    }
  }
  return motors;
}

Options options(std::vector<std::string> extra) {
  std::vector<std::string> args = {"shadow", "--config", "motors.tsv", "--output", "report.json",
      "--acknowledge-hardware-tx", "ZERO_GAIN_NATIVE_SHADOW", "--all-motors-disabled-confirmed"};
  args.insert(args.end(), extra.begin(), extra.end());
  std::vector<char*> argv;
  for (auto& arg : args) argv.push_back(arg.data());
  return parse_options(static_cast<int>(argv.size()), argv.data());
}

int main() {
  const auto baseline = fixture();
  auto candidate = baseline;
  configure_xbox_500hz_schedule(candidate);
  std::array<std::size_t, kJointCount> counts{};
  for (std::size_t slot = 0; slot < kTransportRateHz; ++slot) {
    std::map<std::string, int> counts_by_bus;
    for (std::size_t i = 0; i < candidate.size(); ++i) {
      if (!poll_due(candidate[i], slot)) continue;
      ++counts[i];
      require(++counts_by_bus[candidate[i].interface] <= 2, "too many frames in CAN slot");
    }
  }
  for (std::size_t i = 0; i < candidate.size(); ++i) {
    require(counts[i] == 500, "expected exactly 500 slots per motor per second");
    require(candidate[i].can_id == baseline[i].can_id &&
            candidate[i].master_id == baseline[i].master_id &&
            candidate[i].interface == baseline[i].interface, "endpoint order changed");
    if (baseline[i].poll_rate_hz == 500) {
      require(candidate[i].poll_phase == baseline[i].poll_phase, "ankle phase changed");
    }
  }
  require(!options({}).xbox_500hz_shadow, "legacy mode must remain default");
  require(options({"--xbox-500hz-shadow"}).xbox_500hz_shadow, "experiment flag not applied");
  for (const auto* forbidden : {"--policy-actuation", "--measured-pose-hold"}) {
    bool rejected = false;
    try {
      (void)options({"--xbox-500hz-shadow", forbidden});
    } catch (const std::runtime_error& error) {
      rejected = std::string(error.what()).find("zero-gain shadow only") != std::string::npos;
    }
    require(rejected, "experimental powered operation was not refused");
  }
  candidate[1].can_id = candidate[0].can_id;
  bool rejected = false;
  try {
    configure_xbox_500hz_schedule(candidate);
  } catch (const std::runtime_error& error) {
    rejected = std::string(error.what()).find("duplicate CAN command endpoint") != std::string::npos;
  }
  require(rejected, "duplicate endpoint was not refused");
}
