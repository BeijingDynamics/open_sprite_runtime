#define main sprite_transport_main
#include "../src/sprite_can_shadow.cpp"
#undef main

void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

bool accepts(std::vector<std::string> extra) {
  std::vector<std::string> args{"test", "--config", "unused", "--output", "unused",
      "--ipc-socket", "unused", "--policy-joint-hash", "1", "--kinematics-config", "unused",
      "--joint-safety-config", "unused", "--all-motors-disabled-confirmed",
      "--acknowledge-hardware-tx", "ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION",
      "--policy-actuation", "--saturate-policy-torque-to-commissioning-cap"};
  args.insert(args.end(), extra.begin(), extra.end());
  std::vector<char*> argv;
  for (auto& arg : args) argv.push_back(arg.data());
  try { (void)parse_options(static_cast<int>(argv.size()), argv.data()); return true; }
  catch (const std::runtime_error&) { return false; }
}

int main(int argc, char** argv) {
  std::vector<std::string> flags{"--duration", "120", "--xbox-500hz-actuation",
      "--xbox-500hz-actuation-acknowledgement", "ENABLE_XBOX_500HZ_TURN120",
      "--extended-policy-actuation-acknowledgement", "ENABLE_120_SECOND_XBOX_FORWARD_TEST"};
  require(accepts(flags), "valid powered experiment options refused");
  auto invalid = flags; invalid[4] = "ENABLE_XBOX_TURN120";
  require(!accepts(invalid), "old acknowledgement accepted for new powered path");
  invalid = flags; invalid[1] = "121";
  require(!accepts(invalid), "duration beyond candidate accepted");
  invalid = flags; invalid.push_back("--xbox-500hz-shadow");
  require(!accepts(invalid), "shadow can be promoted to actuation");
  invalid = flags; invalid.push_back("--measured-pose-hold");
  require(!accepts(invalid), "hold mixed with powered experiment");
  require(!accepts({"--duration", "120"}), "missing independent approval accepted");

  Motor slow, fast;
  slow.poll_rate_hz=50; fast.poll_rate_hz=500;
  slow.temperature_filter_time_constant_s=fast.temperature_filter_time_constant_s=1;
  update_temperature_filter(slow, 30, 30); update_temperature_filter(fast, 30, 30);
  for (int i=0;i<50;++i) update_temperature_filter(slow, 50, 50);
  for (int i=0;i<500;++i) update_temperature_filter(fast, 50, 50);
  require(std::abs(slow.filtered_mos_temperature-fast.filtered_mos_temperature)<1e-10,
          "temperature time constant changed with polling rate");
  for (int i=0;i<5000;++i) update_torque_rms_monitor(fast, 2);
  require(fast.short_torque_squared_samples.size()==500 &&
          fast.long_torque_squared_samples.size()==5000 &&
          fast.maximum_short_torque_rms==2 && fast.maximum_long_torque_rms==2,
          "RMS windows are not one and ten seconds at 500Hz");

  if (argc > 1) {
    const std::string directory=argv[1];
    auto baseline=load_motors(directory+"/motors.tsv");
    const auto kinematics=load_kinematics(directory+"/kinematics.tsv", baseline);
    const auto limits=load_joint_safety(directory+"/joint_safety.tsv");
    auto candidate=baseline;
    configure_xbox_500hz_schedule(candidate);
    IpcSocket old_ipc, new_ipc;
    old_ipc.has_target=new_ipc.has_target=true;
    for (std::size_t j=0;j<kJointCount;++j) {
      old_ipc.latest_target.position_rad[j]=std::clamp(0.01, limits[j].position_min, limits[j].position_max);
      old_ipc.latest_target.kp[j]=limits[j].kp_max;
      old_ipc.latest_target.kd[j]=limits[j].kd_max;
    }
    new_ipc.latest_target=old_ipc.latest_target;
    for (int tick=0;tick<100;++tick) {
      preview_final_motor_commands(baseline,old_ipc,kinematics,true);
      preview_final_motor_commands(candidate,new_ipc,kinematics,true);
      for (std::size_t j=0;j<kJointCount;++j) {
        require(protected_policy_payload(baseline[j],true)==protected_policy_payload(candidate[j],true),
                "schedule change altered quantized motor target or ankle torque");
      }
    }
    std::cout << "PASS actual31 parameter tables, held payloads and ankle mapping parity\n";
  }
  std::cout << "PASS separate powered acknowledgements and thermal window invariance; NO hardware\n";
}
