#define main sprite_transport_main
#include "../src/sprite_can_shadow.cpp"
#undef main

void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

struct Fixture {
  std::vector<Motor> motors = std::vector<Motor>(kJointCount);
  NativeKinematics kinematics;
  IpcSocket ipc;
  Fixture() {
    ipc.has_target = true;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      auto& m = motors[i];
      m.name = "test_motor_" + std::to_string(i);
      m.position_min = -12.5; m.position_max = 12.5;
      m.velocity_min = -20; m.velocity_max = 20;
      m.torque_min = -28; m.torque_max = 28;
      m.soft_position_min = -1; m.soft_position_max = 1;
      m.hard_position_min = -1.2; m.hard_position_max = 1.2;
      m.deployment_velocity_max = 10;
      m.commissioning_torque_cap = 28;
      m.mechanical_peak_torque = 40;
      kinematics.joint_to_motor[i][i] = 1;
      kinematics.motor_to_joint[i][i] = 1;
      kinematics.joint_effort_limit[i] = 40;
    }
  }
  void preview(bool bounded) {
    preview_final_motor_commands(motors, ipc, kinematics, bounded);
  }
};

int main() {
  Fixture quantized;
  auto& qm = quantized.motors[0];
  qm.velocity_min = -50; qm.velocity_max = 50;
  qm.torque_min = -10; qm.torque_max = 10;
  qm.commissioning_torque_cap = 1;
  qm.last_position = decode_uint(encode_uint(-0.5, -12.5, 12.5, 16), -12.5, 12.5, 16);
  qm.last_velocity = decode_uint(encode_uint(0, -50, 50, 12), -50, 50, 12);
  qm.preview_initialized = true;
  qm.preview_kp = 7; qm.preview_kd = 3;
  qm.preview_position = qm.last_position + (-0.98 + 3 * qm.last_velocity) / 7;
  bool quantization_rejected = false;
  try { (void)protected_policy_payload(qm); }
  catch (const std::runtime_error&) { quantization_rejected = true; }
  require(quantization_rejected, "must reproduce historical 7/3 quantization failure");
  (void)protected_policy_payload(qm, true);
  require(qm.maximum_abs_commanded_torque <= 1, "encoded correction exceeded cap");
  // Cover both signs, changed feedback after preview, and several gain/cap ranges.
  for (double cap : {0.08, 0.35, 1.0, 3.0, 10.0}) {
    for (double kd : {0.45, 0.5, 3.0, 5.0}) {
      for (int sign : {-1, 1}) {
        qm.commissioning_torque_cap = cap;
        qm.preview_kd = kd;
        qm.preview_position = -0.6;
        qm.last_velocity = sign * 0.75;
        qm.maximum_abs_commanded_torque = 0;
        (void)protected_policy_payload(qm, true);
        require(qm.maximum_abs_commanded_torque <= cap, "sweep encoded cap");
      }
    }
  }
  qm.commissioning_torque_cap = 0.00001;
  qm.preview_kp = 0; qm.preview_kd = 0;
  bool infeasible_rejected = false;
  try { (void)protected_policy_payload(qm, true); }
  catch (const std::runtime_error&) { infeasible_rejected = true; }
  require(infeasible_rejected, "unrepresentable zero torque must fail closed");

  // Reproduce the stationary knee's full-gain preview exceeding protocol TMAX.
  Fixture knee;
  knee.motors[0].last_position = -0.144;
  knee.ipc.latest_target.position_rad[0] = -0.367;
  knee.ipc.latest_target.kp[0] = 130;
  bool rejected = false;
  try { knee.preview(false); } catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "unbounded knee should reproduce failure");
  knee.ipc.motor_preview_count = 0;
  knee.preview(true);
  (void)protected_policy_payload(knee.motors[0]);
  require(knee.motors[0].maximum_abs_commanded_torque <= 28, "knee cap");

  // Strong damping at a joint limit must not move the target outside that limit.
  Fixture elbow;
  auto& m = elbow.motors[0];
  m.soft_position_min = -0.125;
  m.last_position = -0.05; m.last_velocity = -10;
  m.commissioning_torque_cap = 0.35;
  elbow.ipc.latest_target.position_rad[0] = -0.125;
  elbow.ipc.latest_target.kp[0] = 0.8882644;
  elbow.ipc.latest_target.kd[0] = 0.0565487;
  elbow.preview(true);
  require(m.preview_position == -0.125, "elbow target moved outside envelope");
  require(m.preview_kp < 0.8882644, "elbow impedance not reduced");
  (void)protected_policy_payload(m);
  require(m.maximum_abs_commanded_torque <= 0.35, "elbow quantized cap");

  Fixture invalid;
  invalid.ipc.latest_target.position_rad[0] = 1.3;
  rejected = false;
  try { invalid.preview(true); } catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "bounded preview must retain position guard");

  Fixture ankle;
  ankle.motors[0].name = "left_ankle_motor_a";
  ankle.kinematics.joint_names[0] = "left_ankle_pitch_joint";
  ankle.motors[0].commissioning_torque_cap = 10;
  ankle.ipc.latest_target.position_rad[0] = 0.5;
  ankle.ipc.latest_target.kp[0] = 80;
  ankle.preview(true);
  require(std::abs(ankle.motors[0].preview_feedforward_torque - 9.8) < 1e-9,
          "ankle feedforward cap");
  (void)protected_policy_payload(ankle.motors[0]);
  std::cout << "PASS bounded knee, elbow damping, mechanical envelope, ankle, MIT quantization\n";
}
