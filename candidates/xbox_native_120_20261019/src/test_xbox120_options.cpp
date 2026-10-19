#define main unused_transport_main
#include "sprite_can_shadow.cpp"
#undef main
#include <cassert>
bool accepted(const char* duration,const char* extended){
  std::vector<std::string> a={"test","--config","unused","--output","unused","--duration",duration,"--policy-actuation","--ipc-socket","unused",
    "--policy-joint-hash","1","--kinematics-config","unused","--joint-safety-config","unused",
    "--acknowledge-hardware-tx","ENABLE_NATIVE_PROTECTED_POLICY_ACTUATION","--all-motors-disabled-confirmed",
    "--extended-policy-actuation-acknowledgement",extended,"--saturate-policy-torque-to-commissioning-cap"};
  std::vector<char*> argv;for(auto& s:a)argv.push_back(s.data());
  try{parse_options(static_cast<int>(argv.size()),argv.data());return true;}catch(const std::exception&){return false;}
}
int main(int argc, char** argv){
  assert(accepted("120","ENABLE_120_SECOND_XBOX_FORWARD_TEST"));
  assert(!accepted("121","ENABLE_120_SECOND_XBOX_FORWARD_TEST"));
  assert(!accepted("120","ENABLE_60_SECOND_GROUNDED_LOWER_BODY_SIM_PD_TEST"));
  assert(!accepted("120","ENABLE_40_SECOND_GROUNDED_BALANCE_TEST"));
  assert(accepted("40","ENABLE_40_SECOND_GROUNDED_BALANCE_TEST"));
  const std::string bundle = argc > 1 ? argv[1] :
      "/home/tony/open_sprite_runtime/candidates/upper_body_pd_wrist7_20261014";
  const auto motors = load_motors(bundle + "/motors.tsv");
  const auto limits = load_joint_safety(bundle + "/joint_safety.tsv");
  const auto kinematics = load_kinematics(bundle + "/kinematics.tsv", motors);
  (void)kinematics;
  assert(motors.size() == 31 && limits.size() == 31);
  bool has_kd5 = false;
  for (const auto& limit : limits) has_kd5 |= limit.kd_max == 5.0;
  assert(has_kd5);
  std::cout << "PASS actual 31-motor safety/kinematics tables including Kd5\n";
  std::cout<<"PASS duration and exact-ack guards; no hardware accessed\n";
}
