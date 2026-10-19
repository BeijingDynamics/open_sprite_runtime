#include "policy_gamepad_linux.hpp"
#include <iostream>
#include <csignal>
volatile std::sig_atomic_t stopped=0;
int main(int argc,char** argv) {
  if(argc!=5){std::cerr<<"INPUT ONLY NO CAN: gamepad_probe DEVICE SECONDS FORWARD_SIGN TURN_SIGN\n";return 2;}
  try {
    const int seconds=std::stoi(argv[2]);if(seconds<1||seconds>300)return 2;
    sprite_policy::GamepadConfig cfg;cfg.forward_sign=std::stoi(argv[3]);cfg.turn_sign=std::stoi(argv[4]);
    sprite_policy::GamepadCommand command(cfg);sprite_policy::LinuxGamepad device(argv[1]);
    std::signal(SIGINT,[](int){stopped=1;});
    std::cout<<"NO CAN / NO POLICY. Center sticks; squeeze and release LT; press LT again to unlock.\n";
    const auto end=sprite_policy::gamepad_now()+std::uint64_t(seconds)*1000000000;
    while(!stopped && sprite_policy::gamepad_now()<end) {
      const auto s=device.snapshot();const auto stamp=sprite_policy::gamepad_now();const auto c=command.sample(s,stamp);
      std::cout<<"stamp_ns="<<stamp<<" connected="<<s.connected<<" LT_permission="<<s.permitted<<" axis1="<<s.forward<<" axis2="<<s.turn<<" vx="<<c[0]<<" wz="<<c[2]<<std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
