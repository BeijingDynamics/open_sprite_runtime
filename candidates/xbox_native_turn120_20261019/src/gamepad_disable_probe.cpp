#include "policy_gamepad_linux.hpp"
#include <iostream>
int main(int argc,char** argv){
  if(argc!=2)return 2;
  sprite_policy::LinuxGamepad pad(argv[1]);
  const auto begin=sprite_policy::gamepad_now();
  std::cout<<"NO CAN. Release LB/RB first, then hold BOTH continuously for3s. Timeout20s."<<std::endl;
  while(sprite_policy::gamepad_now()-begin<20000000000ULL){
    if(pad.snapshot().disable_requested){std::cout<<"DISABLE_COMBO_CONFIRMED_NO_CAN"<<std::endl;return 0;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cerr<<"Combo not confirmed; no hardware accessed\n";return 1;
}
