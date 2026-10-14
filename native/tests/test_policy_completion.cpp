#include "../src/policy_state_ipc.hpp"
#include <iostream>
int main(int argc,char** argv) {
  try {
    if(argc<2 || argc>4) throw std::runtime_error("fake socket required");
    sprite_policy::PolicyStateChannel channel(argv[1],1,2,100000000);
    if(argc>=3) {
      auto state=channel.receive([](){return std::uint64_t(1000000000);});
      sprite_policy::ImpedanceTarget target;
      channel.send(target,state,1000000000);
    }
    channel.await_controller_close(1000,argc==4);
    std::cout<<"TRANSPORT_CLOSED_NOT_HARDWARE_SUCCESS\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
