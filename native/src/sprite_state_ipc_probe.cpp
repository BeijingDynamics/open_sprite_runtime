#include "policy_state_ipc.hpp"
#include <chrono>
#include <thread>
#include <iostream>
int main(int argc,char** argv) {
    try {
        if(argc!=3)throw std::runtime_error("usage: sprite_state_ipc_probe FAKE_SOCKET DELAY_MS");
        std::array<std::string,31> motors,joints;
        for(int i=0;i<31;++i){motors[i]="motor_"+std::to_string(i);joints[i]="joint_"+std::to_string(i);}
        // Test-only name hashes deliberately do not match a deployed robot.
        sprite_policy::PolicyStateChannel channel(argv[1],sprite_policy::name_hash(motors),
                                                 sprite_policy::name_hash(joints),100000000);
        auto clock=[](){return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());};
        for(int i=0;i<3;++i) {
            auto s=channel.receive(clock);
            sprite_policy::ImpedanceTarget target;
            target.position=s.position;target.velocity=s.velocity;
            for(std::size_t j=0;j<31;++j) {
                target.kp[j]=j+.5;target.kd[j]=j*.01;target.feedforward[j]=-static_cast<double>(j)*.001;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(std::stoi(argv[2])));
            channel.send(target,s,clock());
        }
        std::cout<<"STATE_IPC_OK hardware_tx=0\n";return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
