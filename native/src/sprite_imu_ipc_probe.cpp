#include "policy_imu_client.hpp"
#include <iostream>

// Transport validation only: no CAN, actor or control target output.
int main(int argc,char** argv) {
    try {
        if(argc!=2) throw std::runtime_error("usage: sprite_imu_ipc_probe SOCKET");
        sprite_policy::NativeImuClient client(argv[1]);
        sprite_policy::ImuSnapshot sample;
        const auto deadline=sprite_policy::monotonic_ns()+100000000;
        while (!client.snapshot(sample)) {
            if(sprite_policy::monotonic_ns()>deadline) throw std::runtime_error("IMU startup timeout");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        for(int i=0;i<25;++i) {
            if(!client.snapshot(sample)) throw std::runtime_error("IMU snapshot missing");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        // A stalled inference thread must not stop the independent reader.
        const auto before=sample.sequence;
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        if(!client.snapshot(sample) || sample.sequence<=before)
            throw std::runtime_error("IMU reader did not progress independently");
        std::cout<<"IMU_IPC_OK hardware_tx=0 sequence="<<sample.sequence<<'\n';
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
