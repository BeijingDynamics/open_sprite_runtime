// Socketpair peers only: no PF_CAN, SPI, GPIO, model inference or motor access.
#define main sprite_transport_main
#include "../src/sprite_can_shadow.cpp"
#undef main
#include <atomic>
#include <thread>

int main() {
  std::vector<Motor> motors;
  std::vector<Socket> sockets;
  std::array<int,4> peers{};
  std::map<std::string,std::size_t> indices;
  std::map<std::pair<std::string,int>,std::size_t> feedback;
  std::array<std::array<int,9>,4> disables{};
  for(int b=0;b<4;++b) {
    int pair[2];
    if(socketpair(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK,0,pair))return 1;
    std::string bus="fake"+std::to_string(b);
    indices[bus]=sockets.size(); sockets.push_back({bus,pair[0]}); peers[b]=pair[1];
    for(int id=1;id<=(b==3?7:8);++id) {
      Motor m; m.name=bus+"_"+std::to_string(id);m.interface=bus;
      m.can_id=id;m.master_id=id+16;m.poll_rate_hz=50;m.poll_phase=1;
      m.position_min=-12.5;m.position_max=12.5;m.velocity_min=-20;m.velocity_max=20;
      m.torque_min=-28;m.torque_max=28;m.mos_temperature_limit=100;m.rotor_temperature_limit=80;
      m.temperature_filter_time_constant_s=1;
      feedback[{bus,m.master_id}]=motors.size();motors.push_back(m);
    }
  }
  configure_xbox_500hz_schedule(motors);
  std::atomic<bool> done{false}, enabled{false};
  std::thread hardware([&] {
    while(!done.load()) {
      for(int b=0;b<4;++b) {
        canfd_frame f{};
        while(read(peers[b],&f,sizeof(f))==sizeof(f)) {
          const int id=static_cast<int>(f.can_id);
          if(id<1||id>8)continue;
          if(f.data[0]==255 && f.data[7]==0xFC)enabled=true;
          if(f.data[0]==255 && f.data[7]==0xFD)++disables[b][id];
          f.can_id=static_cast<canid_t>(id+16);f.len=8;f.flags=CANFD_BRS;
          f.data[0]=static_cast<unsigned char>(id);f.data[1]=0x7f;f.data[2]=0xff;
          f.data[3]=0x7f;f.data[4]=0xf7;f.data[5]=0xff;f.data[6]=30;f.data[7]=30;
          (void)write(peers[b],&f,sizeof(f));
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  });
  Options options;options.xbox_500hz_actuation=true;options.duration_s=120;
  options.output="/tmp/sprite_xbox500_shutdown_"+std::to_string(getpid())+".json";
  NativeKinematics k;IpcSocket ipc;
  // Preparation interruption must never enable, and must still verify all31 off.
  g_shutdown_requested=1;
  std::ostringstream sink;auto* previous=std::cout.rdbuf(sink.rdbuf());
  const int rc=run_native_protected_policy(options,sockets,motors,indices,feedback,k,{},ipc,0,false);
  std::cout.rdbuf(previous);
  done=true;hardware.join();g_shutdown_requested=0;
  bool ok=rc==2 && !enabled.load();
  for(std::size_t i=0;i<motors.size();++i) {
    const auto& m=motors[i];
    ok=ok && m.final_disabled && disables[indices[m.interface]][m.can_id]>=3;
  }
  for(auto& s:sockets)close(s.fd);
  for(int fd:peers)close(fd);
  unlink(options.output.c_str());
  if(!ok)throw std::runtime_error("interrupted candidate did not attempt/verify all31 disabled");
  std::cout<<"PASS startup interruption: zero enables, three disables and verified feedback for all31; socketpair only\n";
}
