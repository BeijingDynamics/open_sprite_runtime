#include <cstdlib>
#include <new>
static bool count_allocations = false;
static std::size_t allocations = 0;
void* operator new(std::size_t n) {
  if (count_allocations) ++allocations;
  if (void* p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return ::operator new(n); }
// Keep the replacement allocation boundary visible to optimized builds.
[[gnu::noinline]] void operator delete(void* p) noexcept { std::free(p); }
[[gnu::noinline]] void operator delete[](void* p) noexcept { std::free(p); }
[[gnu::noinline]] void operator delete(void* p, std::size_t) noexcept { std::free(p); }
[[gnu::noinline]] void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

#define main sprite_transport_main
#include "../src/sprite_can_shadow.cpp"
#undef main

void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

int main() {
  Motor old, fixed;
  old.poll_rate_hz = fixed.poll_rate_hz = 500;
  fixed.fixed_rms_enabled = true;
  fixed.fixed_short_rms.prepare(500); fixed.fixed_long_rms.prepare(5000);
  std::size_t old_allocations = 0;
  for (int reset=0; reset<2; ++reset) {
    reset_torque_rms_monitor(old); reset_torque_rms_monitor(fixed);
    for (int i=0;i<30000;++i) {
      const double torque = 4*std::sin(i*0.013)+0.1*(i%7);
      allocations=0; count_allocations=true;
      update_torque_rms_monitor(old, torque);
      count_allocations=false; old_allocations+=allocations;
      allocations=0; count_allocations=true;
      update_torque_rms_monitor(fixed, torque);
      count_allocations=false;
      require(allocations==0, "fixed RMS allocated in update");
      require(old.maximum_short_torque_rms==fixed.maximum_short_torque_rms &&
              old.maximum_long_torque_rms==fixed.maximum_long_torque_rms,
              "RMS differs from original window, including reset/wrap");
    }
  }
  require(old_allocations>0, "allocation instrumentation not working");

  std::vector<Socket> sockets;
  std::array<int,4> peers{};
  std::vector<Motor> motors;
  std::map<std::pair<std::string,int>,std::size_t> feedback_map;
  for (int bus=0;bus<4;++bus) {
    int fds[2];
    require(socketpair(AF_UNIX, SOCK_DGRAM|SOCK_NONBLOCK,0,fds)==0,"socketpair failed");
    sockets.push_back({"kcan"+std::to_string(bus+1),fds[0]}); peers[bus]=fds[1];
    for (int id=1;id<=(bus==3 ? 7 : 8);++id) {
      Motor m;
      m.name="fake_motor_"+std::to_string(motors.size());m.interface=sockets.back().interface;
      m.can_id=id;m.master_id=id+16;m.poll_rate_hz=50;m.poll_phase=1;
      m.position_min=-12.5;m.position_max=12.5;m.velocity_min=-50;m.velocity_max=50;
      m.torque_min=-10;m.torque_max=10;m.hard_position_min=-3;m.hard_position_max=3;
      m.deployment_velocity_max=5;m.feedback_torque_cap=10;
      m.mos_temperature_limit=100;m.rotor_temperature_limit=80;
      m.filtered_mos_temperature_limit=90;m.filtered_rotor_temperature_limit=70;
      m.temperature_filter_time_constant_s=2;
      feedback_map[{m.interface,m.master_id}]=motors.size();motors.push_back(m);
    }
  }
  configure_xbox_500hz_schedule(motors);
  ReceiveTiming timing;
  auto queue = [&](int status, bool overtemperature=false) {
    for (const auto& m : motors) {
      canfd_frame f{}; f.can_id=m.master_id;f.len=8;f.flags=CANFD_BRS;
      f.data[0]=(status<<4)|m.can_id;
      // Midpoint codes: near-zero position, velocity and torque.
      f.data[1]=0x80;f.data[2]=0;f.data[3]=0x80;f.data[4]=0x08;f.data[5]=0;
      f.data[6]=overtemperature ? 120 : 30;f.data[7]=30;
      require(write(peers[m.interface.back()-'1'],&f,sizeof(f))==sizeof(f),"fake write failed");
    }
  };
  // Disabled shadow and enabled processing both run the same monitors.
  for (int cycle=0;cycle<12000;++cycle) {
    const bool shadow = cycle%2==0;
    queue(shadow ? 0 : 1);
    allocations=0;count_allocations=true;
    drain_policy_feedback(sockets,motors,feedback_map,shadow ? 0 : 1,true,64,shadow,&timing,cycle);
    count_allocations=false;
    require(allocations==0,"feedback handling allocated memory");
  }
  require(timing.total_frames==31*12000 && timing.total_rms_updates==31*12000,
          "shadow skipped RMS or frames");
  queue(1);
  bool rejected=false;
  try {drain_policy_feedback(sockets,motors,feedback_map,0,true,64,true);}
  catch(const std::runtime_error&) {rejected=true;}
  require(rejected,"shadow accepted enabled drive");
  // Drain remaining fake replies before checking independent temperature gate.
  drain_policy_feedback(sockets,motors,feedback_map,-1,false,64);
  for (int i=0;i<9;++i) queue(0);
  rejected=false;
  try {drain_policy_feedback(sockets,motors,feedback_map,0,true,64,true);}
  catch(const std::runtime_error& e) {
    rejected=std::string(e.what())=="CAN receive work budget exceeded";
  }
  require(rejected,"receive frame budget was weakened");
  drain_policy_feedback(sockets,motors,feedback_map,0,false);
  queue(0,true); rejected=false;
  try {drain_policy_feedback(sockets,motors,feedback_map,0,true,64,true);}
  catch(const std::runtime_error&) {rejected=true;}
  require(rejected,"shadow bypassed temperature gate");
  for (auto s:sockets) close(s.fd);
  for (auto fd:peers) close(fd);
  std::cout << "PASS_NO_HARDWARE fixed RMS exact parity; old allocations=" << old_allocations
            << "; fixed feedback allocations=0; 372000 fake frames and monitor updates\n";
  timing.json_fields(std::cout);
}
