#include "../src/policy_command_schedule.hpp"
#include <cassert>
#include <iostream>
#include <iomanip>
#include <limits>
int main() {
  using namespace sprite_policy;
  assert(walk40_countdown(34.999)==0);
  assert(walk40_countdown(35)==5);
  assert(walk40_countdown(39.999)==1);
  assert(walk40_countdown(40)==0);
  for (double speed : {0.05, 0.10}) {
    CommandSchedule schedule(speed==0.05 ? "stand-walk-stop-40s" : "stand-walk010-stop-40s");
    // Absolute startup time includes calibration; the first state is time zero.
    for (std::uint64_t i=0;i<=40000;++i) {
      const auto value=schedule.sample(15000000000ULL+i*1000000ULL);
      std::cout<<std::setprecision(17)<<speed<<' '<<i<<' '<<value.vx<<' '<<value.stage<<'\n';
    }
    bool rejected=false;
    try { schedule.sample(15000000000ULL); } catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);
  }
  for(double invalid : {-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected=false;
    try { sample_walk40(invalid,.1); } catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);
  }
}
