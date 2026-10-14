#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace sprite_policy {
struct ScheduledCommand { double vx; const char* stage; };
inline int walk40_countdown(double elapsed) {
  if (!std::isfinite(elapsed) || elapsed < 0)
    throw std::runtime_error("invalid countdown time");
  return elapsed >= 35 && elapsed < 40 ? static_cast<int>(std::ceil(40-elapsed)) : 0;
}
inline ScheduledCommand sample_walk40(double elapsed, double speed) {
  if (!std::isfinite(elapsed) || elapsed < 0 || (speed != 0.05 && speed != 0.10))
    throw std::runtime_error("invalid commissioning schedule");
  if (elapsed < 10) return {0, "stand"};
  if (elapsed < 13) {
    const double x = (elapsed - 10) / 3;
    return {speed * x * x * (3 - 2 * x), "ramp_forward"};
  }
  if (elapsed < 27) return {speed, "forward"};
  if (elapsed < 30) {
    const double x = (elapsed - 27) / 3;
    return {speed * (1 - x * x * (3 - 2 * x)), "ramp_stop"};
  }
  return {0, "final_stand"};
}
class CommandSchedule {
  double speed_;
  std::uint64_t first_=0, last_=0;
  bool started_=false;
  double elapsed_=0;
 public:
  explicit CommandSchedule(const std::string& profile) {
    if (profile == "stand-walk010-stop-40s") speed_=0.10;
    else if (profile == "stand-walk-stop-40s") speed_=0.05;
    else throw std::runtime_error("unsupported command profile");
  }
  ScheduledCommand sample(std::uint64_t state_stamp) {
    if (started_ && state_stamp <= last_)
      throw std::runtime_error("command schedule timestamp not increasing");
    if (!started_) { first_=state_stamp; started_=true; }
    last_=state_stamp;
    elapsed_=static_cast<double>(state_stamp-first_)/1e9;
    return sample_walk40(elapsed_, speed_);
  }
  int countdown() const { return walk40_countdown(elapsed_); }
};
}
