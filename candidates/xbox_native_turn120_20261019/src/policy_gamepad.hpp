#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace sprite_policy {
// Input-only candidate limits. Physical reverse/turn operation is not qualified.
struct GamepadConfig {
  bool forward_only=false;
  bool no_reverse=false;
  int forward_sign=-1,turn_sign=-1;
  double forward=.10,reverse=.05,turn=.20,deadzone=.12;
  double acceleration=.05,deceleration=.10,yaw_acceleration=.20;
  void validate() const {
    if((forward_sign!=1 && forward_sign!=-1)||(turn_sign!=1 && turn_sign!=-1))
      throw std::runtime_error("axis polarity must be +1 or -1");
    for(double v:{forward,reverse,turn,deadzone,acceleration,deceleration,yaw_acceleration})
      if(!std::isfinite(v)||v<=0)throw std::runtime_error("invalid gamepad limits");
    if(deadzone>=1 || forward>.10 || reverse>.05 || turn>.20)
      throw std::runtime_error("outside candidate command envelope");
  }
};
struct GamepadSnapshot {
  double forward=0,turn=0;
  bool connected=false,permitted=false;
  std::uint64_t stamp=0,revocations=0;
  double lt=0;
  bool disable_requested=false;
};
class GamepadDisableCombo {
  bool lb_=false,rb_=false,lb_known_=false,rb_known_=false,ready_=false,latched_=false;
  std::uint64_t since_=0,last_=0;
public:
  void disconnect(){lb_=rb_=lb_known_=rb_known_=ready_=false;since_=0;}
  void event(bool left,bool down,bool initial){
    if(left){lb_=down;lb_known_=true;}else{rb_=down;rb_known_=true;}
    if(!lb_ || !rb_)since_=0;
    if(lb_known_ && rb_known_ && !lb_ && !rb_)ready_=true;
    if(initial && (lb_ || rb_))ready_=false;
  }
  bool sample(std::uint64_t now){
    if(last_ && (now<last_ || now-last_>100000000ULL))since_=0;
    last_=now;
    if(ready_ && lb_ && rb_){
      if(!since_)since_=now;
      if(now>=since_ && now-since_>=3000000000ULL)latched_=true;
    }else since_=0;
    return latched_;
  }
};
class GamepadPermission {
  std::array<double,8> axes_{};
  std::array<bool,8> known_{};
  bool release_seen_=false,held_=false,permitted_=false;
  std::uint64_t revocations_=0;
public:
  void disconnect(){axes_={};known_={};release_seen_=held_=permitted_=false;++revocations_;}
  void event(unsigned axis,double value,bool initial) {
    if(axis>=axes_.size())return;
    if(!std::isfinite(value)||std::abs(value)>1.001)throw std::runtime_error("invalid axis");
    axes_[axis]=std::clamp(value,-1.,1.);known_[axis]=true;
    if(!known_[1]||!known_[2]||!known_[5])return;
    const bool centered=std::abs(axes_[1])<=.12 && std::abs(axes_[2])<=.12;
    // Initialization events never satisfy the deliberate release/press sequence.
    if(initial)return;
    if(axes_[5]<.60) {
      if(held_||permitted_)++revocations_;
      held_=permitted_=false;
      if(centered)release_seen_=true;
    }
    if(axis==5 && axes_[5]>.90 && !held_) {
      held_=true;permitted_=release_seen_&&centered;release_seen_=false;
    }
  }
  GamepadSnapshot snapshot(std::uint64_t stamp) const {
    return {axes_[1],axes_[2],true,permitted_,stamp,revocations_,axes_[5]};
  }
};
class GamepadCommand {
  GamepadConfig cfg_; double vx_=0,wz_=0;std::uint64_t last_=0;
  bool health_latched_=false;std::uint64_t health_epoch_=0;
  static double approach(double a,double b,double step){return a+std::clamp(b-a,-step,step);}
  double axis(double x) const {
    if(std::abs(x)<=cfg_.deadzone)return 0;
    return std::copysign((std::min(std::abs(x),1.)-cfg_.deadzone)/(1.-cfg_.deadzone),x);
  }
public:
  explicit GamepadCommand(GamepadConfig c):cfg_(c){cfg_.validate();}
  bool last_healthy=false,last_allowed=false;
  double target_vx=0,target_wz=0;
  std::array<double,3> sample(const GamepadSnapshot& s,std::uint64_t now,bool window=true) {
    if(last_ && now<=last_)throw std::runtime_error("gamepad clock regression");
    const double dt=last_?std::min((now-last_)*1e-9,.04):0.;last_=now;
    const bool healthy=s.connected && now>=s.stamp && now-s.stamp<=100000000;
    if(!healthy && !health_latched_){health_latched_=true;health_epoch_=s.revocations;}
    if(healthy && health_latched_ && s.revocations!=health_epoch_)health_latched_=false;
    const bool allowed=window && healthy && !health_latched_ && s.permitted;
    double v=0,w=0;
    if(allowed){const auto x=axis(s.forward)*cfg_.forward_sign;v=x*(x>=0?cfg_.forward:cfg_.reverse);w=axis(s.turn)*cfg_.turn_sign*cfg_.turn;}
    if(cfg_.forward_only){v=std::max(0.,v);w=0;}
    if(cfg_.no_reverse)v=std::max(0.,v);
    last_healthy=healthy;last_allowed=allowed;target_vx=v;target_wz=w;
    vx_=approach(vx_,v,(allowed && std::abs(v)>std::abs(vx_)?cfg_.acceleration:cfg_.deceleration)*dt);
    wz_=approach(wz_,w,cfg_.yaw_acceleration*dt);
    return {vx_,0,wz_};
  }
};
}
