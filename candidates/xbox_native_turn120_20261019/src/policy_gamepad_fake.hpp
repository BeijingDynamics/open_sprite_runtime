#pragma once
#ifndef SPRITE_GAMEPAD_FAKE_PEERS
#error Test-only input source
#endif
#include "policy_gamepad.hpp"
#include <chrono>
#include <cstdlib>
namespace sprite_policy {
inline std::uint64_t gamepad_now(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
// Synthetic input for fake Unix peers only; never opens joystick or CAN devices.
class LinuxGamepad {
  std::uint64_t start_=gamepad_now();
public:
  explicit LinuxGamepad(const char*){}
  GamepadSnapshot snapshot() const {
    const auto now=gamepad_now();const double t=(now-start_)*1e-9;
    GamepadSnapshot s{-1,-1,true,true,now,static_cast<std::uint64_t>(t>=12?1:0),1};
    if(t>=18 && t<21){s.permitted=false;s.lt=-1;}
    if(t>=21){s.forward=1;s.revocations=2;}
    if(t>=24){s.forward=-1;s.connected=false;}
    if(t>=26){s.connected=true;} // Must remain latched until another release.
    if(t>=28){s.revocations=3;}
    if(t>=30){s.turn=1;}
    if(std::getenv("SPRITE_FAKE_DISABLE") && t>=2)s.disable_requested=true;
    return s;
  }
};
}
