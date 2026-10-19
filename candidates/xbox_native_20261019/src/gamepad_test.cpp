#include "policy_gamepad.hpp"
#include <cassert>
#include <iostream>
int main(){
  using namespace sprite_policy;
  GamepadPermission p;
  p.event(1,0,true);p.event(2,0,true);p.event(5,1,true);
  assert(!p.snapshot(1).permitted);
  p.event(5,-1,false);p.event(5,1,false);assert(p.snapshot(1).permitted);
  p.event(5,.7,false);assert(p.snapshot(1).permitted);
  p.event(5,.5,false);assert(!p.snapshot(1).permitted);
  p.disconnect();p.event(1,0,true);p.event(2,0,true);p.event(5,1,true);assert(!p.snapshot(1).permitted);
  p.event(1,-1,false);p.event(5,-1,false);p.event(5,1,false);assert(!p.snapshot(1).permitted);
  p.event(5,-1,false);p.event(1,0,false);p.event(5,1,false);assert(p.snapshot(1).permitted);
  GamepadCommand c(GamepadConfig{});GamepadSnapshot s{-1,0,true,true,1,0};
  std::array<double,3> v{};std::uint64_t t=1;
  for(int i=0;i<200;++i){t+=20000000;s.stamp=t;v=c.sample(s,t);assert(v[0]<=.100001 && v[1]==0);}
  assert(v[0]>.099);
  s.connected=false;for(int i=0;i<60;++i){t+=20000000;v=c.sample(s,t);}assert(v[0]==0);
  s.connected=true;s.forward=1;s.turn=-1;
  for(int i=0;i<10;++i){t+=20000000;s.stamp=t;v=c.sample(s,t);}assert(v[0]==0 && v[2]==0);
  ++s.revocations;
  for(int i=0;i<100;++i){t+=20000000;s.stamp=t;v=c.sample(s,t);}
  assert(v[0]==-.05 && v[2]==.20);
  assert(c.last_allowed && c.last_healthy && c.target_vx==-.05 && c.target_wz==.20);
  t+=20000000;s.stamp=t;v=c.sample(s,t,false);
  assert(!c.last_allowed && c.target_vx==0 && c.target_wz==0);
  assert(p.snapshot(t).lt==1);
  for(int i=0;i<100;++i){t+=20000000;v=c.sample(s,t);}assert(v[0]==0 && v[2]==0);
  for(int i=0;i<10;++i){t+=20000000;s.stamp=t;v=c.sample(s,t);}assert(v[0]==0 && v[2]==0);
  bool threw=false;try{c.sample(s,t);}catch(...){threw=true;}assert(threw);
  GamepadConfig forward_cfg;forward_cfg.forward_only=true;
  GamepadCommand forward_only(forward_cfg);
  s={1,-1,true,true,t,0};
  for(int i=0;i<150;++i){t+=20000000;s.stamp=t;v=forward_only.sample(s,t);assert(v[0]==0 && v[2]==0);}
  s.forward=-1;
  for(int i=0;i<150;++i){t+=20000000;s.stamp=t;v=forward_only.sample(s,t);assert(v[0]>=0 && v[0]<=.1 && v[2]==0);}
  assert(v[0]>.099);
  std::cout<<"PASS startup, hysteresis, rearm, disconnect, ramps, polarity, bounds, stale heartbeat, clock\n";
}
