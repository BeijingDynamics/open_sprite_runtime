#include "../src/policy_core.hpp"
#include <cassert>
#include <limits>
int main() {
  sprite_policy::Core c;c.scale.fill(0.5);c.steps=2;
  std::array<double,71> row{};row[0]=1;row[67]=-1;row[68]=0.1;
  auto obs=c.observe(row);
  for(int f=0;f<8;++f) {assert(obs[f*31]==1);assert(obs[768+f*3+2]==-1);}
  std::array<float,31> raw{};raw[0]=4;
  auto target=c.latch(raw,row);assert(target[0]==1.5);assert(c.previous[0]==3);
  row[0]=2;obs=c.observe(row);
  assert(obs[0]==1 && obs[7*31]==2 && obs[496+7*31]==3);
  target=c.latch(raw,row);assert(target[0]==2);assert(c.completed==2);
  row[0]=std::numeric_limits<double>::quiet_NaN();
  bool rejected=false;try{c.observe(row);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
  row[0]=0;raw[0]=std::numeric_limits<float>::infinity();rejected=false;
  try{c.latch(raw,row);}catch(const std::runtime_error&){rejected=true;}assert(rejected);
  sprite_policy::Core invalid;rejected=false;raw[0]=0;
  try{invalid.latch(raw,row);}catch(const std::runtime_error&){rejected=true;}assert(rejected);
}
