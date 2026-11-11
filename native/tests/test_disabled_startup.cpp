#include "../src/disabled_startup.hpp"
#include <iostream>
#include <string>

void require(bool ok) { if (!ok) throw std::runtime_error("startup test failed"); }

int main() {
  // First cold send is retained, followed by a full clean window.
  DisabledStartup s;
  std::int64_t clock = 0;
  std::size_t sends = 0;
  s.run([&]{return clock;}, [&](auto t){clock=t;},
        [&](auto slot, auto& tx, auto& rx){
          ++sends; clock += slot == 0 ? 525'000 : 100'000; tx=rx=clock;
        }, [](auto){return true;}, []{return false;});
  require(s.slots == 400 && sends == 400 && s.clean_slots == 399);
  require(s.timing.count == 1 && s.timing.first_slot.total_ns == 525'000);
  require(!s.passed); // Final disabled feedback must still be verified.
  TransportSlotTiming active;
  active.record(0,0,0,525'000,525'000,525'000);
  require(active.count == 1); // No special exemption in active timing.

  // A late wake sends nothing and schedules a future slot, not a catch-up burst.
  DisabledStartup late;
  clock=0; sends=0;
  late.run([&]{return clock;}, [&](auto t){clock=t+(late.slots==0 ? 700'000 : 0);},
           [&](auto,auto& tx,auto& rx){++sends;clock+=100'000;tx=rx=clock;},
           [](auto){return true;}, []{return false;});
  require(late.timing.skipped_slots == 1 && sends == late.slots-1);

  for (int mode=0;mode<4;++mode) {
    DisabledStartup bad; clock=0; bool rejected=false;
    try {
      bad.run([&]{return clock;}, [&](auto t){clock=t;},
          [&](auto,auto& tx,auto& rx){
            if (mode==3) throw std::runtime_error("invalid feedback");
            clock += mode==0 ? 2'000'000 : (mode==1 ? 600'000 : 100'000);
            tx=rx=clock;
          }, [](auto){return false;}, [&]{return mode==2;});
    } catch (const std::runtime_error&) { rejected=true; }
    require(rejected && !bad.passed);
  }
  // A later timing failure resets the consecutive clean-window requirement.
  DisabledStartup reset;clock=0;
  reset.run([&]{return clock;},[&](auto t){clock=t;},
      [&](auto slot,auto& tx,auto& rx){clock+=slot==350 ? 600'000:100'000;tx=rx=clock;},
      [](auto){return true;},[]{return false;});
  require(reset.slots==552 && reset.clean_slots==201 && reset.timing.count==1);
  std::cout << "disabled startup tests passed; no hardware\n";
}
