#include "../src/first_slot_wait.hpp"
#include "../src/transport_slot_timing.hpp"
#include <iostream>

void require(bool ok) {
  if (!ok) throw std::runtime_error("first-slot wait test failed");
}

int main() {
  constexpr std::int64_t deadline = 100'000'000;
  for (const std::int64_t delay : {0LL, 300'614LL, 1'600'000LL}) {
    std::int64_t t = 0, sleep_at = 0;
    int sleeps = 0;
    wait_first_slot(deadline, [&] { const auto value = t; t += 100; return value; },
                    [&](auto at) { sleep_at = at; t = at + delay; ++sleeps; },
                    [] { return false; });
    require(sleeps == 1 && sleep_at == deadline-kFirstSlotWakeLeadNs);
    require(t >= deadline);
    if (delay < kFirstSlotWakeLeadNs) require(t-deadline < 1000);
    else {
      TransportSlotTiming timing;
      timing.record(0, deadline, t, t, t, t);
      require(timing.count == 1 && timing.skipped_slots == 1);
    }
  }
  // Already late: no sleep and no resetting the epoch.
  std::int64_t t = deadline+600'000;
  wait_first_slot(deadline, [&] { return t; }, [](auto) { require(false); }, [] { return false; });
  require(t == deadline+600'000);
  bool cancelled = false;
  try {
    wait_first_slot(deadline, [] { return 0LL; }, [](auto) { require(false); }, [] { return true; });
  } catch (const std::runtime_error&) { cancelled = true; }
  require(cancelled);
  int calls = 0;
  cancelled = false;
  try {
    wait_first_slot(deadline, [] { return deadline-100LL; }, [](auto) { require(false); },
                    [&] { return ++calls == 3; });
  } catch (const std::runtime_error&) { cancelled = true; }
  require(cancelled);
  std::cout << "FIRST_SLOT_WAIT_OK unchanged deadline, budget, late-wake and cancellation checks\n";
}
