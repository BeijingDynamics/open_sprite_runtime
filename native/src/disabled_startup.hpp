#pragma once

#include "transport_slot_timing.hpp"
#include <stdexcept>

// Only for disabled, zero-gain preparation. Active timing is unchanged.
struct DisabledStartup {
  static constexpr std::int64_t phase_budget_ns = 2'000'000'000;
  static constexpr std::int64_t slot_budget_ns = 2'000'000;
  static constexpr std::size_t minimum_slots = 400, required_clean_slots = 200;
  TransportSlotTiming timing;
  std::size_t slots = 0, clean_slots = 0;
  std::int64_t elapsed_ns = 0;
  bool passed = false;

  template<class Clock, class Sleep, class Step, class Ready, class Cancel>
  void run(Clock now, Sleep sleep, Step step, Ready ready, Cancel cancelled) {
    const auto start = now();
    auto deadline = start + 500'000;
    for (std::size_t slot = 0; ; ++slot) {
      if (cancelled()) throw std::runtime_error("disabled startup cancelled");
      if (now()-start >= phase_budget_ns)
        throw std::runtime_error("disabled startup failed to settle within 2 seconds");
      sleep(deadline);
      if (cancelled()) throw std::runtime_error("disabled startup cancelled");
      const auto woke = now();
      auto tx_end = woke, rx_end = woke;
      // Never send catch-up bursts, even while disabled.
      const bool expired = woke-deadline >= TransportSlotTiming::budget_ns;
      if (!expired) step(slot, tx_end, rx_end);
      const auto end = now();
      timing.record(slot, deadline, woke, tx_end, rx_end, end);
      ++slots;
      elapsed_ns = end-start;
      if (end-deadline >= slot_budget_ns || elapsed_ns >= phase_budget_ns)
        throw std::runtime_error("disabled startup exceeded its bounded preparation budget");
      clean_slots = !expired && end-deadline < TransportSlotTiming::budget_ns && ready(end)
          ? clean_slots+1 : 0;
      if (slots >= minimum_slots && clean_slots >= required_clean_slots && slot%4 == 3)
        return;  // Caller must also drain and verify all final disabled replies.
      deadline += TransportSlotTiming::budget_ns;
      if (end >= deadline) deadline = end + TransportSlotTiming::budget_ns;
    }
  }
};
