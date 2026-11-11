#pragma once

#include <cstdint>
#include <stdexcept>

constexpr std::int64_t kFirstSlotWakeLeadNs = 1'000'000;

// Only the first deadline after a long idle uses a bounded active wait.
// Keep the original deadline: an oversleep is still late, never rebased.
template <class Clock, class Sleep, class Cancel>
void wait_first_slot(std::int64_t deadline, Clock now, Sleep sleep, Cancel cancel) {
  if (cancel()) throw std::runtime_error("first-slot wait cancelled");
  const auto wake_at = deadline - kFirstSlotWakeLeadNs;
  if (now() < wake_at) sleep(wake_at);
  while (now() < deadline) {
    if (cancel()) throw std::runtime_error("first-slot wait cancelled");
  }
  if (cancel()) throw std::runtime_error("first-slot wait cancelled");
}
