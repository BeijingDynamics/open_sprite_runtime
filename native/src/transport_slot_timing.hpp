#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>

// Fixed storage only: format the evidence after leaving the realtime loop.
struct TransportSlotTiming {
  struct Sample {
    std::size_t slot;
    std::int64_t wake_ns, tx_ns, rx_ns, ipc_ns, total_ns;
  };
  static constexpr std::int64_t budget_ns = 500'000;
  std::array<Sample, 32> overruns{};
  std::size_t count = 0, skipped_slots = 0;
  Sample maximum{};
  std::int64_t max_wake_ns = 0, max_tx_ns = 0, max_rx_ns = 0, max_ipc_ns = 0;

  void record(std::size_t slot, std::int64_t deadline, std::int64_t wake,
              std::int64_t tx_end, std::int64_t rx_end, std::int64_t end) {
    const Sample value{slot, std::max<std::int64_t>(0, wake-deadline),
                       tx_end-wake, rx_end-tx_end, end-rx_end, end-deadline};
    record_sample(value);
  }

  void record_sample(const Sample& value) {
    max_wake_ns = std::max(max_wake_ns, value.wake_ns);
    max_tx_ns = std::max(max_tx_ns, value.tx_ns);
    max_rx_ns = std::max(max_rx_ns, value.rx_ns);
    max_ipc_ns = std::max(max_ipc_ns, value.ipc_ns);
    if (value.total_ns > maximum.total_ns) maximum = value;
    skipped_slots += static_cast<std::size_t>(value.wake_ns >= budget_ns);
    if (value.total_ns >= budget_ns) {
      if (count < overruns.size()) overruns[count] = value;
      ++count;
    }
  }

  static void sample_json(std::ostream& out, const Sample& sample) {
    out << "{\"slot\":" << sample.slot << ",\"wake_ms\":" << sample.wake_ns/1e6
        << ",\"tx_ms\":" << sample.tx_ns/1e6 << ",\"rx_ms\":" << sample.rx_ns/1e6
        << ",\"ipc_preview_ms\":" << sample.ipc_ns/1e6
        << ",\"total_ms\":" << sample.total_ns/1e6 << '}';
  }

  void json_fields(std::ostream& out) const {
    out << "  \"transport_timing_scope\": \"deadline_to_end_including_ipc_preview\",\n"
        << "  \"transport_expired_slots_skipped\": " << skipped_slots << ",\n"
        << "  \"transport_stage_max_ms\": {\"wake\":" << max_wake_ns/1e6
        << ",\"tx\":" << max_tx_ns/1e6 << ",\"rx\":" << max_rx_ns/1e6
        << ",\"ipc_preview\":" << max_ipc_ns/1e6 << "},\n"
        << "  \"transport_worst_slot\": ";
    sample_json(out, maximum);
    out << ",\n  \"transport_overrun_samples\": [";
    for (std::size_t i=0; i<std::min(count, overruns.size()); ++i) {
      if (i) out << ',';
      sample_json(out, overruns[i]);
    }
    out << "],\n  \"transport_overrun_samples_truncated\": "
        << (count > overruns.size() ? "true" : "false") << ",\n";
  }
};
