#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace sprite_imu {
constexpr std::size_t kPacketSize = 192;
using Raw = std::array<std::uint8_t, 46>;
using Packet = std::array<std::uint8_t, kPacketSize>;
inline std::int32_t signed16(const Raw& raw, std::size_t index) {
  const auto u = static_cast<std::uint32_t>(raw[index]) |
                 (static_cast<std::uint32_t>(raw[index + 1]) << 8);
  return u >= 32768 ? static_cast<std::int32_t>(u) - 65536 : static_cast<std::int32_t>(u);
}
inline std::array<double, 13> decode(const Raw& raw) {
  std::array<double, 13> v{};
  for (std::size_t i = 0; i < 3; ++i) {
    v[i] = signed16(raw, 2*i) / 100.0;
    v[3+i] = signed16(raw, 12+2*i) / 900.0;
    v[10+i] = signed16(raw, 38+2*i) / 100.0;
  }
  double norm = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    v[6+i] = signed16(raw, 24+2*i) / 16384.0;
    norm += v[6+i]*v[6+i];
  }
  if (std::abs(std::sqrt(norm)-1.0) > 0.05)
    throw std::runtime_error("invalid quaternion norm");
  return v;
}
inline void put64(Packet& packet, std::size_t offset, std::uint64_t value) {
  for (unsigned i=0; i<8; ++i) packet[offset+i]=static_cast<std::uint8_t>(value>>(8*i));
}
inline Packet encode(const Raw& raw, std::uint64_t sequence,
                     std::uint64_t start, std::uint64_t end) {
  if (!sequence || !start || end < start || end-start > 30000000)
    throw std::runtime_error("invalid sample timestamp or read exceeds30ms");
  const auto values=decode(raw);
  Packet p{};
  std::memcpy(p.data(), "SPIMU001", 8);
  put64(p,8,sequence); put64(p,16,start); put64(p,24,end);
  std::memcpy(p.data()+32,raw.data(),raw.size());
  static_assert(sizeof(double)==8, "requires binary64 double");
  for (std::size_t i=0; i<values.size(); ++i) {
    std::uint64_t bits;
    std::memcpy(&bits,&values[i],8); put64(p,80+8*i,bits);
  }
  return p;
}
}  // namespace sprite_imu
