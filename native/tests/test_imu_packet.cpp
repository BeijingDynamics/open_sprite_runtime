#include "../src/imu_packet.hpp"
#include <iostream>

int main() {
  sprite_imu::Raw raw{};
  raw[25]=64;  // Identity quaternion: w=16384.
  raw[12]=0x7c; raw[13]=0xfc;  // -900 -> -1rad/s.
  const auto decoded=sprite_imu::decode(raw);
  if (decoded[3]!=-1.0 || decoded[6]!=1.0) return 1;
  const auto packet=sprite_imu::encode(raw,1,1000000000,1000000001);
  if (packet[0]!='S' || packet[8]!=1 || packet[78]!=0 || packet[191]!=0) return 1;
  int rejected=0;
  try { sprite_imu::encode(raw,0,1,2); } catch (const std::exception&) { ++rejected; }
  try { sprite_imu::encode(raw,1,0,2); } catch (const std::exception&) { ++rejected; }
  try { sprite_imu::encode(raw,1,3,2); } catch (const std::exception&) { ++rejected; }
  try { sprite_imu::encode(raw,1,1,30000002); } catch (const std::exception&) { ++rejected; }
  raw[25]=0;
  try { sprite_imu::encode(raw,1,1,2); } catch (const std::exception&) { ++rejected; }
  if (rejected!=5) return 1;
  std::cout<<"native IMU packet decode/guards passed\n";
  return 0;
}
