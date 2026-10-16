#pragma once
#include "imu/inv_imu_driver.h"
#include "imu_six_axis.hpp"
namespace icm {
inline void check(int rc) { if(rc) throw std::runtime_error("TDK transport/configuration failure"); }
inline int odr(int rate) {
  if(rate==400)return 7;
  if(rate==800)return 6;
  throw std::runtime_error("only 400 and 800 Hz are permitted");
}
inline sprite_imu::Vec3 accel(const inv_imu_sensor_data_t& s) {
  sprite_imu::Vec3 v{};
  for(int i=0;i<3;++i)v[i]=s.accel_data[i]*(4.*9.80665/32768.);
  return v;
}
inline sprite_imu::Vec3 gyro(const inv_imu_sensor_data_t& s) {
  sprite_imu::Vec3 v{};
  for(int i=0;i<3;++i)v[i]=s.gyro_data[i]*(2000.*3.141592653589793/180./32768.);
  return v;
}
inline sprite_imu::Vec3 body(const sprite_imu::Vec3& s) {return {-s[2],-s[1],-s[0]};}
inline void write(inv_imu_device_t& d,uint32_t reg,uint8_t value) {
  check(inv_imu_write_reg(&d,reg,1,&value));
}
inline uint8_t read(inv_imu_device_t& d,uint32_t reg) {
  uint8_t v=0;check(inv_imu_read_reg(&d,reg,1,&v));return v;
}
inline void configure(inv_imu_device_t& d,int rate) {
  const int code=odr(rate);
  check(inv_imu_soft_reset(&d));
  write(d,INT1_CONFIG0,0);write(d,INT1_CONFIG1,0);
  check(inv_imu_set_accel_frequency(&d,static_cast<accel_config0_accel_odr_t>(code)));
  check(inv_imu_set_gyro_frequency(&d,static_cast<gyro_config0_gyro_odr_t>(code)));
  check(inv_imu_set_accel_fsr(&d,ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G));
  check(inv_imu_set_gyro_fsr(&d,GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS));
  check(inv_imu_set_accel_ln_bw(&d,IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4));
  check(inv_imu_set_gyro_ln_bw(&d,IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4));
  inv_imu_int_pin_config_t pin{};
  pin.int_polarity=INTX_CONFIG2_INTX_POLARITY_HIGH;
  pin.int_mode=INTX_CONFIG2_INTX_MODE_PULSE;
  pin.int_drive=INTX_CONFIG2_INTX_DRIVE_PP;
  check(inv_imu_set_pin_config_int(&d,INV_IMU_INT1,&pin));
  check(inv_imu_set_accel_mode(&d,PWR_MGMT0_ACCEL_MODE_LN));
  check(inv_imu_set_gyro_mode(&d,PWR_MGMT0_GYRO_MODE_LN));
  d.transport.sleep_us(100000);
  if(read(d,ACCEL_CONFIG0)!=(0x30|code) || read(d,GYRO_CONFIG0)!=(0x10|code))
    throw std::runtime_error("ODR/full-scale readback mismatch");
  // Explicit bytes avoid uninitialized AUX/reserved bitfields in upstream's
  // set_config_int helper. Only UI DRDY is enabled; no FIFO/APEX/fusion.
  write(d,INT1_CONFIG0,4);write(d,INT1_CONFIG1,0);
  if(read(d,INT1_CONFIG0)!=4 || read(d,INT1_CONFIG1)!=0)
    throw std::runtime_error("interrupt readback mismatch");
}
}
