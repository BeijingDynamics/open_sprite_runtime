#include "core.hpp"
#include <iostream>
void require(bool b) {if(!b)throw std::runtime_error("offline assertion failed");}
struct Mock {
  uint8_t bytes[256]{};unsigned reads=0,writes=0;uint32_t last_size=0;
  static int read(void* p,uint8_t r,uint8_t* b,uint32_t n){
    auto& m=*static_cast<Mock*>(p);++m.reads;m.last_size=n;
    if(unsigned(r)+n>256)return -1;
    std::memcpy(b,m.bytes+r,n);return 0;
  }
  static int write(void* p,uint8_t r,const uint8_t* b,uint32_t n){
    auto& m=*static_cast<Mock*>(p);++m.writes;
    if(unsigned(r)+n>256)return -1;
    std::memcpy(m.bytes+r,b,n);return 0;
  }
};
int main() {
  Mock m;inv_imu_device_t d{};d.transport.context=&m;d.transport.serif_type=UI_SPI4;
  d.transport.read_reg=Mock::read;d.transport.write_reg=Mock::write;
  m.bytes[WHO_AM_I]=0xE9;uint8_t id=0;
  icm::check(inv_imu_get_who_am_i(&d,&id));require(id==0xE9&&m.writes==0);
  for(int rate:{400,800}) {
    icm::check(inv_imu_set_accel_frequency(&d,static_cast<accel_config0_accel_odr_t>(icm::odr(rate))));
    icm::check(inv_imu_set_gyro_frequency(&d,static_cast<gyro_config0_gyro_odr_t>(icm::odr(rate))));
    icm::check(inv_imu_set_accel_fsr(&d,ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G));
    icm::check(inv_imu_set_gyro_fsr(&d,GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS));
    require(m.bytes[ACCEL_CONFIG0]==(0x30|icm::odr(rate)));
    require(m.bytes[GYRO_CONFIG0]==(0x10|icm::odr(rate)));
  }
  for(int endian:{0,1}) {
    d.endianness_data=endian;
    for(int i=0;i<7;++i){m.bytes[2*i]=endian?0xFE:0xDC;m.bytes[2*i+1]=endian?0xDC:0xFE;}
    inv_imu_sensor_data_t raw{};icm::check(inv_imu_get_register_data(&d,&raw));
    require(m.last_size==14&&raw.accel_data[0]==-292&&raw.gyro_data[2]==-292&&raw.temp_data==-292);
  }
  require(icm::odr(400)==7 && icm::odr(800)==6);
  bool rejected=false;try{icm::odr(1000);}catch(...){rejected=true;}require(rejected);
  inv_imu_sensor_data_t s{};s.accel_data[0]=8192;s.gyro_data[2]=16384;
  require(std::abs(icm::accel(s)[0]-9.80665)<1e-10);
  require(std::abs(icm::gyro(s)[2]-17.4532925199433)<1e-10);
  require(icm::body({1,2,3})==sprite_imu::Vec3{-3,-2,-1});
  for(int hz:{400,800}) {
    sprite_imu::SixAxis f;std::uint64_t t=1000000000;
    f.initialize({0,0,9.80665},{0,0,.01},t);
    for(int i=0;i<hz;++i) {t+=1000000000/hz;f.update({0,0,9.80665},{0,0,1.01},t);}
    require(std::abs(f.quaternion()[0]-std::cos(.5))<1e-9);
    require(std::abs(f.quaternion()[3]-std::sin(.5))<1e-9);
  }
  std::cout<<"PASS read-only identity, register rates/ranges, 14-byte endian decode, SI units, mounting, estimator at 400/800Hz\n";
}
