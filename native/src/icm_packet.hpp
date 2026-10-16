#pragma once
#include "imu_packet.hpp"

namespace sprite_imu {
inline Packet encode_icm(const std::array<std::int16_t,6>& raw,
                         const std::array<double,3>& sensor_bias,
                         const std::array<double,4>& body_q,
                         std::uint64_t sequence,std::uint64_t start,std::uint64_t end) {
  if (!sequence || !start || end<start || end-start>30000000)
    throw std::runtime_error("invalid ICM timestamps");
  Packet p{};std::memcpy(p.data(),"SPIMU003",8);
  put64(p,8,sequence);put64(p,16,start);put64(p,24,end);
  for(unsigned i=0;i<6;++i) {
    const auto u=static_cast<std::uint16_t>(raw[i]);
    p[32+2*i]=u&255;p[33+2*i]=u>>8;
  }
  auto put_double=[&](unsigned offset,double v) {
    if(!std::isfinite(v))throw std::runtime_error("nonfinite ICM value");
    std::uint64_t bits;std::memcpy(&bits,&v,8);put64(p,offset,bits);
  };
  std::array<double,13> v{};
  for(unsigned i=0;i<3;++i) {
    put_double(56+8*i,sensor_bias[i]);
    v[i]=raw[i]*(4.0*9.80665/32768.0);
    v[3+i]=raw[3+i]*(2000.0*std::acos(-1.0)/180.0/32768.0)-sensor_bias[i];
  }
  // Policy's existing adapter expects world<-sensor, not world<-body.
  const double h=std::sqrt(.5),w=body_q[0],x=body_q[1],y=body_q[2],z=body_q[3];
  if(std::abs(w*w+x*x+y*y+z*z-1)>1e-6)throw std::runtime_error("ICM quaternion norm");
  const double a=h*(z-x),b=h*(w-y),c=h*(z+x),d=-h*(w+y);
  v[6]=a;v[7]=b;v[8]=c;v[9]=d;
  v[10]=9.80665*2*(b*d-a*c);v[11]=9.80665*2*(c*d+a*b);
  v[12]=9.80665*(1-2*(b*b+c*c));
  for(unsigned i=0;i<13;++i)put_double(80+8*i,v[i]);
  return p;
}
}
