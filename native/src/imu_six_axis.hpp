#pragma once
#include "imu_packet.hpp"
#include <algorithm>

namespace sprite_imu {
using Vec3 = std::array<double,3>;
inline double length(const Vec3& v) {
  return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}
inline void finite(const Vec3& v) {
  for(double x:v) if(!std::isfinite(x)) throw std::runtime_error("nonfinite six-axis sample");
}
inline Vec3 cross(const Vec3& a,const Vec3& b) {
  return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
constexpr double gravity=9.80665;

// Fixed, startup-only bias: motion must not silently become a gyro offset.
class StaticBias {
  Vec3 a_mean_{},w_mean_{},a_m2_{},w_m2_{};
  std::uint64_t first_=0,last_=0;
  unsigned n_=0;
public:
  void add(const Vec3& a,const Vec3& w,std::uint64_t timestamp) {
    finite(a);finite(w);
    if(!timestamp || (last_ && (timestamp<=last_ || timestamp-last_>30000000)))
      throw std::runtime_error("bias sampling timestamp/gap");
    if(std::abs(length(a)-gravity)>.1*gravity || length(w)>.1)
      throw std::runtime_error("keep IMU still for startup bias calibration");
    if(!n_) first_=timestamp;
    last_=timestamp;++n_;
    for(int i=0;i<3;++i) {
      const double da=a[i]-a_mean_[i],dw=w[i]-w_mean_[i];
      a_mean_[i]+=da/n_;w_mean_[i]+=dw/n_;
      a_m2_[i]+=da*(a[i]-a_mean_[i]);w_m2_[i]+=dw*(w[i]-w_mean_[i]);
    }
    if(n_>50) for(int i=0;i<3;++i)
      if(std::sqrt(a_m2_[i]/(n_-1))>.15 || std::sqrt(w_m2_[i]/(n_-1))>.005)
        throw std::runtime_error("startup bias window is not stationary");
  }
  bool ready() const {return n_>=1400 && last_-first_>=15000000000ULL;}
  Vec3 bias() const {
    if(!ready()) throw std::runtime_error("bias window incomplete");
    return w_mean_;
  }
  Vec3 acceleration() const {return a_mean_;}
};

// Nonlinear complementary attitude filter: gyro propagation and proportional
// measured-up x predicted-up correction, no magnetometer or online bias learning.
// Independently expressed standard equations; reference behavior documented in
// docs/BNO055_SOFTWARE_FUSION.md (SpacemiT BMI270 and Mahony et al.).
class SixAxis {
  std::array<double,4> q_{1,0,0,0}; // sensor -> world, wxyz
  Vec3 bias_{};
  std::uint64_t last_=0;
public:
  void initialize(const Vec3& a,const Vec3& bias,std::uint64_t timestamp) {
    finite(a);finite(bias);
    if(!timestamp || std::abs(length(a)-gravity)>.1*gravity || length(bias)>.1)
      throw std::runtime_error("invalid six-axis initialization");
    const double roll=std::atan2(a[1],a[2]);
    const double pitch=std::atan2(-a[0],std::hypot(a[1],a[2]));
    const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
    q_={cr*cp,sr*cp,cr*sp,-sr*sp};bias_=bias;last_=timestamp;
  }
  Vec3 up() const {
    const auto& q=q_;
    return {2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2])};
  }
  Vec3 corrected_gyro(const Vec3& w) const {
    finite(w);return {w[0]-bias_[0],w[1]-bias_[1],w[2]-bias_[2]};
  }
  const std::array<double,4>& quaternion() const {return q_;}
  Vec3 bias() const {return bias_;}
  double update(const Vec3& a,const Vec3& w,std::uint64_t timestamp) {
    finite(a);finite(w);
    if(!last_ || timestamp<=last_ || timestamp-last_>30000000)
      throw std::runtime_error("six-axis timestamp gap/regression; restart required");
    const double dt=(timestamp-last_)*1e-9;
    const double magnitude=length(a);
    double trust=std::clamp((.25-std::abs(magnitude/gravity-1))/.20,0.,1.);
    auto omega=corrected_gyro(w);
    if(trust>0 && magnitude>1e-9) {
      Vec3 measured{a[0]/magnitude,a[1]/magnitude,a[2]/magnitude};
      const auto error=cross(measured,up());
      // 1/s correction gain, matching the inspected reference's initial setting.
      for(int i=0;i<3;++i) omega[i]+=trust*error[i];
    }
    const double speed=length(omega),half=.5*speed*dt;
    const double scale=speed>1e-12 ? std::sin(half)/speed : .5*dt;
    const double wq=std::cos(half),x=scale*omega[0],y=scale*omega[1],z=scale*omega[2];
    const auto q=q_;
    q_={q[0]*wq-q[1]*x-q[2]*y-q[3]*z,
        q[0]*x+q[1]*wq+q[2]*z-q[3]*y,
        q[0]*y-q[1]*z+q[2]*wq+q[3]*x,
        q[0]*z+q[1]*y-q[2]*x+q[3]*wq};
    double norm=0;for(double v:q_)norm+=v*v;
    if(!std::isfinite(norm) || norm<1e-12)throw std::runtime_error("invalid software attitude");
    for(double& v:q_)v/=std::sqrt(norm);
    last_=timestamp;return trust;
  }
};

inline Vec3 raw_accel(const Raw& raw) {
  return {signed16(raw,0)/100.,signed16(raw,2)/100.,signed16(raw,4)/100.};
}
inline Vec3 raw_gyro(const Raw& raw) {
  return {signed16(raw,12)/900.,signed16(raw,14)/900.,signed16(raw,16)/900.};
}
inline void put_double(Packet& p,std::size_t offset,double v) {
  std::uint64_t bits;std::memcpy(&bits,&v,8);put64(p,offset,bits);
}
// Distinct wire version; never forge hardware fusion registers for old clients.
// 32..49 raw18,50..55 zero,56..79 bias,80..183 values,184..191 zero.
inline Packet encode_software(const Raw& raw,const SixAxis& filter,std::uint64_t seq,
                              std::uint64_t start,std::uint64_t end) {
  if(!seq || !start || end<start || end-start>30000000)
    throw std::runtime_error("invalid software IMU timestamp");
  Packet p{};std::memcpy(p.data(),"SPIMU002",8);
  put64(p,8,seq);put64(p,16,start);put64(p,24,end);
  std::memcpy(p.data()+32,raw.data(),18);
  std::array<double,13> values{};
  const auto a=raw_accel(raw),w=filter.corrected_gyro(raw_gyro(raw)),u=filter.up(),b=filter.bias();
  for(int i=0;i<3;++i) {values[i]=a[i];values[3+i]=w[i];values[10+i]=gravity*u[i];put_double(p,56+8*i,b[i]);}
  for(int i=0;i<4;++i)values[6+i]=filter.quaternion()[i];
  for(int i=0;i<13;++i)put_double(p,80+8*i,values[i]);
  return p;
}
} // namespace sprite_imu
