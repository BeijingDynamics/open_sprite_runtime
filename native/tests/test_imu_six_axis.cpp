#include "imu_six_axis.hpp"
#include "policy_feedback.hpp"
#include "policy_imu_client.hpp"
#include <iostream>
#include <functional>
#include <limits>
using namespace sprite_imu;
void require(bool value) {if(!value)throw std::runtime_error("test assertion failed");}
void near(double a,double b,double tol=1e-9){require(std::abs(a-b)<tol);}
void rejects(const std::function<void()>& f){bool threw=false;try{f();}catch(const std::exception&){threw=true;}require(threw);}
void raw16(Raw& r,int offset,int value){r[offset]=value&255;r[offset+1]=(value>>8)&255;}
int main() {
  try {
    StaticBias calibration;
    for(int n=0;n<=1500;++n)calibration.add({0,0,gravity},{.01,-.02,.03},1ULL+n*10000000ULL);
    require(calibration.ready());near(calibration.bias()[1],-.02);
    StaticBias moved;rejects([&]{moved.add({0,0,gravity},{.2,0,0},1);});
    StaticBias rocking;
    rejects([&]{for(int n=0;n<100;++n)rocking.add({0,0,gravity},{n%2?.04:-.04,0,0},1ULL+n*10000000ULL);});
    SixAxis f;f.initialize({0,0,gravity},{.01,-.02,.03},1);
    for(int n=1;n<1000;++n)f.update({0,0,gravity},{.01,-.02,.03},1ULL+n*10000000ULL);
    near(f.up()[2],1);near(f.corrected_gyro({.4,.5,.6})[0],.39);
    rejects([&]{f.update({0,0,gravity},{0,0,0},1);});
    rejects([&]{f.update({0,0,gravity},{0,0,0},50000000000ULL);});
    rejects([&]{f.initialize({0,0,0},{},1);});
    rejects([&]{f.initialize({0,0,gravity},{std::numeric_limits<double>::quiet_NaN(),0,0},1);});
    // Arbitrary tilted startup is preserved, never leveled to identity.
    f.initialize({0,gravity*.5,gravity*std::sqrt(.75)},{},1);
    near(f.up()[1],.5);near(f.up()[2],std::sqrt(.75));
    // Gyro propagation around each sensor axis; zero accel disables correction.
    for(int axis=0;axis<3;++axis) {
      f.initialize({0,0,gravity},{},1);Vec3 w{};w[axis]=1;
      for(int n=1;n<=100;++n)f.update({},w,1ULL+n*10000000ULL);
      near(f.quaternion()[0],std::cos(.5));near(f.quaternion()[axis+1],std::sin(.5));
    }
    // Correction sign converges toward measured up without contaminating gyro obs.
    f.initialize({0,0,gravity},{},1);
    for(int n=1;n<=1000;++n)f.update({0,gravity*.5,gravity*std::sqrt(.75)},{},1ULL+n*10000000ULL);
    near(f.up()[1],.5,1e-4);near(f.corrected_gyro({})[0],0);
    require(f.update({0,0,3*gravity},{},10010000001ULL)==0);
    // Installed P0: upright specific force is sensor -X, mapped body +Z.
    Raw raw{};raw16(raw,0,-981);raw16(raw,12,9);raw16(raw,14,-18);raw16(raw,16,27);
    f.initialize(raw_accel(raw),{.01,-.02,.03},1000000000);
    auto p=encode_software(raw,f,1,1000000000,1001000000);
    auto s=sprite_policy::validate_imu_packet(p.data(),p.size(),1002000000,0,true);
    rejects([&]{sprite_policy::validate_imu_packet(p.data(),p.size(),1002000000,0);});
    rejects([&]{sprite_policy::validate_imu_packet(p.data(),p.size(),1100000000,0,true);});
    rejects([&]{sprite_policy::validate_imu_packet(p.data(),p.size(),1002000000,2,true);});
    auto damaged=p;put_double(damaged,56,.2);
    rejects([&]{sprite_policy::validate_imu_packet(damaged.data(),damaged.size(),1002000000,0,true);});
    damaged=p;put_double(damaged,80+8*6,0);
    rejects([&]{sprite_policy::validate_imu_packet(damaged.data(),damaged.size(),1002000000,0,true);});
    Raw old_raw{};old_raw[25]=64;
    auto old_packet=encode(old_raw,1,1000000000,1001000000);
    sprite_policy::validate_imu_packet(old_packet.data(),old_packet.size(),1002000000,0);
    rejects([&]{sprite_policy::validate_imu_packet(old_packet.data(),old_packet.size(),1002000000,0,true);});
    std::array<double,69> input{};
    for(int i=0;i<3;++i)input[62+i]=s.values[3+i];
    for(int i=0;i<4;++i)input[65+i]=s.values[6+i];
    auto result=sprite_policy::FeedbackMap{}.convert(input);
    near(result[65],0);near(result[66],0);near(result[67],-1);
    near(result[62],0);near(result[63],0);near(result[64],0);
    // Nonzero rates and arbitrary tilt must survive the entire packet adapter.
    for(const Vec3 a: {Vec3{-900,200,350},Vec3{-850,-300,-400},Vec3{0,0,981}}) {
      Raw tilted{};
      for(int i=0;i<3;++i)raw16(tilted,2*i,static_cast<int>(a[i]));
      raw16(tilted,12,90);raw16(tilted,14,-180);raw16(tilted,16,270);
      SixAxis estimate;estimate.initialize(raw_accel(tilted),{.01,-.02,.03},1000000000);
      auto packet=encode_software(tilted,estimate,1,1000000000,1001000000);
      auto sample=sprite_policy::validate_imu_packet(packet.data(),packet.size(),1002000000,0,true);
      for(int i=0;i<3;++i)input[62+i]=sample.values[3+i];
      for(int i=0;i<4;++i)input[65+i]=sample.values[6+i];
      auto mapped=sprite_policy::FeedbackMap{}.convert(input);
      near(mapped[62],-.27);near(mapped[63],.18);near(mapped[64],-.09);
      const double length=std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
      near(mapped[65],a[2]/length);near(mapped[66],a[1]/length);near(mapped[67],a[0]/length);
    }
    p[80]^=1;p[85]^=127;
    rejects([&]{sprite_policy::validate_imu_packet(p.data(),p.size(),1002000000,0,true);});
    std::cout<<"PASS six_axis: bias, motion rejection, tilt, 3-axis rotation, correction, timestamps, packet and policy mount\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
