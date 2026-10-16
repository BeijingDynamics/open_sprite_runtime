#include "icm_packet.hpp"
#include "policy_imu_client.hpp"
#include "policy_feedback.hpp"
#include "ipc.hpp"
#include <iostream>
void require(bool ok){if(!ok)throw std::runtime_error("test failed");}
int main(){
  using namespace sprite_imu;
  const std::array<std::int16_t,6> raw{-8192,123,-456,321,-234,567};
  const std::array<double,3> bias{.001,-.002,.003};
  for(int i=0;i<200;++i){
    std::array<double,4> q{std::cos(i*.03),std::sin(i*.04),std::cos(i*.05),std::sin(i*.06)};
    double norm=0;for(auto v:q)norm+=v*v;for(auto& v:q)v/=std::sqrt(norm);
    auto p=encode_icm(raw,bias,q,10,1000000000,1000200000);
    auto s=sprite_policy::validate_imu_packet(p.data(),p.size(),1001000000,9,false,true);
    const double w=s.values[6],x=s.values[7],y=s.values[8],z=s.values[9];
    const std::array<double,3> policy_g{1-2*(x*x+y*y),2*(y*z+x*w),2*(x*z-y*w)};
    const std::array<double,3> expected{-2*(q[1]*q[3]-q[0]*q[2]),-2*(q[2]*q[3]+q[0]*q[1]),-(1-2*(q[1]*q[1]+q[2]*q[2]))};
    for(unsigned j=0;j<3;++j)require(std::abs(policy_g[j]-expected[j])<1e-12);
    std::array<double,69> input{};
    for(unsigned j=0;j<3;++j)input[62+j]=s.values[3+j];
    for(unsigned j=0;j<4;++j)input[65+j]=s.values[6+j];
    auto feedback=sprite_policy::FeedbackMap{}.convert(input);
    for(unsigned j=0;j<3;++j)require(std::abs(feedback[65+j]-expected[j])<1e-12);
    require(std::abs(feedback[62]+s.values[5])<1e-12);
    require(std::abs(feedback[63]+s.values[4])<1e-12);
    require(std::abs(feedback[64]+s.values[3])<1e-12);
    require(std::abs(s.values[3]-(raw[3]*(2000.*std::acos(-1.)/180./32768.)-bias[0]))<1e-12);
    for(int fault=0;fault<6;++fault){
      auto bad=p;auto now=1001000000ULL;std::uint64_t prev=9;
      if(fault==0)bad[0]=0;if(fault==1)bad[44]=1;if(fault==2)bad[32]^=1;
      if(fault==3)now=1040000000;if(fault==4)prev=11;if(fault==5)now=999999999;
      bool caught=false;try{sprite_policy::validate_imu_packet(bad.data(),bad.size(),now,prev,false,true);}catch(const std::runtime_error&){caught=true;}
      require(caught);
    }
  }
  char pattern[]="/tmp/icm_packet_test_XXXXXX";
  auto dir=::mkdtemp(pattern);require(dir!=nullptr);
  const std::string path=std::string(dir)+"/imu.sock";
  {
    IcmServer server(path.c_str());
    auto now=sprite_policy::monotonic_ns();
    server.publish(encode_icm(raw,bias,{1,0,0,0},1,now,now));
    sprite_policy::NativeImuClient client(path,false,true);
    sprite_policy::ImuSnapshot result;
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
    bool received=false;std::uint64_t seq=1;
    while(std::chrono::steady_clock::now()<deadline){
      now=sprite_policy::monotonic_ns();server.publish(encode_icm(raw,bias,{1,0,0,0},++seq,now,now));
      if(client.snapshot(result)){received=true;break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(received);
  }
  require(::access(path.c_str(),F_OK)!=0);require(::rmdir(dir)==0);
  std::cout<<"ICM packet: 200 orientations and 1200 rejection cases passed\n";
}
