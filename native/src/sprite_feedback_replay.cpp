// Offline conversion: no devices, transport, or actuation.
#include "policy_feedback.hpp"
#include <fstream>
#include <iostream>
#include <cstring>
#include <cstdint>
template<class T> void read_value(std::istream& in,T& value) {
  if(!in.read(reinterpret_cast<char*>(&value),sizeof(value))) throw std::runtime_error("truncated input");
}
int main(int argc,char** argv) {
  try {
    if(argc!=3) throw std::runtime_error("Usage: sprite_feedback_replay INPUT OUTPUT");
    const std::uint32_t endian=1;
    if(*reinterpret_cast<const char*>(&endian)!=1) throw std::runtime_error("little endian required");
    std::ifstream in(argv[1],std::ios::binary);std::ofstream out(argv[2],std::ios::binary);
    if(!in || !out) throw std::runtime_error("file open failed");
    std::array<char,8> magic{};read_value(in,magic);
    if(std::memcmp(magic.data(),"SPFDB001",8)) throw std::runtime_error("bad magic");
    std::uint32_t count=0;read_value(in,count);
    if(!count || count>1000000) throw std::runtime_error("invalid count");
    sprite_policy::FeedbackMap map;read_value(in,map.motor_zero);read_value(in,map.motor_to_joint);
    for(double v:map.motor_zero) if(!std::isfinite(v)) throw std::runtime_error("invalid offset");
    for(double v:map.motor_to_joint) if(!std::isfinite(v)) throw std::runtime_error("invalid map");
    for(std::uint32_t i=0;i<count;++i) {
      std::array<double,69> frame{};read_value(in,frame);
      const auto result=map.convert(frame);
      if(!out.write(reinterpret_cast<const char*>(result.data()),sizeof(result))) throw std::runtime_error("write failed");
    }
    if(in.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("trailing bytes");
    std::cout<<"FEEDBACK_REPLAY frames="<<count<<" hardware_tx=0\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
