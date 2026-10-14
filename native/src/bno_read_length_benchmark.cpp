// Read-only length comparison; no CAN, IPC targets or sensor configuration.
#include "imu_packet.hpp"
#include <algorithm>
#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <vector>
namespace {
volatile std::sig_atomic_t stopped=0;
void stop(int){stopped=1;}
std::uint64_t ns(){timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec*1000000000ULL+t.tv_nsec;}
struct Fd{int n=-1;~Fd(){if(n>=0)close(n);}};
void read(int fd,std::uint8_t reg,std::uint8_t* data,std::uint16_t count){
  i2c_msg m[2]={{0x28,0,1,&reg},{0x28,I2C_M_RD,count,data}};
  i2c_rdwr_ioctl_data tx{m,2};
  if(ioctl(fd,I2C_RDWR,&tx)!=2)throw std::runtime_error("I2C read failed");
}
void health(int fd){
  for(auto p:std::array<std::array<std::uint8_t,2>,9>{{{{7,0}},{{0,160}},{{0x3b,2}},{{0x3d,8}},{{0x3e,0}},{{0x41,0x21}},{{0x42,4}},{{0x39,5}},{{0x3a,0}}}}){
    std::uint8_t value=0;read(fd,p[0],&value,1);
    if(value!=p[1])throw std::runtime_error("IMUPLUS/P0 health mismatch; no auto-write");
  }
}
struct Row{int block,mode;std::uint64_t start,duration,first,late,skip;};
}
int main(int argc,char**argv){
  if(argc!=2){std::cerr<<"Usage: bno_read_length_benchmark NEW_OUTPUT.csv\n";return 2;}
  std::vector<Row> rows;rows.reserve(12000);int rc=0;
  try{
    Fd output;output.n=open(argv[1],O_CREAT|O_EXCL|O_WRONLY,0600);
    if(output.n<0)throw std::runtime_error("output already exists or cannot create");
    Fd lock;lock.n=open("/tmp/sprite_bno055_bus7.lock",O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(lock.n<0||flock(lock.n,LOCK_EX|LOCK_NB)<0)throw std::runtime_error("BNO reader already active");
    Fd fd;fd.n=open("/dev/i2c-7",O_RDWR|O_CLOEXEC);
    if(fd.n<0)throw std::runtime_error("cannot open I2C7");
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    const std::array<int,8> order{{46,32,26,23,23,26,32,46}};
    for(int block=0;block<8;++block){
      health(fd.n);const int mode=order[block];auto deadline=ns();
      const auto finish=deadline+15000000000ULL;
      std::cout<<"BLOCK "<<block<<" mode="<<mode<<" duration=15s NO_CAN\n"<<std::flush;
      while(ns()<finish&&!stopped){
        timespec when{static_cast<time_t>(deadline/1000000000ULL),static_cast<long>(deadline%1000000000ULL)};
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&when,nullptr);
        if(stopped)break;
        sprite_imu::Raw raw{};const auto start=ns();std::uint64_t first=0;
        if(mode==26)read(fd.n,0x14,raw.data()+12,26);
        else if(mode==23){read(fd.n,0x08,raw.data(),23);first=ns()-start;read(fd.n,0x1f,raw.data()+23,23);}
        else read(fd.n,0x08,raw.data(),mode);
        const auto end=ns();sprite_imu::decode(raw);
        const auto late=start-deadline;deadline+=10000000;
        std::uint64_t skip=0;const auto now=ns();
        if(now>deadline){skip=(now-deadline)/10000000+1;deadline+=skip*10000000;}
        rows.push_back({block,mode,start,end-start,first,late,skip});
        if(end-start>30000000)throw std::runtime_error("read exceeded30ms");
      }
      if(stopped)throw std::runtime_error("interrupted");
    }
    health(fd.n);
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';rc=1;}
  // File writes happen only after acquisition.
  if(!rows.empty()){
    std::ofstream out(argv[1]);out<<"block,mode,start_ns,read_ns,first_read_ns,lateness_ns,skipped_slots\n";
    for(auto&r:rows)out<<r.block<<','<<r.mode<<','<<r.start<<','<<r.duration<<','<<r.first<<','<<r.late<<','<<r.skip<<'\n';
    if(!out)rc=1;
  }
  std::cout<<"FINISHED rows="<<rows.size()<<" exit="<<rc<<"\n";return rc;
}
