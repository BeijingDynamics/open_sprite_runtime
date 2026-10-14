// Independent read-only BNO055 process. No CAN, policy or configuration writes.
#include "imu_packet.hpp"
#ifdef SPRITE_BNO_SOFTWARE_FUSION
#include "imu_six_axis.hpp"
#endif
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <poll.h>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

namespace {
volatile std::sig_atomic_t stopping=0;
void on_signal(int) { stopping=1; }
std::uint64_t now_ns() {
  timespec t{};
  if (clock_gettime(CLOCK_MONOTONIC,&t)!=0) throw std::runtime_error("clock failure");
  return static_cast<std::uint64_t>(t.tv_sec)*1000000000ULL+t.tv_nsec;
}
struct Fd {
  int value=-1;
  ~Fd() { if (value>=0) close(value); }
};
std::uint64_t runqueue_wait_ns(int fd) {
  std::array<char,256> text{};
  const auto n=pread(fd,text.data(),text.size()-1,0);
  unsigned long long runtime=0,wait=0,slices=0;
  if (n<=0 || std::sscanf(text.data(),"%llu %llu %llu",&runtime,&wait,&slices)!=3)
    throw std::runtime_error("cannot read thread schedstat");
  return wait;
}
void read_registers(int fd, std::uint8_t reg, std::uint8_t* data, std::uint16_t count) {
  // The write message selects an address only; no configuration value is sent.
  i2c_msg messages[2]={{0x28,0,1,&reg},{0x28,I2C_M_RD,count,data}};
  i2c_rdwr_ioctl_data txn{messages,2};
  if (ioctl(fd,I2C_RDWR,&txn)!=2) throw std::runtime_error("I2C_RDWR read failed");
}
void check_config(int fd,int only=-1) {
#ifdef SPRITE_BNO_SOFTWARE_FUSION
  const std::array<std::array<std::uint8_t,2>,9> expected{{
    {{0x07,0}},{{0x00,0xA0}},{{0x3B,2}},{{0x3D,5}},{{0x3E,0}},
    {{0x41,0x21}},{{0x42,4}},{{0x39,6}},{{0x3A,0}}}};
#else
  const std::array<std::array<std::uint8_t,2>,9> expected{{
    {{0x07,0}},{{0x00,0xA0}},{{0x3B,2}},{{0x3D,8}},{{0x3E,0}},
    {{0x41,0x21}},{{0x42,4}},{{0x39,5}},{{0x3A,0}}}};
#endif
  for (std::size_t i=0;i<expected.size();++i) {
    if(only>=0 && i!=static_cast<std::size_t>(only))continue;
    const auto& pair=expected[i];
    std::uint8_t value=0; read_registers(fd,pair[0],&value,1);
    if (value!=pair[1]) throw std::runtime_error("BNO config/health mismatch (no auto-write)");
  }
}
#ifdef SPRITE_BNO_SOFTWARE_FUSION
void configure_raw(int fd) {
  std::uint8_t page=0,id=0;read_registers(fd,0x07,&page,1);
  if(page)throw std::runtime_error("unexpected BNO register page before configuration");
  read_registers(fd,0,&id,1);
  if(id!=0xA0)throw std::runtime_error("BNO identity mismatch");
  auto write_value=[&](std::uint8_t reg,std::uint8_t value) {
    std::uint8_t bytes[2]={reg,value};i2c_msg msg{0x28,0,2,bytes};
    i2c_rdwr_ioctl_data tx{&msg,1};
    if(ioctl(fd,I2C_RDWR,&tx)!=1)throw std::runtime_error("BNO config write failure");
    std::uint8_t actual=0;read_registers(fd,reg,&actual,1);
    if(actual!=value)throw std::runtime_error("BNO config readback failure");
  };
  write_value(0x3D,0);usleep(25000);
  write_value(0x3E,0);write_value(0x3B,2);
  write_value(0x41,0x21);write_value(0x42,4); // P0, no second software P0 remap.
  write_value(0x07,1);
  try {
    write_value(0x08,0x0D); // 4g,62.5Hz bandwidth, normal accel.
    write_value(0x0A,0x18); // 2000dps,47Hz bandwidth, gyro.
    write_value(0x0B,0);    // normal gyro power.
  } catch(...) {write_value(0x07,0);throw;}
  write_value(0x07,0);write_value(0x3D,5);usleep(100000);
  check_config(fd);
}
#endif
double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0;
  std::sort(values.begin(),values.end());
  return values[static_cast<std::size_t>(fraction*(values.size()-1))];
}
int replay(const std::string& input, const std::string& output) {
  std::ifstream in(input,std::ios::binary); std::ofstream out(output,std::ios::binary);
  if (!in || !out) throw std::runtime_error("replay file open failed");
  sprite_imu::Raw raw{}; std::uint64_t seq=0;
  while (in.read(reinterpret_cast<char*>(raw.data()),raw.size())) {
    ++seq;
    const auto packet=sprite_imu::encode(raw,seq,seq*10000000,seq*10000000+1000);
    out.write(reinterpret_cast<const char*>(packet.data()),packet.size());
  }
  if (in.gcount()!=0 || !out) throw std::runtime_error("truncated replay or output failure");
  std::cout<<"REPLAY samples="<<seq<<" no_hardware=true\n"; return 0;
}
}  // namespace

int main(int argc,char** argv) {
  Fd device,endpoint,lock,schedstat;
  std::string socket_path,report_path;
  bool owns_socket=false;
  std::vector<double> gaps,reads;
  struct Timing { std::uint64_t sequence,start,lateness,read,health,skips,cpu,voluntary,involuntary,runqueue; };
  std::vector<Timing> timing;
  std::uint64_t count=0,skipped=0,replies=0,drops=0,last_start=0,max_gap=0;
  int exit_code=0;
  int schedstats_enabled=0;
  int rate_hz=100;
  std::uint64_t period_ns=10000000;
  std::string read_method="burst18";
#ifdef SPRITE_BNO_SOFTWARE_FUSION
  sprite_imu::StaticBias calibration;
  sprite_imu::SixAxis estimator;
  bool ready=false;
  int health_cursor=0;
  std::uint64_t published=0;
  std::vector<sprite_imu::Packet> software_packets;
#endif
  try {
    if (argc==4 && std::string(argv[1])=="--replay") return replay(argv[2],argv[3]);
#ifdef SPRITE_BNO_SOFTWARE_FUSION
    if ((argc!=6 && argc!=8) || std::string(argv[1])!="--record" || std::string(argv[5])!="CONFIGURE_RAW_P0_NO_CAN") {
      std::cerr<<"Usage: candidate --record SECONDS SOCKET REPORT CONFIGURE_RAW_P0_NO_CAN\n";
      return 2;
    }
    if(argc==8) {
      const std::string rate=argv[6];
      if(rate!="100" && rate!="200")throw std::runtime_error("rate must be100 or200");
      rate_hz=std::stoi(rate);period_ns=1000000000ULL/rate_hz;
      read_method=argv[7];
      if(read_method!="burst18" && read_method!="split6")throw std::runtime_error("read method must be burst18 or split6");
    }
#else
    if (argc!=5 || std::string(argv[1])!="--record") {
      std::cerr<<"Usage: sprite_imu_bno --record SECONDS SOCKET_PATH REPORT.json\n"
                 "       sprite_imu_bno --replay RAW46_FILE PACKETS_FILE\n";
      return 2;
    }
#endif
    std::size_t consumed=0; const int seconds=std::stoi(argv[2],&consumed);
    if (consumed!=std::strlen(argv[2]) || seconds<1 || seconds>600)
      throw std::runtime_error("duration must be integer1..600 seconds");
    socket_path=argv[3]; report_path=argv[4];
    sockaddr_un address{}; address.sun_family=AF_UNIX;
    if (socket_path.empty() || socket_path.size()>=sizeof(address.sun_path))
      throw std::runtime_error("invalid socket path");
    std::memcpy(address.sun_path,socket_path.c_str(),socket_path.size()+1);
    // Never remove a pre-existing endpoint; refuse a concurrent producer.
    endpoint.value=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if (endpoint.value<0 || bind(endpoint.value,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0)
      throw std::runtime_error("cannot bind fresh IMU endpoint");
    owns_socket=true;
    if (chmod(socket_path.c_str(),0600)<0) throw std::runtime_error("chmod socket failed");
    lock.value=open("/tmp/sprite_bno055_bus7.lock",O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if (lock.value<0 || flock(lock.value,LOCK_EX|LOCK_NB)<0)
      throw std::runtime_error("another native BNO producer holds lock");
    device.value=open("/dev/i2c-7",O_RDWR|O_CLOEXEC);
    if (device.value<0) throw std::runtime_error("cannot open /dev/i2c-7");
#ifdef SPRITE_BNO_SOFTWARE_FUSION
    configure_raw(device.value);
#else
    check_config(device.value);
#endif
    std::ifstream("/proc/sys/kernel/sched_schedstats")>>schedstats_enabled;
    schedstat.value=open("/proc/thread-self/schedstat",O_RDONLY|O_CLOEXEC);
    if (schedstat.value<0) throw std::runtime_error("cannot open thread schedstat");
    gaps.reserve(static_cast<std::size_t>(seconds)*rate_hz+1);
    reads.reserve(static_cast<std::size_t>(seconds)*rate_hz+1);
    timing.reserve(static_cast<std::size_t>(seconds)*rate_hz+1);
#ifdef SPRITE_BNO_SOFTWARE_FUSION
    software_packets.reserve(static_cast<std::size_t>(seconds)*rate_hz+1);
#endif
    std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal);
    const std::uint64_t begin=now_ns(),finish=begin+static_cast<std::uint64_t>(seconds)*1000000000ULL;
    std::uint64_t deadline=begin,next_check=begin+1000000000ULL;
    sprite_imu::Packet latest{};
#ifdef SPRITE_BNO_SOFTWARE_FUSION
    std::cout<<"RAW_IMU_CALIBRATING hold_still=15s no_CAN=true mode=ACCGYRO_P0 hz="<<rate_hz<<" read="<<read_method<<"\n"<<std::flush;
#else
    std::cout<<"NATIVE_IMU_READY no_CAN=true no_config_writes=true hz=100\n"<<std::flush;
#endif
    while (!stopping && now_ns()<finish) {
      auto now=now_ns();
      if (now>=deadline) {
        rusage before{},after{};
        const auto queue_before=runqueue_wait_ns(schedstat.value);
        getrusage(RUSAGE_THREAD,&before);
        timespec cpu_before{},cpu_after{};
        clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu_before);
        sprite_imu::Raw raw{}; const auto start=now_ns();
        const auto lateness=start-deadline;
#ifdef SPRITE_BNO_SOFTWARE_FUSION
        if(read_method=="split6") {
          read_registers(device.value,0x08,raw.data(),6);
          read_registers(device.value,0x14,raw.data()+12,6);
        } else read_registers(device.value,0x08,raw.data(),18);
#else
        read_registers(device.value,0x08,raw.data(),raw.size());
#endif
        const auto end=now_ns();
        clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu_after);
        getrusage(RUSAGE_THREAD,&after);
        const auto queue_after=runqueue_wait_ns(schedstat.value);
        const auto cpu_ns=static_cast<std::uint64_t>((cpu_after.tv_sec-cpu_before.tv_sec)*1000000000LL+cpu_after.tv_nsec-cpu_before.tv_nsec);
#ifdef SPRITE_BNO_SOFTWARE_FUSION
        if(end-start>30000000)throw std::runtime_error("raw IMU read exceeds30ms");
        const auto accel=sprite_imu::raw_accel(raw),gyro=sprite_imu::raw_gyro(raw);
        if(!ready) {
          calibration.add(accel,gyro,start);
          if(calibration.ready()) {
            estimator.initialize(calibration.acceleration(),calibration.bias(),start);
            ready=true;
            const auto b=estimator.bias();
            std::cout<<"SOFTWARE_IMU_READY bias_p0_rad_s="<<b[0]<<","<<b[1]<<","<<b[2]<<"\n"<<std::flush;
          }
        } else estimator.update(accel,gyro,start);
        if(ready) {
          latest=sprite_imu::encode_software(raw,estimator,++published,start,end);
          software_packets.push_back(latest);
        }
#else
        latest=sprite_imu::encode(raw,count+1,start,end);
#endif
        if (last_start) {
          const auto gap=start-last_start; gaps.push_back(gap/1e6);
          max_gap=std::max(max_gap,gap);
          if (gap>30000000) throw std::runtime_error("acquisition gap exceeds30ms");
        }
        last_start=start; ++count; reads.push_back((end-start)/1e6);
        std::uint64_t health_ns=0,skips_now=0;
        if (end>=next_check) {
          const auto health_start=now_ns();
#ifdef SPRITE_BNO_SOFTWARE_FUSION
          // Complete the same nine checks over nine 100ms intervals.
          check_config(device.value,health_cursor);
          health_cursor=(health_cursor+1)%9;
          next_check=now_ns()+100000000ULL;
#else
          check_config(device.value);
          next_check=now_ns()+1000000000ULL;
#endif
          health_ns=now_ns()-health_start;
        }
        deadline+=period_ns;
        now=now_ns();
        if (now>deadline) { const auto n=(now-deadline)/period_ns+1; skipped+=n; skips_now=n; deadline+=n*period_ns; }
        timing.push_back({count,start,lateness,end-start,health_ns,skips_now,cpu_ns,
          static_cast<std::uint64_t>(after.ru_nvcsw-before.ru_nvcsw),
          static_cast<std::uint64_t>(after.ru_nivcsw-before.ru_nivcsw),queue_after-queue_before});
      }
      // Bounded servicing budget; absent/slow clients never block sampling.
      for (int i=0;i<8 && now_ns()<deadline;++i) {
        std::array<char,9> request{}; sockaddr_un peer{}; socklen_t length=sizeof(peer);
        const auto n=recvfrom(endpoint.value,request.data(),request.size(),MSG_DONTWAIT,
                              reinterpret_cast<sockaddr*>(&peer),&length);
        if (n<0) {
          if (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR) break;
          throw std::runtime_error("IPC receive failed");
        }
        if (n!=8 || std::memcmp(request.data(),"GETIMU01",8)!=0 || !count) continue;
#ifdef SPRITE_BNO_SOFTWARE_FUSION
        if(!ready)continue;
#endif
        if (now_ns()-last_start>30000000) throw std::runtime_error("latest IMU expired");
        const auto sent=sendto(endpoint.value,latest.data(),latest.size(),MSG_DONTWAIT,
                              reinterpret_cast<sockaddr*>(&peer),length);
        if (sent==static_cast<ssize_t>(latest.size())) ++replies; else ++drops;
      }
      const auto wait_now=now_ns();
      if (wait_now<deadline) {
        const auto wait=deadline-wait_now;
        timespec timeout{static_cast<time_t>(wait/1000000000ULL),static_cast<long>(wait%1000000000ULL)};
        pollfd fd{endpoint.value,POLLIN,0};
        if (ppoll(&fd,1,&timeout,nullptr)<0 && errno!=EINTR) throw std::runtime_error("ppoll failed");
      }
    }
    if (stopping) throw std::runtime_error("capture interrupted");
#ifdef SPRITE_BNO_SOFTWARE_FUSION
    if(!ready || !published)throw std::runtime_error("no qualified software IMU samples");
#endif
  } catch (const std::exception& error) {
    std::cerr<<"NATIVE_IMU_ERROR "<<error.what()<<'\n'; exit_code=1;
  }
  if (owns_socket) unlink(socket_path.c_str());
  if (!report_path.empty()) {
    // Persist after acquisition so diagnostic disk I/O cannot delay sampling.
    std::ofstream trace(report_path+".timing.csv");
    trace<<"sequence,start_ns,lateness_ns,read_ns,health_ns,skipped_slots,read_cpu_ns,voluntary_switches,involuntary_switches,runqueue_wait_ns\n";
    for (const auto& t:timing)
      trace<<t.sequence<<','<<t.start<<','<<t.lateness<<','<<t.read<<','<<t.health<<','<<t.skips<<','<<t.cpu<<','<<t.voluntary<<','<<t.involuntary<<','<<t.runqueue<<'\n';
    if (!trace) exit_code=1;
    std::ofstream out(report_path);
#ifdef SPRITE_BNO_SOFTWARE_FUSION
    const auto bias=estimator.bias();
    std::ofstream packets(report_path+".packets",std::ios::binary);
    for(const auto& p:software_packets)packets.write(reinterpret_cast<const char*>(p.data()),p.size());
    if(!packets)exit_code=1;
    out<<"{\"mode\":\"native_bno_software_six_axis_candidate\",\"passed\":"<<(exit_code?"false":"true")
       <<",\"can_tx_count\":0,\"configuration_writes_allowed\":true,\"samples\":"<<count
       <<",\"published_samples\":"<<published<<",\"bias_p0_rad_s\":["<<bias[0]<<","<<bias[1]<<","<<bias[2]<<"]"
       <<",\"requested_rate_hz\":"<<rate_hz<<",\"read_method\":\""<<read_method<<"\""
#else
    out<<"{\"mode\":\"native_bno_readonly\",\"passed\":"<<(exit_code?"false":"true")
       <<",\"can_tx_count\":0,\"configuration_writes\":0,\"samples\":"<<count
#endif
       <<",\"schedstats_enabled\":"<<schedstats_enabled
       <<",\"skipped_slots\":"<<skipped<<",\"replies\":"<<replies<<",\"send_drops\":"<<drops
       <<",\"gap_p99_ms\":"<<percentile(gaps,.99)<<",\"gap_max_ms\":"<<max_gap/1e6
       <<",\"read_p99_ms\":"<<percentile(reads,.99)<<",\"read_max_ms\":"<<percentile(reads,1)<<"}\n";
    if (!out) exit_code=1;
  }
  return exit_code;
}
