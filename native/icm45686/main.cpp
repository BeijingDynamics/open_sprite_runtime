#include "core.hpp"
#include "ipc.hpp"
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <vector>
#include <csignal>
#include <cerrno>
#include <iomanip>
#include <cstdlib>

namespace {
volatile std::sig_atomic_t stopped=0;
void stop(int){stopped=1;}
uint64_t now() {timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return uint64_t(t.tv_sec)*1000000000+t.tv_nsec;}
uint64_t cpu_now() {timespec t{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return uint64_t(t.tv_sec)*1000000000+t.tv_nsec;}
struct Fd {
  int fd=-1;
  explicit Fd(int f):fd(f){if(f<0)throw std::runtime_error(std::strerror(errno));}
  ~Fd(){if(fd>=0)close(fd);}
  Fd(const Fd&)=delete;
};
struct Spi {
  Fd file;
  explicit Spi(const char* path):file(open(path,O_RDWR|O_CLOEXEC)) {
    if(flock(file.fd,LOCK_EX|LOCK_NB))throw std::runtime_error("SPI already in use");
    uint8_t mode=SPI_MODE_3,bits=8,lsb=0;uint32_t speed=1000000;
    if(ioctl(file.fd,SPI_IOC_WR_MODE,&mode)<0 || ioctl(file.fd,SPI_IOC_WR_BITS_PER_WORD,&bits)<0 || ioctl(file.fd,SPI_IOC_WR_LSB_FIRST,&lsb)<0 || ioctl(file.fd,SPI_IOC_WR_MAX_SPEED_HZ,&speed)<0)
      throw std::runtime_error("SPI setup failed");
  }
  static int transfer(void* ctx,uint8_t reg,uint8_t* data,uint32_t n,bool reading) {
    if(n>256)return -1;
    auto* self=static_cast<Spi*>(ctx);
    uint8_t tx[257]{},rx[257]{};tx[0]=reg|(reading?0x80:0);
    if(!reading)std::memcpy(tx+1,data,n);
    spi_ioc_transfer x{};x.tx_buf=reinterpret_cast<uintptr_t>(tx);x.rx_buf=reinterpret_cast<uintptr_t>(rx);
    x.len=n+1;x.speed_hz=1000000;x.bits_per_word=8;
    if(ioctl(self->file.fd,SPI_IOC_MESSAGE(1),&x)!=int(n+1))return -1;
    if(reading)std::memcpy(data,rx+1,n);
    return 0;
  }
  static int read_cb(void* c,uint8_t r,uint8_t* b,uint32_t n){return transfer(c,r,b,n,true);}
  static int write_cb(void* c,uint8_t r,const uint8_t* b,uint32_t n){return transfer(c,r,const_cast<uint8_t*>(b),n,false);}
};
int gpio(const char* path,unsigned offset) {
  Fd chip(open(path,O_RDONLY|O_CLOEXEC));
  gpiochip_info ci{};
  gpio_v2_line_info li{};li.offset=offset;
  if(ioctl(chip.fd,GPIO_GET_CHIPINFO_IOCTL,&ci)<0 || ioctl(chip.fd,GPIO_V2_GET_LINEINFO_IOCTL,&li)<0)
    throw std::runtime_error("GPIO information query failed");
  if(std::string(ci.label)!="tegra234-gpio" || std::string(li.name)!="PY.01" || (li.flags&GPIO_V2_LINE_FLAG_USED))
    throw std::runtime_error("expected unused Orin Nano physical PIN22: tegra234-gpio PY.01");
  gpio_v2_line_request req{};req.offsets[0]=offset;req.num_lines=1;
  std::strcpy(req.consumer,"sprite-icm-no-can");req.event_buffer_size=128;
  req.config.flags=GPIO_V2_LINE_FLAG_INPUT|GPIO_V2_LINE_FLAG_EDGE_RISING;
  if(ioctl(chip.fd,GPIO_V2_GET_LINE_IOCTL,&req)<0)throw std::runtime_error("GPIO edge request failed");
  if(fcntl(req.fd,F_SETFL,O_NONBLOCK)<0){close(req.fd);throw std::runtime_error("GPIO nonblocking setup failed");}
  return req.fd;
}
struct Row {
  uint64_t event,start,end;uint32_t sequence;
  inv_imu_sensor_data_t raw;
  sprite_imu::Vec3 a,w,up;
  std::array<double,4> q;
  bool ready;
};
struct Timing { uint64_t event,start,end,cpu; uint32_t sequence; int rejected; };
}
int main(int argc,char** argv) {
  if(argc==1 || std::string(argv[1])=="--help") {
    std::cout<<"NO CAN. probe SPI | capture SPI GPIOCHIP OFFSET RATE SECONDS CSV X_DOWN_Z_BACK CONFIGURE_ICM45686_RAW_NO_CAN\n"
      <<"400/800Hz only; capture uses x-down/z-back mounting, 15s stationary bias included in duration.\n";
    return 0;
  }
  std::vector<Row> rows;std::string output,error;bool configured=false;
  std::vector<Timing> timings;
  uint64_t missed=0,coalesced=0,late=0;int rate=0;
  try {
    const bool probe=std::string(argv[1])=="probe";
    if((probe && argc!=3) || (!probe && (argc!=10 || std::string(argv[1])!="capture")))throw std::runtime_error("invalid arguments; see --help");
    double seconds=0;
    if(!probe) {
      rate=std::stoi(argv[5]);icm::odr(rate);seconds=std::stod(argv[6]);
      if(!std::isfinite(seconds)||seconds<20||seconds>600)throw std::runtime_error("duration must be 20..600s including bias");
      if(std::string(argv[9])!="CONFIGURE_ICM45686_RAW_NO_CAN")throw std::runtime_error("configuration confirmation missing");
      // argv[8] is an explicit mounting assertion, not an inferred orientation.
      if(std::string(argv[8])!="X_DOWN_Z_BACK")throw std::runtime_error("mount confirmation must be X_DOWN_Z_BACK");
      output=argv[7];
      if(access(output.c_str(),F_OK)==0 || access((output+".json").c_str(),F_OK)==0)throw std::runtime_error("output exists; choose a new capture path");
      rows.reserve(size_t(seconds*rate*1.2));
      timings.reserve(size_t(seconds*rate*1.2));
    }
    Spi spi(argv[2]);inv_imu_device_t d{};
    d.transport.context=&spi;d.transport.read_reg=Spi::read_cb;d.transport.write_reg=Spi::write_cb;
    d.transport.serif_type=UI_SPI4;
    d.transport.sleep_us=[](uint32_t us){std::this_thread::sleep_for(std::chrono::microseconds(us));};
    uint8_t id=0;icm::check(inv_imu_get_who_am_i(&d,&id));
    std::cout<<"WHO_AM_I="<<unsigned(id)<<" expected=233 NO_CAN\n";
    if(id!=0xE9)throw std::runtime_error("ICM45686 identity mismatch");
    if(probe)return 0;
    Fd edge(gpio(argv[3],unsigned(std::stoul(argv[4]))));
    IcmServer server(std::getenv("ICM_IPC_SOCKET"));
    icm::configure(d,rate);configured=true;
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    sprite_imu::StaticBias bias;sprite_imu::SixAxis estimator;bool ready=false;
    uint32_t last_seq=0;uint64_t begin=now(),last_good=begin;
    std::cout<<"KEEP STILL for first 15 seconds. Rate="<<rate<<" total seconds="<<seconds<<std::endl;
    try {
      while(!stopped && now()-begin<uint64_t(seconds*1e9)) {
        pollfd p{edge.fd,POLLIN,0};int rc=poll(&p,1,100);
        if(rc<0 && errno==EINTR)continue;
        if(rc<=0 || (p.revents&(POLLERR|POLLHUP|POLLNVAL)))throw std::runtime_error("DRDY timeout/error");
        gpio_v2_line_event ev{},next{};
        if(read(edge.fd,&ev,sizeof(ev))!=sizeof(ev))throw std::runtime_error("GPIO event read failed");
        while(read(edge.fd,&next,sizeof(next))==sizeof(next)){ev=next;++coalesced;}
        if(last_seq && ev.seqno>last_seq+1)missed+=ev.seqno-last_seq-1;
        last_seq=ev.seqno;
        const auto start=now();
        if(start<ev.timestamp_ns || start-ev.timestamp_ns>uint64_t(1e9/rate)) {
          timings.push_back({ev.timestamp_ns,start,start,0,ev.seqno,1});
          ++late;if(start-last_good>30000000)throw std::runtime_error("no fresh DRDY for 30ms");continue;
        }
        const auto cpu_start=cpu_now();
        inv_imu_sensor_data_t raw{};icm::check(inv_imu_get_register_data(&d,&raw));const auto end=now();
        timings.push_back({ev.timestamp_ns,start,end,cpu_now()-cpu_start,ev.seqno,
          end-ev.timestamp_ns>=uint64_t(1e9/rate)?2:0});
        // Register capture is not FIFO. Never attribute a possibly overwritten
        // sample to an old edge when acquisition spans the next sample period.
        if(end-ev.timestamp_ns>=uint64_t(1e9/rate)){
          ++late;if(end-last_good>30000000)throw std::runtime_error("SPI acquisition has no fresh sample for 30ms");continue;
        }
        auto a=icm::body(icm::accel(raw)),w=icm::body(icm::gyro(raw));
        if(!ready) {
          bias.add(a,w,ev.timestamp_ns);
          if(bias.ready()) {estimator.initialize(bias.acceleration(),bias.bias(),ev.timestamp_ns);ready=true;std::cout<<"BIAS_READY: movement may begin"<<std::endl;}
        } else estimator.update(a,w,ev.timestamp_ns);
        rows.push_back({ev.timestamp_ns,start,end,ev.seqno,raw,a,ready?estimator.corrected_gyro(w):w,ready?estimator.up():sprite_imu::Vec3{},ready?estimator.quaternion():std::array<double,4>{},ready});
        if(ready && std::getenv("ICM_VIEW_STDOUT") && rows.size()%16==0) {
          std::cout<<"POSE "<<ev.timestamp_ns<<std::setprecision(17);
          for(double v:estimator.quaternion())std::cout<<' '<<v;
          std::cout<<std::endl;
        }
        if(ready) {
          std::array<std::int16_t,6> counts{};
          for(unsigned i=0;i<3;++i){counts[i]=raw.accel_data[i];counts[3+i]=raw.gyro_data[i];}
          server.publish(sprite_imu::encode_icm(counts,icm::body(bias.bias()),estimator.quaternion(),ev.seqno,ev.timestamp_ns,end));
        }
        last_good=end;
      }
      if(!ready)throw std::runtime_error("stationary bias incomplete");
    } catch(...) {try{icm::write(d,INT1_CONFIG0,0);}catch(...){}throw;}
    icm::write(d,INT1_CONFIG0,0);
  } catch(const std::exception& e){error=e.what();std::cerr<<error<<'\n';}
  if(!output.empty() && configured) {
    std::ofstream timing_file(output+".timing.csv");
    timing_file<<"event_ns,start_ns,end_ns,thread_cpu_ns,sequence,rejected\n";
    for(const auto& t:timings) timing_file<<t.event<<','<<t.start<<','<<t.end<<','<<t.cpu<<','<<t.sequence<<','<<t.rejected<<'\n';
    timing_file.flush();
    if(!timing_file){std::cerr<<"timing report write failed\n";return 1;}
    std::ofstream f(output);f<<std::setprecision(17)<<"event_ns,read_start_ns,read_end_ns,sequence,bias_ready,ax_body,ay_body,az_body,wx_body,wy_body,wz_body,gx_body,gy_body,gz_body,qw,qx,qy,qz,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,raw_temp\n";
    uint64_t max_spi=0,max_age=0,max_gap=0,previous=0;
    for(const auto& r:rows) {
      f<<r.event<<','<<r.start<<','<<r.end<<','<<r.sequence<<','<<r.ready;
      for(double x:r.a)f<<','<<x;
      for(double x:r.w)f<<','<<x;
      for(double x:r.up)f<<','<<-x;
      for(double x:r.q)f<<','<<x;
      for(auto x:r.raw.accel_data)f<<','<<x;
      for(auto x:r.raw.gyro_data)f<<','<<x;
      f<<','<<r.raw.temp_data<<'\n';
      max_spi=std::max(max_spi,r.end-r.start);max_age=std::max(max_age,r.end-r.event);
      if(previous)max_gap=std::max(max_gap,r.event-previous);
      previous=r.event;
    }
    std::ofstream j(output+".json");j<<"{\"no_can\":true,\"rate_hz\":"<<rate<<",\"samples\":"<<rows.size()<<",\"missed_sequence_events\":"<<missed<<",\"coalesced_events\":"<<coalesced<<",\"late_samples_discarded\":"<<late<<",\"max_spi_ns\":"<<max_spi<<",\"max_edge_to_read_end_ns\":"<<max_age<<",\"max_sample_gap_ns\":"<<max_gap<<",\"completed\":"<<((error.empty()&&!stopped)?"true":"false")<<",\"hardware_qualified\":false}\n";
    f.flush();j.flush();if(!f||!j){std::cerr<<"report write failed\n";return 1;}
  }
  return error.empty()&&!stopped?0:1;
}
