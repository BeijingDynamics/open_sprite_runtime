#pragma once
#include "imu_packet.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>
#include <cstdlib>

namespace sprite_policy {
inline std::uint64_t monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct ImuSnapshot {
    std::uint64_t sequence=0, start=0, end=0;
    std::array<double,13> values{};
};
inline std::uint64_t read_u64(const std::uint8_t* p) {
    std::uint64_t v=0;
    for (unsigned i=0;i<8;++i) v|=static_cast<std::uint64_t>(p[i])<<(8*i);
    return v;
}
inline ImuSnapshot validate_imu_packet(const std::uint8_t* p,std::size_t size,
                                     std::uint64_t now,std::uint64_t previous,
                                     bool software_fusion=false) {
    if (size!=192 || std::memcmp(p,software_fusion?"SPIMU002":"SPIMU001",8))
        throw std::runtime_error("IMU packet size/version");
    if ((!software_fusion && (p[78] || p[79])) || read_u64(p+184)) throw std::runtime_error("IMU reserved bytes");
    ImuSnapshot s;
    s.sequence=read_u64(p+8);s.start=read_u64(p+16);s.end=read_u64(p+24);
    if (!s.sequence || s.sequence<previous) throw std::runtime_error("IMU sequence regression");
    if (!s.start || s.end<s.start || s.end>now || now-s.start>30000000)
        throw std::runtime_error("IMU stale/future timestamp");
    sprite_imu::Raw raw{};std::memcpy(raw.data(),p+32,46);
    if(!software_fusion)s.values=sprite_imu::decode(raw);
    for (std::size_t i=0;i<13;++i) {
        auto bits=read_u64(p+80+8*i);double value;std::memcpy(&value,&bits,8);
        if (!std::isfinite(value) || (!software_fusion && std::abs(value-s.values[i])>1e-12))
            throw std::runtime_error("IMU raw/decoded disagreement");
        if(software_fusion)s.values[i]=value;
    }
    if(software_fusion) {
        for(int i=50;i<56;++i)if(p[i])throw std::runtime_error("software IMU reserved bytes");
        double bias_norm=0;
        for(int i=0;i<3;++i) {
            auto bits=read_u64(p+56+8*i);double bias;std::memcpy(&bias,&bits,8);
            if(!std::isfinite(bias))throw std::runtime_error("nonfinite gyro bias");
            bias_norm+=bias*bias;
            if(std::abs(s.values[i]-sprite_imu::signed16(raw,2*i)/100.)>1e-12 ||
               std::abs(s.values[3+i]-(sprite_imu::signed16(raw,12+2*i)/900.-bias))>1e-12)
                throw std::runtime_error("software IMU raw/bias disagreement");
        }
        if(bias_norm>.01)throw std::runtime_error("software IMU excessive gyro bias");
        const double w=s.values[6],x=s.values[7],y=s.values[8],z=s.values[9];
        if(std::abs(w*w+x*x+y*y+z*z-1)>1e-6)throw std::runtime_error("software quaternion norm");
        const std::array<double,3> up{2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)};
        for(int i=0;i<3;++i)if(std::abs(s.values[10+i]-9.80665*up[i])>1e-9)
            throw std::runtime_error("software gravity/quaternion disagreement");
    }
    return s;
}
class NativeImuClient {
    int fd_=-1;
    std::string directory_, path_, error_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
    std::mutex mutex_;
    ImuSnapshot latest_;
    bool software_fusion_=false;
    void cleanup() noexcept {
        if (fd_>=0) { ::close(fd_);fd_=-1; }
        if (!path_.empty()) ::unlink(path_.c_str());
        if (!directory_.empty()) ::rmdir(directory_.c_str());
    }
    static sockaddr_un address(const std::string& path) {
        sockaddr_un a{};a.sun_family=AF_UNIX;
        if (path.empty() || path.size()>=sizeof(a.sun_path)) throw std::runtime_error("invalid IMU socket path");
        std::memcpy(a.sun_path,path.c_str(),path.size()+1);return a;
    }
    void run() noexcept {
        try {
            std::uint64_t previous=0;
            while (!stop_) {
                const auto begin=std::chrono::steady_clock::now();
                if (::send(fd_,"GETIMU01",8,MSG_DONTWAIT)!=8) throw std::runtime_error("IMU request failed");
                pollfd poller{fd_,POLLIN,0};
                if (::poll(&poller,1,25)!=1 || !(poller.revents&POLLIN))
                    throw std::runtime_error("IMU IPC timeout/error");
                std::array<std::uint8_t,193> packet{};
                const auto n=::recv(fd_,packet.data(),packet.size(),MSG_DONTWAIT);
                if (n<0) throw std::runtime_error("IMU receive failed");
                auto s=validate_imu_packet(packet.data(),static_cast<std::size_t>(n),monotonic_ns(),previous,software_fusion_);
                previous=s.sequence;
                { std::lock_guard<std::mutex> lock(mutex_);latest_=s; }
                std::this_thread::sleep_until(begin+std::chrono::milliseconds(5));
            }
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lock(mutex_);error_=e.what();
        }
    }
public:
    explicit NativeImuClient(const std::string& server,bool software_fusion=false):software_fusion_(software_fusion) {
        try {
            auto peer=address(server);
            char pattern[]="/tmp/sprite_policy_imu_XXXXXX";
            auto* dir=::mkdtemp(pattern);
            if (!dir) throw std::runtime_error("IMU temporary directory failed");
            directory_=dir;path_=directory_+"/rx.sock";
            auto local=address(path_);
            fd_=::socket(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0);
            if (fd_<0 || ::bind(fd_,reinterpret_cast<sockaddr*>(&local),sizeof(local)) ||
                ::connect(fd_,reinterpret_cast<sockaddr*>(&peer),sizeof(peer)))
                throw std::runtime_error("IMU socket setup failed");
            worker_=std::thread([this]{run();});
        } catch (...) { cleanup();throw; }
    }
    NativeImuClient(const NativeImuClient&)=delete;
    NativeImuClient& operator=(const NativeImuClient&)=delete;
    ~NativeImuClient() { stop_=true;if(worker_.joinable())worker_.join();cleanup(); }
    bool snapshot(ImuSnapshot& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!error_.empty()) throw std::runtime_error(error_);
        if (!latest_.sequence) return false;
        const auto now=monotonic_ns();
        if (now<latest_.start || now-latest_.start>30000000) throw std::runtime_error("IMU snapshot stale");
        result=latest_;return true;
    }
};
}
