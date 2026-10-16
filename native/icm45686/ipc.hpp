#pragma once
#include "icm_packet.hpp"
#include <atomic>
#include <mutex>
#include <thread>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>

class IcmServer {
  int fd_=-1;
  std::string path_;
  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex mutex_;
  sprite_imu::Packet latest_{};
  bool ready_=false;
  void run() noexcept {
    while(!stop_) {
      pollfd p{fd_,POLLIN,0};
      if(::poll(&p,1,10)!=1 || !(p.revents&POLLIN))continue;
      sockaddr_un peer{};socklen_t len=sizeof(peer);char request[9]{};
      auto n=::recvfrom(fd_,request,sizeof(request),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&peer),&len);
      if(n!=8 || std::memcmp(request,"GETIMU01",8))continue;
      sprite_imu::Packet packet;
      {std::lock_guard<std::mutex> lock(mutex_);if(!ready_)continue;packet=latest_;}
      // Slow/disconnected consumers cannot block the acquisition thread.
      ::sendto(fd_,packet.data(),packet.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&peer),len);
    }
  }
public:
  explicit IcmServer(const char* path) {
    if(!path || !*path)return;
    sockaddr_un a{};a.sun_family=AF_UNIX;
    if(std::strlen(path)>=sizeof(a.sun_path))throw std::runtime_error("ICM socket path too long");
    std::strcpy(a.sun_path,path);
    fd_=::socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(fd_<0)throw std::runtime_error("ICM socket creation failed");
    if(::bind(fd_,reinterpret_cast<sockaddr*>(&a),sizeof(a))) {
      ::close(fd_);fd_=-1;throw std::runtime_error("ICM socket bind failed; existing socket not removed");
    }
    path_=path;
    try {worker_=std::thread([this]{run();});}
    catch(...) {::close(fd_);::unlink(path);throw;}
  }
  ~IcmServer(){stop_=true;if(worker_.joinable())worker_.join();if(fd_>=0)::close(fd_);if(!path_.empty())::unlink(path_.c_str());}
  void publish(const sprite_imu::Packet& p){if(fd_<0)return;std::lock_guard<std::mutex> lock(mutex_);latest_=p;ready_=true;}
};
