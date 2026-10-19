#pragma once
#include "policy_gamepad.hpp"
#include <linux/joystick.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace sprite_policy {
inline std::uint64_t gamepad_now(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
// All device IO stays off the policy inference thread. No event-age timeout:
// Linux joystick events are change-driven even when a stick is held steadily.
class LinuxGamepad {
  std::atomic<bool> stop_{false};std::mutex mutex_;GamepadSnapshot latest_{};
  std::thread worker_;
  void publish(GamepadSnapshot s){std::lock_guard<std::mutex> lock(mutex_);latest_=s;}
  void run(const std::string& path) {
    GamepadPermission permission;
    while(!stop_) {
      const int fd=open(path.c_str(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);
      if(fd<0){publish({});std::this_thread::sleep_for(std::chrono::milliseconds(100));continue;}
      unsigned char axes=0;
      bool good=ioctl(fd,JSIOCGAXES,&axes)==0 && axes>=6;
      permission.disconnect();
      while(good && !stop_) {
        pollfd p{fd,POLLIN,0};const int rc=poll(&p,1,10);
        if(rc<0 || p.revents&(POLLERR|POLLHUP|POLLNVAL)){good=false;break;}
        for(int n=0;n<256 && rc>0;++n) {
          js_event event{};const auto size=read(fd,&event,sizeof(event));
          if(size<0 && (errno==EAGAIN||errno==EWOULDBLOCK))break;
          if(size!=sizeof(event)){good=false;break;}
          if(event.type&JS_EVENT_AXIS)permission.event(event.number,std::clamp(event.value/32767.,-1.,1.),event.type&JS_EVENT_INIT);
        }
        int version=0;
        if(!good || ioctl(fd,JSIOCGVERSION,&version)<0){good=false;break;}
        publish(permission.snapshot(gamepad_now()));
      }
      permission.disconnect();publish({});close(fd);
    }
  }
public:
  explicit LinuxGamepad(const std::string& path):worker_([this,path]{try{run(path);}catch(...){publish({});}}){}
  ~LinuxGamepad(){stop_=true;if(worker_.joinable())worker_.join();}
  GamepadSnapshot snapshot(){std::lock_guard<std::mutex> lock(mutex_);return latest_;}
};
}
