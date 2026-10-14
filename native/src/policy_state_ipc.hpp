#pragma once
#include "policy_protection.hpp"
#include <cstring>
#include <cerrno>
#include <limits>
#include <string>
#include <unordered_set>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>

namespace sprite_policy {
static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559,"IPC requires IEEE binary64");
inline std::uint64_t ipc_u64(const unsigned char* p) {
    std::uint64_t v=0;for(unsigned i=0;i<8;++i)v|=std::uint64_t(p[i])<<(8*i);return v;
}
inline void ipc_put64(unsigned char* p,std::uint64_t v) {
    for(unsigned i=0;i<8;++i)p[i]=static_cast<unsigned char>(v>>(8*i));
}
inline std::uint64_t name_hash(const std::array<std::string,31>& names) {
    std::unordered_set<std::string> seen;std::uint64_t h=0xcbf29ce484222325ULL;
    for(const auto& name:names) {
        if(name.empty() || name.find('\0')!=std::string::npos || !seen.insert(name).second)
            throw std::runtime_error("invalid IPC name order");
        for(unsigned char c:name) {h^=c;h*=0x100000001b3ULL;}
        h*=0x100000001b3ULL;
    }
    return h;
}
struct PolicyState {
    std::uint64_t sequence=0, stamp=0;
    ProtectionVector position{},velocity{};
};
inline void state_fresh(const PolicyState& s,std::uint64_t now,std::uint64_t maximum_age) {
    if(!maximum_age || !s.stamp || s.stamp>now || now-s.stamp>maximum_age)
        throw std::runtime_error("state timestamp stale/future");
}
inline PolicyState decode_state(const unsigned char* p,std::size_t n,std::uint64_t hash,
                                std::uint64_t previous,std::uint64_t now,std::uint64_t age) {
    if(n!=528 || std::memcmp(p,"SPRT\1\0\1\0",8) || ipc_u64(p+24)!=hash)
        throw std::runtime_error("state ABI/order mismatch");
    PolicyState s;s.sequence=ipc_u64(p+8);s.stamp=ipc_u64(p+16);
    if(s.sequence<=previous)throw std::runtime_error("state sequence did not increase");
    state_fresh(s,now,age);
    for(std::size_t i=0;i<62;++i) {
        auto bits=ipc_u64(p+32+8*i);double x;std::memcpy(&x,&bits,8);
        if(!std::isfinite(x))throw std::runtime_error("state nonfinite value");
        (i<31?s.position[i]:s.velocity[i-31])=x;
    }
    return s;
}
inline std::array<unsigned char,1280> encode_target(const ImpedanceTarget& t,
        std::uint64_t sequence,std::uint64_t stamp,std::uint64_t hash,std::uint64_t source) {
    validate_target(t);
    if(!sequence || !stamp || stamp>std::uint64_t(std::numeric_limits<std::int64_t>::max()) || !source)
        throw std::runtime_error("invalid target metadata");
    std::array<unsigned char,1280> p{};std::memcpy(p.data(),"SPRT\1\0\2\0",8);
    ipc_put64(p.data()+8,sequence);ipc_put64(p.data()+16,stamp);
    ipc_put64(p.data()+24,hash);ipc_put64(p.data()+32,source);
    std::size_t offset=40;
    for(const auto* v:{&t.position,&t.velocity,&t.kp,&t.kd,&t.feedforward})
        for(double x:*v) {std::uint64_t bits;std::memcpy(&bits,&x,8);ipc_put64(p.data()+offset,bits);offset+=8;}
    return p;
}
class PolicyStateChannel {
    int fd_=-1;
    std::uint64_t motor_hash_,joint_hash_,max_age_,last_state_=0,last_target_=0;
    bool failed_=false;
    int startup_wait_ms_;
public:
    PolicyStateChannel(const std::string& path,std::uint64_t motor_hash,std::uint64_t joint_hash,
                       std::uint64_t maximum_age_ns,int startup_wait_ms=100)
        :motor_hash_(motor_hash),joint_hash_(joint_hash),max_age_(maximum_age_ns),startup_wait_ms_(startup_wait_ms) {
        sockaddr_un address{};address.sun_family=AF_UNIX;
        if(path.empty() || path.size()>=sizeof(address.sun_path) || !max_age_ || startup_wait_ms<1 || startup_wait_ms>2000)
            throw std::runtime_error("invalid state channel configuration");
        std::memcpy(address.sun_path,path.c_str(),path.size()+1);
        fd_=::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC|SOCK_NONBLOCK,0);
        if(fd_<0 || ::connect(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))) {
            if(fd_>=0)::close(fd_);
            fd_=-1;throw std::runtime_error("state connect failed");
        }
    }
    ~PolicyStateChannel(){if(fd_>=0)::close(fd_);}
    void close_after_fault() noexcept {
        failed_=true;
        if(fd_>=0) { ::close(fd_);fd_=-1; }
    }
    PolicyStateChannel(const PolicyStateChannel&)=delete;
    PolicyStateChannel& operator=(const PolicyStateChannel&)=delete;
    template<class Clock> PolicyState receive(Clock clock) {
        if(failed_)throw std::runtime_error("state channel fault latched");
        try {
            pollfd p{fd_,POLLIN,0};
            if(::poll(&p,1,last_state_?100:startup_wait_ms_)!=1 || !(p.revents&POLLIN))throw std::runtime_error("state receive timeout/error");
            std::array<unsigned char,529> bytes{};
            auto n=::recv(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT);
            if(n<=0)throw std::runtime_error("state disconnected/receive failed");
            auto state=decode_state(bytes.data(),static_cast<std::size_t>(n),motor_hash_,last_state_,clock(),max_age_);
            last_state_=state.sequence;return state;
        } catch(...) {failed_=true;throw;}
    }
    void send(const ImpedanceTarget& target,const PolicyState& state,std::uint64_t now) {
        if(failed_)throw std::runtime_error("state channel fault latched");
        try {
            if(state.sequence!=last_state_)throw std::runtime_error("target source state mismatch");
            state_fresh(state,now,max_age_);
            auto packet=encode_target(target,last_target_+1,now,joint_hash_,state.sequence);
            if(::send(fd_,packet.data(),packet.size(),MSG_DONTWAIT|MSG_NOSIGNAL)!=static_cast<ssize_t>(packet.size()))
                throw std::runtime_error("target send failed/backpressure");
            ++last_target_;
        } catch(...) {failed_=true;throw;}
    }
    void await_controller_close(int timeout_ms=1000, bool allow_terminal_reset=false) {
        if(failed_)throw std::runtime_error("state channel fault latched");
        if(timeout_ms<1 || timeout_ms>2000)throw std::runtime_error("invalid completion timeout");
        // Transport completion only: the supervisor must also check the CAN report.
        // No targets are sent while the controller verifies disabled feedback.
        pollfd p{fd_,POLLIN,0};
        if(::poll(&p,1,timeout_ms)!=1)throw std::runtime_error("controller close timeout");
        unsigned char byte;
        const auto received=::recv(fd_,&byte,1,MSG_DONTWAIT);
        // Only a completed powered schedule may reach this branch. The controller
        // can close with its last reply unread; supervisor still verifies disable.
        if(received<0 && errno==ECONNRESET && allow_terminal_reset)return;
        if(received!=0)
            throw std::runtime_error("controller completion recv="+std::to_string(received)+
                " errno="+std::to_string(received<0?errno:0));
    }
    void finish_shadow() { await_controller_close(); }
};
}
