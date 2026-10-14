#include "policy_protection.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

// Offline text fixture runner. No transport or hardware interfaces are linked.
int main(int argc,char** argv) {
    try {
        if (argc!=3) throw std::runtime_error("usage: sprite_protection_replay INPUT OUTPUT");
        std::ifstream in(argv[1]); std::ofstream out(argv[2]);
        if (!in || !out) throw std::runtime_error("file open failed");
        auto scalar=[&]() { double x;
            if (!(in>>x) || !std::isfinite(x)) throw std::runtime_error("invalid input scalar");
            return x; };
        auto vector=[&]() { sprite_policy::ProtectionVector v{};
            for (auto& x:v) x=scalar();
            return v; };
        std::string magic; std::uint64_t count;
        if (!(in>>magic>>count) || magic!="SPPROT01" || count==0 || count>1000000)
            throw std::runtime_error("invalid fixture header");
        auto lower=vector(),upper=vector(),gain=vector(),damping=vector();
        double scale=scalar(),kdmax=scalar(),hz=scalar(),hold=scalar(),ramp_seconds=scalar();
        sprite_policy::TargetProjector projector(lower,upper,gain,damping,scale,kdmax);
        sprite_policy::StartupRamp ramp(hz,hold,ramp_seconds);
        out<<std::setprecision(17);
        for (std::uint64_t n=0;n<count;++n) {
            sprite_policy::ImpedanceTarget t;
            t.position=vector();t.velocity=vector();t.kp=vector();t.kd=vector();t.feedforward=vector();
            auto measured=vector();
            t=projector.project(t);
            if (!ramp.captured()) measured=projector.require_position(measured);
            t=ramp.apply(t,measured);
            for (const auto* v:{&t.position,&t.velocity,&t.kp,&t.kd,&t.feedforward})
                for (double x:*v) out<<x<<' ';
            out<<ramp.last_alpha<<'\n';
        }
        in>>std::ws;
        if (!in.eof()) throw std::runtime_error("trailing input");
        if (!out) throw std::runtime_error("output write failed");
        std::cout<<"PROTECTION_REPLAY frames="<<count<<" hardware_tx=0\n";
        return 0;
    } catch (const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
