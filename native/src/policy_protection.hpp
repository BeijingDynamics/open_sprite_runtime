#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace sprite_policy {
using ProtectionVector = std::array<double, 31>;
struct ImpedanceTarget {
    ProtectionVector position{}, velocity{}, kp{}, kd{}, feedforward{};
};
inline void require_finite(const ProtectionVector& v) {
    for (double x : v) if (!std::isfinite(x)) throw std::runtime_error("nonfinite target");
}
inline void validate_target(const ImpedanceTarget& t) {
    require_finite(t.position); require_finite(t.velocity);
    require_finite(t.kp); require_finite(t.kd); require_finite(t.feedforward);
    for (std::size_t i=0; i<31; ++i)
        if (t.kp[i]<0 || t.kd[i]<0) throw std::runtime_error("negative gain");
}
class TargetProjector {
    ProtectionVector lower_, upper_, gain_, damping_;
    double scale_, maximum_kd_;
public:
    std::array<std::uint64_t,31> clamp_count{};
    double maximum_overshoot=0;
    TargetProjector(ProtectionVector lower, ProtectionVector upper,
                    ProtectionVector gain, ProtectionVector damping,
                    double scale, double maximum_kd)
        : lower_(lower),upper_(upper),gain_(gain),damping_(damping),
          scale_(scale),maximum_kd_(maximum_kd) {
        require_finite(lower); require_finite(upper); require_finite(gain); require_finite(damping);
        if (!std::isfinite(scale) || scale<=0 || scale>1 ||
            !std::isfinite(maximum_kd) || maximum_kd<=0) throw std::runtime_error("invalid gain envelope");
        for (std::size_t i=0;i<31;++i)
            if (lower[i]>=upper[i] || gain[i]<=0 || gain[i]>1 || damping[i]<=0 || damping[i]>4)
                throw std::runtime_error("invalid projection configuration");
    }
    ProtectionVector require_position(const ProtectionVector& p) const {
        require_finite(p);
        for (std::size_t i=0;i<31;++i)
            if (p[i]<lower_[i] || p[i]>upper_[i]) throw std::runtime_error("startup pose outside limits");
        return p;
    }
    ImpedanceTarget project(const ImpedanceTarget& t) {
        validate_target(t);
        auto result=t;
        for (std::size_t i=0;i<31;++i) {
            result.position[i]=std::clamp(t.position[i],lower_[i],upper_[i]);
            if (result.position[i]!=t.position[i]) ++clamp_count[i];
            maximum_overshoot=std::max(maximum_overshoot,
                std::max(lower_[i]-t.position[i],t.position[i]-upper_[i]));
            const double s=scale_*gain_[i];
            result.kp[i]=s*t.kp[i];
            result.kd[i]=(s*damping_[i])*t.kd[i];
            result.feedforward[i]=s*t.feedforward[i];
            if (result.kd[i]>maximum_kd_) throw std::runtime_error("projected Kd exceeds qualified limit");
        }
        validate_target(result);
        return result;
    }
};
class StartupRamp {
    std::uint64_t hold_, ramp_, completed_=0;
    ProtectionVector initial_{}, previous_{};
    bool captured_=false, has_previous_=false;
public:
    double last_alpha=0, maximum_position_delta=0;
    StartupRamp(double hz,double hold_seconds,double ramp_seconds) {
        if (!std::isfinite(hz) || hz<=0 || !std::isfinite(hold_seconds) || hold_seconds<0 ||
            !std::isfinite(ramp_seconds) || ramp_seconds<=0)
            throw std::runtime_error("invalid startup timing");
        const double h=std::ceil(hz*hold_seconds), r=std::ceil(hz*ramp_seconds);
        // Bound counters before converting floating-point configuration to integers.
        if (!std::isfinite(h) || !std::isfinite(r) || h>1e9 || r>1e9)
            throw std::runtime_error("startup duration too large");
        hold_=static_cast<std::uint64_t>(h);
        ramp_=std::max<std::uint64_t>(1,static_cast<std::uint64_t>(r));
    }
    bool captured() const { return captured_; }
    bool complete() const { return completed_>=hold_+ramp_; }
    ImpedanceTarget apply(const ImpedanceTarget& t,const ProtectionVector& measured) {
        require_finite(measured); validate_target(t);
        if (!captured_) { initial_=measured; captured_=true; }
        double a=0;
        if (completed_>=hold_) {
            const double linear=std::min(1.0,static_cast<double>(completed_-hold_+1)/ramp_);
            a=linear*linear*(3-2*linear);
        }
        auto result=t;
        for (std::size_t i=0;i<31;++i) {
            result.position[i]=(1-a)*initial_[i]+a*t.position[i];
            result.velocity[i]=a*t.velocity[i]; result.kp[i]=a*t.kp[i];
            result.kd[i]=a*t.kd[i]; result.feedforward[i]=a*t.feedforward[i];
            if (has_previous_) maximum_position_delta=std::max(maximum_position_delta,
                std::abs(result.position[i]-previous_[i]));
        }
        validate_target(result);
        previous_=result.position; has_previous_=true; last_alpha=a; ++completed_;
        return result;
    }
};
}
