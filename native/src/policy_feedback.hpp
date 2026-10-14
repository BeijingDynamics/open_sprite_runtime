#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

namespace sprite_policy {
// Coefficients are exported in recorded motor/contract joint order, not CAN ID order.
struct FeedbackMap {
  std::array<double,31> motor_zero{};
  std::array<double,961> motor_to_joint{};
  std::array<double,68> convert(const std::array<double,69>& input) const {
    for(double v:input) if(!std::isfinite(v)) throw std::runtime_error("nonfinite feedback");
    std::array<double,68> result{};
    for(int j=0;j<31;++j) for(int m=0;m<31;++m) {
      const auto coefficient=motor_to_joint[j*31+m];
      result[j]+=coefficient*(input[m]-motor_zero[m]);
      result[31+j]+=coefficient*input[31+m];
    }
    // P0 output axes: X down, Y right, Z rear. Policy: X front,Y left,Z up.
    result[62]=-input[64];result[63]=-input[63];result[64]=-input[62];
    double norm2=0;
    for(int i=65;i<69;++i) norm2+=input[i]*input[i];
    const double norm=std::sqrt(norm2);
    if(!std::isfinite(norm) || norm<1e-9) throw std::runtime_error("invalid quaternion");
    const double w=input[65]/norm,x=input[66]/norm,y=input[67]/norm,z=input[68]/norm;
    // -third row of R_world_sensor * R_sensor_body.
    result[65]=1-2*(x*x+y*y);
    result[66]=2*(y*z+x*w);
    result[67]=2*(x*z-y*w);
    for(double v:result) if(!std::isfinite(v)) throw std::runtime_error("feedback conversion overflow");
    return result;
  }
};
}
