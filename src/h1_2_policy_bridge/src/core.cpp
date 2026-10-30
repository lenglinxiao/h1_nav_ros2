#include "h1_2_policy_bridge/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace h1_policy {
namespace {
template<class A> void require_finite(
  const A& values
  ) {
  for (float value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("non-finite control input");
  }
}
void validate(
  const State& state
  ) {
  require_finite(state.q);
  require_finite(state.dq);
  require_finite(state.gyro);
  require_finite(state.gravity);
}
}

Joints defaults(){
  return {0.027, -0.45, 0, 0.9, -0.45, 0, -0.027, -0.45, 0, 0.9, -0.475, 0, 0, 0.28, 0, 0, 0.52, 0, 0, 0, -0.20, -0.23, 0.90, -0.55, 0, 0, 0};
}
// SDK order is wrist roll, pitch; the supplied policy guide says pitch, roll.
Joints from_sdk(
  const Joints& a
  ){
  auto b = a;
  std::swap(b[17], b[18]);
  std::swap(b[24], b[25]);
  return b;
}
Joints to_sdk(
  const Joints& a
  ){
  return from_sdk(a);
}
Observation Core::observe(
  const State& s, const Command& cmd
  ){
  validate(s);
  require_finite(cmd);
  Observation o{};
  phase_ = std::fmod(phase_ + 0.02f / 0.6f, 1.f);
  for(int i = 0; i < 3; i++) {
    o[i] = s.gyro[i];
    o[i + 3] = s.gravity[i];
    o[i + 6] = cmd[i];
  } if(std::hypot(cmd[0], cmd[1], cmd[2]) >= 0.1f) {
    o[9] = std::sin(phase_ * 6.28318530718f);
    o[10] = std::cos(phase_ * 6.28318530718f);
  }
  auto d = defaults();
  for(int i = 0; i < 27; i++) {
    o[11 + i] = s.q[i] - d[i];
    o[38 + i] = s.dq[i];
    o[65 + i] = last_[i];
  } return o;
}
Joints Core::torque(
  const State&s, const Joints&a
  ) const {
  validate(s);
  require_finite(a);
  const Joints kp = {150, 150, 150, 200, 40, 40, 150, 150, 150, 200, 40, 40, 300, 19.7, 19.7, 7.9, 7.9, 7.9, 7.9, 7.9, 19.7, 19.7, 7.9, 7.9, 7.9, 7.9, 7.9};
  const Joints kd = {2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 6, 1.3, 1.3, .5, .5, .5, .5, .5, 1.3, 1.3, .5, .5, .5, .5, .5};
  const Joints scale = {.3333, .3333, .3333, .375, .25, .25, .3333, .3333, .3333, .375, .25, .25, .1667, .5076, .5076, .5696, .5696, .5696, .5696, .5696, .5076, .5076, .5696, .5696, .5696, .5696, .5696};
  const Joints lim = {200, 200, 200, 300, 40, 40, 200, 200, 200, 300, 40, 40, 200, 40, 40, 18, 18, 19, 19, 19, 40, 40, 18, 18, 19, 19, 19};
  auto d = defaults();
  Joints t{};
  for(int i = 0; i < 27; i++) t[i] = std::clamp(kp[i] * (d[i] + scale[i] * a[i] - s.q[i]) - kd[i] * s.dq[i], -lim[i], lim[i]);
  return t;
}
}
