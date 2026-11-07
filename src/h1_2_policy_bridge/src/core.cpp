#include "h1_2_policy_bridge/core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
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

Joints torque_limits() {
  return {200, 200, 200, 300, 40, 40, 200, 200, 200, 300, 40, 40, 200,
          40, 40, 18, 18, 19, 19, 19, 40, 40, 18, 18, 19, 19, 19};
}
namespace {
void validate_control(const JointControl& c) {
  require_finite(c.q); require_finite(c.kp); require_finite(c.kd); require_finite(c.tau);
  for (int i = 0; i < 27; ++i) {
    if (c.kp[i] < 0 || c.kd[i] < 0) throw std::invalid_argument("negative PD gain");
  }
}
}
PositionPdProjection project_position_pd(const State& s, const JointControl& desired) {
  validate(s);
  validate_control(desired);
  PositionPdProjection result{desired, {}};
  const auto limits = torque_limits();
  for (int i = 0; i < 27; ++i) {
    const double kp = desired.kp[i], kd = desired.kd[i];
    auto sample_tau = [&](float q) {
      return kp * (double(q) - s.q[i]) - kd * s.dq[i];
    };
    const double tau = sample_tau(desired.q[i]);
    if (kp == 0) {
      result.control.tau[i] = static_cast<float>(tau);
    } else if (tau >= -limits[i] && tau <= limits[i]) {
      result.control.tau[i] = static_cast<float>(tau);
    } else {
      result.limited[i] = true;
      const double clipped = std::clamp(tau, -double(limits[i]), double(limits[i]));
      float q = static_cast<float>(double(s.q[i]) + (clipped + kd * s.dq[i]) / kp);
      if (!std::isfinite(q)) throw std::invalid_argument("PD projection target is not finite");
      double actual = sample_tau(q);
      for (int attempt = 0; attempt < 2 && (actual < -limits[i] || actual > limits[i]); ++attempt) {
        q = std::nextafter(q, actual > limits[i] ? -std::numeric_limits<float>::infinity()
                                                 : std::numeric_limits<float>::infinity());
        if (!std::isfinite(q)) throw std::invalid_argument("PD projection target is not finite");
        actual = sample_tau(q);
      }
      if (actual < -limits[i] || actual > limits[i])
        throw std::invalid_argument("PD projection not representable within torque limit");
      result.control.q[i] = q;
      result.control.tau[i] = static_cast<float>(actual);
    }
  }
  validate_control(result.control);
  return result;
}
JointControl blend_position_pd(const JointControl& a, const JointControl& b, float progress) {
  validate_control(a); validate_control(b);
  if (!std::isfinite(progress) || progress < 0 || progress > 1)
    throw std::invalid_argument("invalid blend progress");
  if (progress == 0) return a;
  if (progress == 1) return b;
  JointControl out{};
  for (int i = 0; i < 27; ++i) {
    auto blend = [progress](float x, float y) {
      return static_cast<float>((1.0 - progress) * double(x) + progress * double(y));
    };
    out.q[i] = blend(a.q[i], b.q[i]);
    out.kp[i] = blend(a.kp[i], b.kp[i]);
    out.kd[i] = blend(a.kd[i], b.kd[i]);
    out.tau[i] = blend(a.tau[i], b.tau[i]);
  }
  validate_control(out);
  return out;
}
GuardedRelease::GuardedRelease(const JointControl& from, const State& start_state, double now)
    : from_(from), protected_(from), passive_(from), output_(from), started_(now), last_time_(now) {
  validate_control(from);
  validate(start_state);
  if (!std::isfinite(now)) throw std::invalid_argument("invalid release time");
  const auto passive_kd = passive_damping_gains();
  protected_.q = start_state.q;
  protected_.kp.fill(0);
  protected_.tau.fill(0);
  for (std::size_t i = 0; i < protected_.kd.size(); ++i)
    protected_.kd[i] = std::max(from.kd[i], passive_kd[i]);
  passive_ = protected_;
  passive_.kd = passive_kd;
}
GuardedReleaseResult GuardedRelease::step(const State& state, double now) {
  validate(state);
  if (!std::isfinite(now) || now < last_time_)
    throw std::invalid_argument("invalid release time");
  last_time_ = now;
  if (phase_ != GuardedReleasePhase::Blocked && phase_ != GuardedReleasePhase::Done) {
    const bool too_fast = std::any_of(state.dq.begin(), state.dq.end(),
        [](float dq) { return std::abs(dq) >= kReleaseBlockSpeed; });
    const bool settled = std::all_of(state.dq.begin(), state.dq.end(),
        [](float dq) { return std::abs(dq) < kReleaseQuietSpeed; });
    const bool timeout = now - started_ >= 10.;
    if (too_fast || timeout) {
      const auto max_it = std::max_element(state.dq.begin(), state.dq.end(),
          [](float a, float b) { return std::abs(a) < std::abs(b); });
      const int index = static_cast<int>(max_it - state.dq.begin());
      block_ = {too_fast && timeout ? ReleaseBlockReason::Both :
                    too_fast ? ReleaseBlockReason::Speed : ReleaseBlockReason::Timeout,
                phase_, index, state.q[index], state.dq[index], std::abs(state.dq[index]), now - started_};
      phase_ = GuardedReleasePhase::Blocked;
    } else if (phase_ == GuardedReleasePhase::Unloading) {
      const double elapsed = now - started_;
      output_ = blend_position_pd(from_, protected_, static_cast<float>(std::min(elapsed / 3., 1.)));
      if (elapsed >= 3.) {
        phase_ = GuardedReleasePhase::Settling;
        settling_since_ = now;
        settling_active_ = settled;
      }
    } else if (phase_ == GuardedReleasePhase::Settling) {
      if (!settled) {
        settling_active_ = false;
      } else if (!settling_active_) {
        settling_since_ = now;
        settling_active_ = true;
      } else if (now - settling_since_ >= .5) {
        phase_ = GuardedReleasePhase::DampingDown;
        damping_since_ = now;
      }
    } else if (phase_ == GuardedReleasePhase::DampingDown) {
      if (!settled) {
        protected_ = output_;
        phase_ = GuardedReleasePhase::Settling;
        settling_active_ = false;
      } else {
        const double elapsed = now - damping_since_;
        output_ = blend_position_pd(protected_, passive_, static_cast<float>(std::min(elapsed / 3., 1.)));
        if (elapsed >= 3.) phase_ = GuardedReleasePhase::Done;
      }
    }
  }
  return {output_, phase_, phase_ == GuardedReleasePhase::Blocked, phase_ == GuardedReleasePhase::Done, block_};
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
JointControl Core::control(
  const State&s, const Joints&a
  ) const {
  validate(s);
  require_finite(a);
  const Joints kp = {150, 150, 150, 200, 40, 40, 150, 150, 150, 200, 40, 40, 300, 19.7, 19.7, 7.9, 7.9, 7.9, 7.9, 7.9, 19.7, 19.7, 7.9, 7.9, 7.9, 7.9, 7.9};
  const Joints kd = {2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 6, 1.3, 1.3, .5, .5, .5, .5, .5, 1.3, 1.3, .5, .5, .5, .5, .5};
  const Joints scale = {.3333, .3333, .3333, .375, .25, .25, .3333, .3333, .3333, .375, .25, .25, .1667, .5076, .5076, .5696, .5696, .5696, .5696, .5696, .5076, .5076, .5696, .5696, .5696, .5696, .5696};
  const Joints lim = torque_limits();
  auto d = defaults();
  JointControl out{};
  out.kp = kp; out.kd = kd;
  for(int i = 0; i < 27; i++) {
    out.q[i] = d[i] + scale[i] * a[i];
    out.tau[i] = std::clamp(kp[i] * (out.q[i] - s.q[i]) - kd[i] * s.dq[i], -lim[i], lim[i]);
  }
  require_finite(out.q);
  return out;
}
Joints Core::torque(const State& s, const Joints& a) const { return control(s, a).tau; }
}
