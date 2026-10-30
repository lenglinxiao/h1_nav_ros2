#include "h1_2_policy_bridge/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace h1_policy {
Joints fix_stand_target(){
  return {0, -.2, 0, .5, -.3, 0, 0, -.2, 0, .5, -.3, 0, 0, .28, 0, 0, .52, 0, 0, 0, .28, 0, 0, .52, 0, 0, 0};
}
static Joints stand_target(){
  auto a = defaults();
  Joints arms = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, .270, .001, 0, .483, -.003, -.030, -.002, -.239, -.192, .941, -.633, .004, -.076, .026};
  for(int i = 13; i < 27; i++) a[i] = arms[i];
  return a;
}
static bool valid(
  const State&s
  ){
  for(auto a:{s.q, s.dq}) for(float x:a) if(!std::isfinite(x)) return false;
  for(auto a:{s.gyro, s.gravity}) for(float x:a) if(!std::isfinite(x)) return false;
  return true;
}
Controller::Controller(
  const std::string& path
  ) : policy_(path){
}
bool Controller::transition(
  Mode next, const State&s, double now
  ){
  if(!valid(s)||!std::isfinite(now)) return false;
  if(next == Mode::Passive) {
    mode_ = next;
    entered_ = now;
    return true;
  }
  if(mode_ == Mode::Fault) return false;
  if(next == Mode::FixStand && mode_ == Mode::Passive) {
  }else if(next == Mode::ArmHoldStand && mode_ == Mode::FixStand && now - entered_ >= 3) {
  }else if(next == Mode::ArmHold && mode_ == Mode::ArmHoldStand && now - entered_ >= 3 && std::abs(s.q[3] - .9f) < .07f && std::abs(s.q[9] - .9f) < .07f && s.gravity[2] < -.9f) {
    core_.reset_action();
  }else return false;
  start_ = s.q;
  entered_ = now;
  mode_ = next;
  return true;
}
Joints Controller::step(
  const State&s, const Command&cmd, double now
  ){
  if(!valid(s)||!std::isfinite(now)||!std::all_of(cmd.begin(), cmd.end(), [](float x){
      return std::isfinite(x);
    })) {
    fault();
    throw std::runtime_error("invalid control input");
  }
  if(mode_ == Mode::Fault) throw std::runtime_error("controller fault");
  if(mode_ == Mode::ArmHold) {
    try{
      auto obs = core_.observe(s, cmd);
      auto a = policy_.infer(obs);
      auto t = core_.torque(s, a);
      core_.set_last_action(a);
      return t;
    }catch(...) {
      fault();
      throw;
    }
  }
// Advance the gait clock during preparation and passive control too.
  core_.observe(s, cmd);
  const Joints damp = {6.3, 6.3, 6.3, 10.1, 1.3, 1.3, 6.3, 6.3, 6.3, 10.1, 1.3, 1.3, 6.3, 1.3, 1.3, .5, .5, .5, .5, .5, 1.3, 1.3, .5, .5, .5, .5, .5};
  const Joints lim = {200, 200, 200, 300, 40, 40, 200, 200, 200, 300, 40, 40, 200, 40, 40, 18, 18, 19, 19, 19, 40, 40, 18, 18, 19, 19, 19};
  Joints kp = {98.7, 98.7, 98.7, 157.7, 19.7, 19.7, 98.7, 98.7, 98.7, 157.7, 19.7, 19.7, 98.7, 19.7, 19.7, 7.9, 7.9, 7.9, 7.9, 7.9, 19.7, 19.7, 7.9, 7.9, 7.9, 7.9, 7.9};
  Joints kd = damp;
  if(mode_ == Mode::ArmHoldStand) {
    kp = {98.7, 98.7, 98.7, 157.7, 100, 100, 98.7, 98.7, 98.7, 157.7, 100, 100, 98.7, 100, 100, 60, 120, 40, 40, 40, 100, 100, 60, 120, 40, 40, 40};
    kd = {6.3, 6.3, 6.3, 10.1, 4, 4, 6.3, 6.3, 6.3, 10.1, 4, 4, 6.3, 6, 6, 3, 4, 2, 2, 2, 6, 6, 3, 4, 2, 2, 2};
  }
  auto target = mode_ == Mode::FixStand?fix_stand_target():stand_target();
  float f = std::clamp(float((now - entered_) / 3.), 0.f, 1.f);
  Joints t{};
  for(int i = 0; i < 27; i++) {
    float desired = start_[i] + f * (target[i] - start_[i]);
    t[i] = std::clamp((mode_ == Mode::Passive?0.f:kp[i] * (desired - s.q[i])) - kd[i] * s.dq[i], -lim[i], lim[i]);
  } return t;
}
}
