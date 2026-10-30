#pragma once
#include <array>
#include <string>
namespace h1_policy {
using Joints = std::array<float, 27>;
using Command = std::array<float, 3>;
using Observation = std::array<float, 92>;
// All joint arrays use the policy document order, not SDK motor order.
struct State {
  Joints q{}, dq{};
  std::array<float, 3> gyro{}, gravity{};
};
Joints defaults();
Joints from_sdk(const Joints&);
Joints to_sdk(const Joints&);
class Core {
public: Observation observe(const State&, const Command&);
Joints torque(const State&, const Joints&) const;
void set_last_action(
  const Joints& a
  ){
  last_ = a;
}
void reset_action(){
  last_ = {};
}
private: float phase_ = 0;
Joints last_{};
};
enum class Mode {
  Passive, FixStand, ArmHoldStand, ArmHold, Fault
};
Joints fix_stand_target();
class OrtPolicy {
public: explicit OrtPolicy(const std::string&);
~OrtPolicy();
OrtPolicy(const OrtPolicy&) = delete;
OrtPolicy& operator=(const OrtPolicy&) = delete;
Joints infer(const Observation&);
private: struct Impl;
Impl* impl_;
};
class Controller {
public: explicit Controller(const std::string& model);
bool transition(Mode, const State&, double now);
Joints step(const State&, const Command&, double now);
Mode mode() const {
  return mode_;
}
void fault() {
  mode_ = Mode::Fault;
}
private: Core core_;
OrtPolicy policy_;
Mode mode_ = Mode::Passive;
double entered_ = 0;
Joints start_{};
};
}
