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
struct JointControl {
  Joints q{}, kp{}, kd{}, tau{};
};
// Projection limits ideal PD torque only at the sampled state; it does not
// guarantee motor torque between samples or enforce a firmware limit. kp=0 is
// left unprojected, so native damping is not torque limited by this helper.
struct PositionPdProjection {
  JointControl control;
  std::array<bool, 27> limited{};
};
Joints torque_limits();
PositionPdProjection project_position_pd(const State&, const JointControl&);
JointControl blend_position_pd(const JointControl&, const JointControl&, float progress);
Joints defaults();
Joints from_sdk(const Joints&);
Joints to_sdk(const Joints&);
class Core {
public: Observation observe(const State&, const Command&);
JointControl control(const State&, const Joints&) const;
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
Joints passive_damping_gains();
inline constexpr float kReleaseBlockSpeed = .5f;
inline constexpr float kReleaseQuietSpeed = .1f;
enum class GuardedReleasePhase { Unloading, Settling, DampingDown, Blocked, Done };
enum class ReleaseBlockReason { None, Speed, Timeout, Both };
struct ReleaseBlockInfo {
  ReleaseBlockReason reason = ReleaseBlockReason::None;
  GuardedReleasePhase previous_phase = GuardedReleasePhase::Unloading;
  int policy_index = -1;
  float q = 0, dq = 0, max_abs_dq = 0;
  double elapsed = 0;
};
struct GuardedReleaseResult {
  JointControl control;
  GuardedReleasePhase phase;
  bool blocked;
  bool done;
  ReleaseBlockInfo block;
};
class GuardedRelease {
public:
  GuardedRelease(const JointControl& from, const State& start_state, double now);
  GuardedReleaseResult step(const State& state, double now);
private:
  JointControl from_{}, protected_{}, passive_{}, output_{};
  GuardedReleasePhase phase_ = GuardedReleasePhase::Unloading;
  double started_, last_time_, settling_since_ = 0, damping_since_ = 0;
  bool settling_active_ = false;
  ReleaseBlockInfo block_{};
};
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
public: explicit Controller(const std::string& model, double knee_tolerance = .07);
void set_training_stand_target(const Joints& target) { training_stand_target_ = target; use_training_stand_target_ = true; }
bool transition(Mode, const State&, double now, bool check_knees = true);
Joints step(const State&, const Command&, double now);
JointControl step_control(const State&, const Command&, double now);
void shadow(const State&, Observation&, Joints& action, Joints& torque);
Mode mode() const {
  return mode_;
}
void fault() {
  mode_ = Mode::Fault;
}
private: double knee_tolerance_;
bool use_training_stand_target_ = false;
Joints training_stand_target_{};
Core core_;
Core shadow_core_;
OrtPolicy policy_;
Mode mode_ = Mode::Passive;
double entered_ = 0;
Joints start_{};
};
}
