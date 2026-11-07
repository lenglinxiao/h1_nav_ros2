#include <limits>
#include "h1_2_policy_bridge/core.hpp"
#include "check.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace h1_policy;
int main(int argc, char** argv){
 if (argc != 2) return 1;
 try {
  {
    State extreme;
    JointControl input;
    extreme.q[0] = 1e8f; extreme.dq[0] = 101.f;
    input.q[0] = extreme.q[0] + 32.f; input.kp[0] = 200.f; input.kd[0] = 2.f;
    bool impossible = false;
    try { project_position_pd(extreme, input); } catch (const std::invalid_argument&) { impossible = true; }
    CHECK(impossible);
    extreme.q[0] = 0; extreme.dq[0] = 1;
    input.q[0] = 1e30f; input.kp[0] = 1e-30f; input.kd[0] = 1e30f;
    impossible = false;
    try { project_position_pd(extreme, input); } catch (const std::invalid_argument&) { impossible = true; }
    CHECK(impossible);
  }
  const auto limits = torque_limits();
  State sample;
  JointControl desired;
  for (int i = 0; i < 27; ++i) {
    sample.q[i] = .2f; sample.dq[i] = 1.25f;
    desired.kp[i] = 10.f; desired.kd[i] = 2.f;
    desired.q[i] = sample.q[i] + (limits[i] + 12.5f) / desired.kp[i];
  }
  auto projected = project_position_pd(sample, desired);
  for (int i = 0; i < 27; ++i) {
    CHECK(projected.limited[i]);
    CHECK(std::abs(projected.control.tau[i] - limits[i]) < 1e-4f);
    CHECK(double(desired.kp[i]) * (double(projected.control.q[i]) - sample.q[i]) - double(desired.kd[i]) * sample.dq[i] <= limits[i]);
    CHECK(std::abs(desired.kp[i] * (projected.control.q[i] - sample.q[i]) - desired.kd[i] * sample.dq[i] - limits[i]) < 1e-3f);
    desired.q[i] = sample.q[i] + (-limits[i] - 12.5f) / desired.kp[i];
  }
  projected = project_position_pd(sample, desired);
  for (int i = 0; i < 27; ++i) {
    CHECK(projected.limited[i]);
    CHECK(std::abs(projected.control.tau[i] + limits[i]) < 1e-4f);
    CHECK(std::abs(desired.kp[i] * (projected.control.q[i] - sample.q[i]) - desired.kd[i] * sample.dq[i] + limits[i]) < 1e-3f);
    desired.q[i] = sample.q[i];
  }
  projected = project_position_pd(sample, desired);
  CHECK(projected.control.q == desired.q);
  for (bool limited : projected.limited) CHECK(!limited);
  desired.kp[23] = 0; desired.kd[23] = 100; desired.q[23] = 3;
  projected = project_position_pd(sample, desired);
  CHECK(projected.control.q[23] == 3 && !projected.limited[23]);
  CHECK(projected.control.tau[23] == -125);
  desired.kp[23] = 7.9f; desired.kd[23] = .5f; sample.q[23] = -.556579f;
  sample.dq[23] = -.112356f; desired.q[23] = -11.44f;
  projected = project_position_pd(sample, desired);
  CHECK(projected.limited[23]);
  CHECK(std::abs(projected.control.kp[23] * (projected.control.q[23] - sample.q[23]) - projected.control.kd[23] * sample.dq[23]) <= 18.0001f);
  auto mapped = to_sdk(projected.control.q);
  CHECK(mapped[24] == projected.control.q[25] && mapped[25] == projected.control.q[24]);
  JointControl from{}, to{};
  from.q.fill(1); from.kp.fill(2); from.kd.fill(3); from.tau.fill(4);
  to.q.fill(3); to.kp.fill(6); to.kd.fill(7); to.tau.fill(8);
  auto mid = blend_position_pd(from, to, .5f);
  CHECK(mid.q[0] == 2 && mid.kp[0] == 4 && mid.kd[0] == 5 && mid.tau[0] == 6);
  CHECK(blend_position_pd(from, to, 0).q == from.q);
  CHECK(blend_position_pd(from, to, 1).kp == to.kp);
  auto rejects = [](auto fn) { try { fn(); } catch (const std::invalid_argument&) { return true; } return false; };
  {
    State state{};
    state.q.fill(2.f);
    JointControl start{};
    start.q.fill(1.f); start.kp.fill(10.f); start.kd.fill(.1f); start.tau.fill(4.f);
    GuardedRelease release(start, state, 0);
    auto result = release.step(state, 0);
    CHECK(result.control.q == start.q && result.control.kd == start.kd && result.control.tau == start.tau);
    result = release.step(state, 1.5);
    CHECK(result.control.q[0] == 1.5f && result.control.kp[0] == 5.f);
    CHECK(result.control.kd[0] > start.kd[0] && result.control.tau[0] == 2.f);
    result = release.step(state, 3);
    CHECK(result.phase == GuardedReleasePhase::Settling && result.control.kp[0] == 0);
    CHECK(result.control.q == state.q && result.control.tau[0] == 0);
    CHECK(release.step(state, 3.49).phase == GuardedReleasePhase::Settling);
    CHECK(release.step(state, 3.5).phase == GuardedReleasePhase::DampingDown);
    result = release.step(state, 6.5);
    CHECK(result.done && result.control.kd == passive_damping_gains());
    GuardedRelease settling(start, state, 0);
    settling.step(state, 3);
    state.dq[26] = .1f;
    CHECK(settling.step(state, 3.49).phase == GuardedReleasePhase::Settling);
    state.dq[26] = 0;
    CHECK(settling.step(state, 3.5).phase == GuardedReleasePhase::Settling);
    CHECK(settling.step(state, 3.99).phase == GuardedReleasePhase::Settling);
    CHECK(settling.step(state, 4).phase == GuardedReleasePhase::DampingDown);
    GuardedRelease settling_fast(start, state, 0);
    settling_fast.step(state, 3);
    state.dq[26] = .5f;
    CHECK(settling_fast.step(state, 3.1).blocked);
    state.dq[26] = 0;
    CHECK(settling_fast.step(state, 3.6).blocked);
    JointControl pause_start = start;
    pause_start.kd[0] = passive_damping_gains()[0] + 2.f;
    GuardedRelease paused(pause_start, state, 0);
    paused.step(state, 3); paused.step(state, 3.5);
    auto before_pause = paused.step(state, 4.5);
    state.dq[0] = .2f;
    auto at_pause = paused.step(state, 5);
    CHECK(at_pause.phase == GuardedReleasePhase::Settling && at_pause.control.kd == before_pause.control.kd);
    CHECK(paused.step(state, 5.5).control.kd == before_pause.control.kd);
    state.dq[0] = 0;
    CHECK(paused.step(state, 6).phase == GuardedReleasePhase::Settling);
    CHECK(paused.step(state, 6.49).phase == GuardedReleasePhase::Settling);
    auto resumed = paused.step(state, 6.5);
    CHECK(resumed.phase == GuardedReleasePhase::DampingDown && resumed.control.kd == before_pause.control.kd);
    CHECK(paused.step(state, 7).control.kd[0] < before_pause.control.kd[0]);
    CHECK(paused.step(state, 10).blocked);
    GuardedRelease fast(start, state, 0);
    state.dq[26] = -.5f;
    auto fast_result = fast.step(state, 1);
    CHECK(fast_result.blocked && fast_result.block.reason == ReleaseBlockReason::Speed);
    CHECK(fast_result.block.policy_index == 26 && fast_result.block.dq == -.5f && fast_result.block.q == 2.f);
    CHECK(fast_result.block.max_abs_dq == .5f && fast_result.block.elapsed == 1);
    CHECK(fast_result.block.previous_phase == GuardedReleasePhase::Unloading);
    state.dq[26] = 0;
    auto latched = fast.step(state, 2);
    CHECK(latched.blocked && latched.block.reason == ReleaseBlockReason::Speed && latched.block.dq == -.5f && latched.block.elapsed == 1);
    GuardedRelease timeout(start, state, 0);
    CHECK(timeout.step(state, 10).block.reason == ReleaseBlockReason::Timeout);
    GuardedRelease both(start, state, 0);
    state.dq[5] = .6f;
    CHECK(both.step(state, 10).block.reason == ReleaseBlockReason::Both);
    state.dq[5] = 0;
    GuardedRelease late_fast(start, state, 0);
    late_fast.step(state, 3); late_fast.step(state, 3.5);
    state.dq[0] = .5f;
    CHECK(late_fast.step(state, 4).blocked);
    state.dq[0] = std::numeric_limits<float>::quiet_NaN();
    CHECK(rejects([&]{ GuardedRelease invalid(start, state, 0); }));
    state.dq[0] = 0;
    GuardedRelease invalid_time(start, state, 1);
    CHECK(rejects([&]{ invalid_time.step(state, .9); }));
    CHECK(rejects([&]{ invalid_time.step(state, std::numeric_limits<double>::infinity()); }));
    state.q[26] = std::numeric_limits<float>::infinity();
    CHECK(rejects([&]{ invalid_time.step(state, 2); }));
  }
  CHECK(rejects([&]{ blend_position_pd(from, to, -0.1f); }));
  CHECK(rejects([&]{ blend_position_pd(from, to, std::numeric_limits<float>::quiet_NaN()); }));
  to.q[0] = std::numeric_limits<float>::infinity();
  CHECK(rejects([&]{ blend_position_pd(from, to, .5f); }));
  desired.kp[0] = -1;
  CHECK(rejects([&]{ project_position_pd(sample, desired); }));
  desired.kp[0] = 1; desired.kd[0] = -1;
  CHECK(rejects([&]{ project_position_pd(sample, desired); }));
  desired.kd[0] = 1; desired.q[0] = std::numeric_limits<float>::infinity();
  CHECK(rejects([&]{ project_position_pd(sample, desired); }));
  Joints x{};
  for(int i = 0; i < 27; i++) x[i] = i;
  auto p = from_sdk(x);
  CHECK(p[17] == 18 && p[18] == 17 && p[24] == 25 && p[25] == 24);
  CHECK(to_sdk(p) == x);
  State s;
  s.q = defaults();
  s.gravity = {0.04f, 0, -1};
  Core c;
  auto o = c.observe(s, {0, 0, 0});
  CHECK(o[3] == 0.04f && o[5] == -1 && o[9] == 0 && o[10] == 0 && o[11] == 0);
  for(int i = 0; i < 9; i++) c.observe(s, {0, 0, 0});
  o = c.observe(s, {0.2f, 0, 0});
  CHECK(std::abs(o[9] - 0.743145f) < 0.001);
  Joints a{};
  a[0] = 1000;
  a[3] = -1000;
  auto t = c.torque(s, a);
  CHECK(t[0] == 200 && t[3] == -300);
  c.set_last_action(a);
  o = c.observe(s, {0, 0, 0});
  CHECK(o[65] == 1000);
  // Independently specified nonzero observations catch ordering and scaling errors.
  State sentinel;
  sentinel.q = defaults();
  sentinel.q[0] = .127f;
  sentinel.q[17] = .2f;
  sentinel.dq[26] = .7f;
  sentinel.gyro = {.31f, -.22f, .13f};
  sentinel.gravity = {.04f, 0, -1};
  Joints previous{};
  previous[26] = -.4f;
  Core raw;
  raw.set_last_action(previous);
  auto observed = raw.observe(sentinel, {.17f, -.08f, .33f});
  CHECK(std::abs(observed[0] - .31f) < 1e-6f);
  CHECK(std::abs(observed[1] + .22f) < 1e-6f);
  CHECK(std::abs(observed[2] - .13f) < 1e-6f);
  CHECK(std::abs(observed[6] - .17f) < 1e-6f);
  CHECK(std::abs(observed[7] + .08f) < 1e-6f);
  CHECK(std::abs(observed[8] - .33f) < 1e-6f);
  CHECK(std::abs(observed[11] - .1f) < 1e-6f);
  CHECK(std::abs(observed[28] - .2f) < 1e-6f);
  CHECK(std::abs(observed[64] - .7f) < 1e-6f);
  CHECK(std::abs(observed[91] + .4f) < 1e-6f);
  State pd;
  pd.q = defaults();
  pd.q[0] += .1f;
  pd.dq[0] = .5f;
  Joints action{};
  action[3] = 1;
  auto worked = raw.torque(pd, action);
  auto control = raw.control(pd, action);
  CHECK(std::abs(worked[0] + 16.f) < 1e-4f);
  CHECK(std::abs(worked[3] - 75.f) < 1e-4f);
  CHECK(control.tau == worked);
  CHECK(std::abs(control.q[3] - 1.275f) < 1e-5f && control.kp[3] == 200 && control.kd[3] == 4);
  s.q[0] = std::numeric_limits<float>::quiet_NaN();
  bool rejected = false;
  try {
    c.torque(s, a);
  } catch(const std::invalid_argument&) {
    rejected = true;
  }
  CHECK(rejected);
  s.q = defaults();
  s.gravity = {0, 0, -1};
  Controller prep(argv[1]);
  CHECK(prep.transition(Mode::FixStand, s, 0));
  auto halfway = prep.step_control(s, {0, 0, 0}, 1.5);
  CHECK(std::abs(halfway.q[3] - .7f) < 1e-5f);
  CHECK(std::abs(halfway.kp[3] - 157.7f) < 1e-4f && halfway.kd[3] == 10.1f);
  Controller train_prep(argv[1]);
  train_prep.set_training_stand_target(from_sdk(to_sdk(defaults())));
  CHECK(train_prep.transition(Mode::FixStand, s, 0));
  CHECK(train_prep.transition(Mode::ArmHoldStand, s, 3));
  const auto train_target = train_prep.step_control(s, {0, 0, 0}, 6).q;
  for (int i = 0; i < 27; ++i) CHECK(std::abs(train_target[i] - defaults()[i]) < 1e-6f);
  Controller legacy_prep(argv[1]);
  CHECK(legacy_prep.transition(Mode::FixStand, s, 0));
  CHECK(legacy_prep.transition(Mode::ArmHoldStand, s, 3));
  const auto legacy_sdk = to_sdk(legacy_prep.step_control(s, {0, 0, 0}, 6).q);
  CHECK(std::abs(legacy_sdk[24] - (-.076f)) < 1e-6f);
  for (double tolerance : {0.101, std::numeric_limits<double>::quiet_NaN()}) {
    bool invalid = false;
    try { Controller bad(argv[1], tolerance); } catch (const std::invalid_argument&) { invalid = true; }
    CHECK(invalid);
  }
  for (double tolerance : {.07, .10}) {
    Controller gate(argv[1], tolerance);
    CHECK(gate.transition(Mode::FixStand, s, 0));
    CHECK(!gate.transition(Mode::ArmHoldStand, s, 2.9));
    CHECK(gate.transition(Mode::ArmHoldStand, s, 3));
    s.q[3] = s.q[9] = .97087f;
    CHECK(!gate.transition(Mode::ArmHold, s, 5.9));
    s.gravity[2] = -.8f;
    CHECK(!gate.transition(Mode::ArmHold, s, 6));
    s.gravity[2] = -1.f;
    CHECK(gate.transition(Mode::ArmHold, s, 6) == (tolerance == .10));
    s.q[3] = s.q[9] = .9f;
  }
  Controller ctl(argv[1]);
  CHECK(!ctl.transition(Mode::ArmHold, s, 0));
  CHECK(ctl.transition(Mode::FixStand, s, 0));
  CHECK(!ctl.transition(Mode::ArmHoldStand, s, 2.9));
  CHECK(ctl.transition(Mode::ArmHoldStand, s, 3));
  s.q[3] = .5;
  CHECK(!ctl.transition(Mode::ArmHold, s, 6));
  s.q[3] = .9;
  CHECK(ctl.transition(Mode::ArmHold, s, 6));
  Controller early(argv[1]);
  Controller prepared(argv[1]);
  CHECK(early.transition(Mode::FixStand, s, 0));
  CHECK(prepared.transition(Mode::FixStand, s, 0));
  for (int i = 0; i < 5; ++i) prepared.step(s, {0, 0, 0}, i * .02);
  CHECK(early.transition(Mode::ArmHoldStand, s, 3));
  CHECK(prepared.transition(Mode::ArmHoldStand, s, 3));
  CHECK(early.transition(Mode::ArmHold, s, 6));
  CHECK(prepared.transition(Mode::ArmHold, s, 6));
  auto first = early.step(s, {.2, 0, 0}, 6.02);
  auto later = prepared.step(s, {.2, 0, 0}, 6.02);
  bool different = false;
  for (int i = 0; i < 27; ++i) different |= std::abs(first[i] - later[i]) > .001;
  CHECK(different);
  Controller baseline(argv[1]);
  Controller shadowed(argv[1]);
  Observation shadow_obs; Joints shadow_action, shadow_torque;
  for (int i = 0; i < 5; ++i) shadowed.shadow(s, shadow_obs, shadow_action, shadow_torque);
  CHECK(baseline.transition(Mode::FixStand, s, 0));
  CHECK(shadowed.transition(Mode::FixStand, s, 0));
  CHECK(baseline.transition(Mode::ArmHoldStand, s, 3));
  CHECK(shadowed.transition(Mode::ArmHoldStand, s, 3));
  CHECK(baseline.transition(Mode::ArmHold, s, 6));
  CHECK(shadowed.transition(Mode::ArmHold, s, 6));
  auto expected_torque = baseline.step(s, {0, 0, 0}, 6.02);
  auto shadow_torque_after = shadowed.step_control(s, {0, 0, 0}, 6.02).tau;
  for (int i = 0; i < 27; ++i) CHECK(std::abs(expected_torque[i] - shadow_torque_after[i]) < 1e-5f);
  expected_torque = baseline.step(s, {.2f, 0, 0}, 6.04);
  shadow_torque_after = shadowed.step_control(s, {.2f, 0, 0}, 6.04).tau;
  for (int i = 0; i < 27; ++i) CHECK(std::abs(expected_torque[i] - shadow_torque_after[i]) < 1e-5f);
  ctl.fault();
  CHECK(!ctl.transition(Mode::FixStand, s, 7));
  CHECK(ctl.transition(Mode::Passive, s, 7));
  CHECK(ctl.transition(Mode::FixStand, s, 7));
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
