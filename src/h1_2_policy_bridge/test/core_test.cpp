#include "h1_2_policy_bridge/core.hpp"
#include "check.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace h1_policy;
int main(int argc, char** argv){
 if (argc != 2) return 1;
 try {
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
