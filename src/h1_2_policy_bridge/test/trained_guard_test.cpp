#include "h1_2_policy_bridge/trained_guard.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
using namespace h1_policy;
TrainedGuardConfig config() {
  TrainedGuardConfig c; c.joint_speed_limits.fill(1); c.pos_lower.fill(-1); c.pos_upper.fill(1); return c;
}
State state() { State s; s.gravity[2] = -1; return s; }
template<class F> void rejects(F f) { bool threw = false; try { f(); } catch (const std::exception&) { threw = true; } assert(threw); }
int main() {
  assert(std::string(sdk_joint_names()[1]) == "left_hip_pitch_joint");
  assert(std::string(sdk_joint_names()[17]) == "left_wrist_roll_joint");
  assert(std::string(sdk_joint_names()[25]) == "right_wrist_pitch_joint");
  auto c = config();
  for (int i = 0; i < 27; ++i) {
    auto s = state(); TrainedGuard g(c); g.arm(0);
    s.dq[i] = 1; assert(!g.observe(s, 1, 2));
    s.dq[i] = 1.01; assert(!g.observe(s, 2, 2.02)); assert(!g.observe(s, 3, 2.04));
    auto e = g.observe(s, 4, 2.06); assert(e && e->reason == TrainedGuardReason::JointSpeed && e->sdk_index == i && e->consecutive == 3);
    assert(g.observe(state(), 5, 2.08)); g.reset(); assert(!g.tripped());
    g.arm(0); s = state(); s.q[i] = 1; assert(!g.observe(s, 1, 0));
    s.q[i] = 1.01; assert(!g.observe(s, 2, .02)); assert(!g.observe(s, 3, .04));
    e = g.observe(s, 4, .06); assert(e && e->reason == TrainedGuardReason::JointPosition && e->sdk_index == i);
    s = state(); s.q[i] = .099f; assert(g.prearm(s).ready);
    s.q[i] = .1f; auto p = g.prearm(s); assert(!p.ready && p.sdk_index == i);
  }
  { auto boundary = config(); auto s = state(); s.q[0] = .1f;
    boundary.prearm_max_pos_error = double(s.q[0]);
    assert(!TrainedGuard(boundary).prearm(s).ready);
  }
  { TrainedGuard g(c); auto s = state(); g.arm(0); s.dq[0] = 1.5;
    assert(!g.observe(s, 1, 1.99)); assert(!g.observe(s, 2, 2)); assert(!g.observe(s, 2, 2.5));
    assert(!g.observe(s, 3, 2.52)); assert(g.observe(s, 4, 2.54));
  }
  { TrainedGuard g(c); auto s = state(); g.arm(0); s.dq[0] = 3;
    assert(!g.observe(s, 1, .01)); assert(!g.observe(s, 2, .02)); s.dq[0] = 0; assert(!g.observe(s, 3, .03));
    s.dq[0] = 3; assert(!g.observe(s, 4, .04)); assert(!g.observe(s, 5, .05)); assert(g.observe(s, 6, .06));
  }
  { TrainedGuard g(c); auto s = state(); g.arm(0); s.gyro = {2, 2, 0};
    assert(!g.observe(s, 1, 2)); assert(!g.observe(s, 2, 2.02)); auto e = g.observe(s, 3, 2.04); assert(e && e->reason == TrainedGuardReason::BaseAngularSpeed);
  }
  for (double roll : {-.3, .3}) for (double pitch : {-.25, .25}) {
    TrainedGuard g(c); auto s = state(); g.arm(0);
    // Quaternion gravity projection is independent of yaw.
    s.gravity = {float(std::sin(pitch)), float(-std::sin(roll)*std::cos(pitch)), float(-std::cos(roll)*std::cos(pitch))};
    assert(!g.observe(s, 1, .1)); assert(!g.observe(s, 1, .8)); assert(g.observe(s, 2, .81)->reason == TrainedGuardReason::Tilt);
    g.arm(1); assert(!g.observe(s, 1, 1)); auto upright = state(); assert(!g.observe(upright, 2, 1.4));
    assert(!g.observe(s, 3, 1.5)); assert(!g.observe(s, 4, 1.99)); assert(g.observe(s, 5, 2.01));
  }
  { auto bad = c; bad.pos_lower[3] = 2; rejects([&] { TrainedGuard g(bad); }); bad = c; bad.pos_upper[2] = -.1; rejects([&] { TrainedGuard g(bad); });
    bad = c; bad.joint_speed_limits[1] = std::numeric_limits<float>::quiet_NaN(); rejects([&] { TrainedGuard g(bad); });
    TrainedGuard g(c); g.arm(5); rejects([&] { g.observe(state(), 1, 4); });
  }
}
