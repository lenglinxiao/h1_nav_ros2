#include "h1_2_policy_bridge/hardware.hpp"
#include "check.hpp"
#include <limits>
int main() {
  using namespace h1_policy;
  CHECK(valid_profile("sim", "lo", 42));
  CHECK(valid_profile("hardware", "eth0", 0));
  CHECK(!valid_profile("hardware", "lo", 0));
  // A fresh incoming stream must not make a command computed from an old
  // snapshot publishable. The hardware control period is 20 ms.
  CHECK(command_snapshot_fresh(1.0, 1.019));
  CHECK(!command_snapshot_fresh(1.0, 1.021));
  CHECK(!command_snapshot_fresh(1.0, .999));
  struct Fixture { uint32_t data; uint32_t checksum; };
  Fixture fixture{0, 0xc704dd7b};
  CHECK(valid_crc(fixture, fixture.checksum));
  fixture.data ^= 1; CHECK(!valid_crc(fixture, fixture.checksum));
  State state; state.gravity = {0, 0, -1};
  CHECK(valid_low_state(state, {1, 0, 0, 0}, 6, 0, 6, true));
  CHECK(!valid_low_state(state, {1, 0, 0, 0}, 6, 0, 6, false));
  CHECK(!valid_low_state(state, {1, 0, 0, 0}, 0, 0, 6, true));
  CHECK(!valid_low_state(state, {0, 0, 0, 0}, 6, 0, 6, true));
  state.q[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(!valid_low_state(state, {1, 0, 0, 0}, 6, 0, 6, true));
  int publishers = 0, writes = 0;
  HardwareGate g;
  CHECK(!g.authorized());
  auto manual_service = [&] { if (!g.can_control()) return false; ++writes; return true; };
  CHECK(!manual_service()); CHECK(writes == 0);
  CHECK(!g.authorize(true, true, false, true, [&] { ++publishers; }));
  CHECK(publishers == 0);
  CHECK(g.authorize(true, true, true, true, [&] { ++publishers; }));
  CHECK(publishers == 1);
  CHECK(manual_service()); CHECK(writes == 1);
  g.revoke([&] { --publishers; }); CHECK(!g.authorized());
  CHECK(publishers == 0);
  CHECK(!g.can_control()); CHECK(!manual_service()); CHECK(writes == 1);
  FaultLatch fault;
  CHECK(!fault.pending()); fault.mark(); CHECK(fault.pending());
  CHECK(fault.pending()); fault.clear_after_inhibit(); CHECK(!fault.pending());
  PolicyBudget budget;
  budget.reset(0); CHECK(!budget.begin());
  budget.reset(2); CHECK(budget.begin());
  CHECK(!budget.step()); CHECK(budget.step());
  CHECK(!budget.begin());
  TickProgress p;
  CHECK(p.update(0xffffffffu, 0) == TickResult::First);
  CHECK(p.update(0, .002) == TickResult::Advance);
  CHECK(p.fresh(.05));
  CHECK(p.update(0, .06) == TickResult::Duplicate);
  CHECK(!p.fresh(.103));
  CHECK(p.update(0xffffffffu, .104) == TickResult::Backward);
}
