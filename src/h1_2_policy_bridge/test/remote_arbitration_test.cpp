#include "h1_2_policy_bridge/remote_arbitration.hpp"
#include "check.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>

using namespace h1_policy;

static std::array<uint8_t, 40> remote(float lx, float rx, float ly) {
  std::array<uint8_t, 40> bytes{};
  std::memcpy(bytes.data() + 4, &lx, sizeof(float));
  std::memcpy(bytes.data() + 8, &rx, sizeof(float));
  std::memcpy(bytes.data() + 20, &ly, sizeof(float));
  return bytes;
}

int main() {
  RemoteAxes axes;
  CHECK(decode_remote_axes(remote(0, .5f, .8f), axes));
  CHECK(axes.valid);
  CHECK(std::abs(axes.rx - .5f) < 1e-6f);
  CHECK(std::abs(axes.ly - .8f) < 1e-6f);

  const Command nav{.08f, 0.f, .02f};
  auto selected = arbitrate_velocity(nav, true, axes, true, true, .2f, .1f);
  CHECK(selected.source == VelocitySource::Remote);
  CHECK(std::abs(selected.command[0] - .16f) < 1e-6f);
  CHECK(std::abs(selected.command[2] + .05f) < 1e-6f);

  RemoteAxes neutral;
  CHECK(decode_remote_axes(remote(0, 0, 0), neutral));
  selected = arbitrate_velocity(nav, true, neutral, true, true, .2f, .1f);
  CHECK(selected.source == VelocitySource::Navigation);
  CHECK(std::abs(selected.command[0] - nav[0]) < 1e-6f);

  selected = arbitrate_velocity(nav, false, axes, false, true, .2f, .1f);
  CHECK(selected.source == VelocitySource::Zero);
  selected = arbitrate_velocity(nav, true, axes, false, true, .2f, .1f);
  CHECK(selected.source == VelocitySource::Navigation);
  selected = arbitrate_velocity(nav, true, axes, true, false, .2f, .1f);
  CHECK(selected.source == VelocitySource::Navigation);

  auto invalid_bytes = remote(0, 0, 0);
  float nan = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(invalid_bytes.data() + 20, &nan, sizeof(float));
  CHECK(!decode_remote_axes(invalid_bytes, axes));
  selected = arbitrate_velocity(nav, true, axes, true, true, .2f, .1f);
  CHECK(selected.source == VelocitySource::Navigation);
  selected = arbitrate_velocity(nav, false, axes, true, true, .2f, .1f);
  CHECK(selected.source == VelocitySource::Zero);

  return 0;
}
