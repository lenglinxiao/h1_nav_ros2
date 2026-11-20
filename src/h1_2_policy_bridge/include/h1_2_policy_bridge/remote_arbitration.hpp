#pragma once

#include "h1_2_policy_bridge/core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace h1_policy {

// H1-2 LowState carries the Unitree 40-byte remote packet.  Keep this
// decoder independent of ROS and of the SDK headers so the arbitration can
// be tested without a DDS participant.  The layout is the SDK
// REMOTE_DATA_RX layout: two header bytes, a 16-bit button field, then
// lx/rx/ry/L2/ly float axes.
struct RemoteAxes {
  float lx = 0;
  float rx = 0;
  float ry = 0;
  float l2 = 0;
  float ly = 0;
  uint16_t buttons = 0;
  bool valid = false;
};

inline bool decode_remote_axes(const std::array<uint8_t, 40>& bytes, RemoteAxes& axes) {
  std::memcpy(&axes.buttons, bytes.data() + 2, sizeof(axes.buttons));
  std::memcpy(&axes.lx, bytes.data() + 4, sizeof(float));
  std::memcpy(&axes.rx, bytes.data() + 8, sizeof(float));
  std::memcpy(&axes.ry, bytes.data() + 12, sizeof(float));
  std::memcpy(&axes.l2, bytes.data() + 16, sizeof(float));
  std::memcpy(&axes.ly, bytes.data() + 20, sizeof(float));
  axes.valid = std::isfinite(axes.lx) && std::isfinite(axes.rx) &&
               std::isfinite(axes.ry) && std::isfinite(axes.l2) &&
               std::isfinite(axes.ly);
  if (!axes.valid) axes = {};
  return axes.valid;
}

enum class VelocitySource { Zero, Navigation, Remote };

struct ArbitrationResult {
  Command command{0, 0, 0};
  VelocitySource source = VelocitySource::Zero;
  bool remote_valid = false;
  bool remote_active = false;
};

inline ArbitrationResult arbitrate_velocity(
    const Command& navigation, bool navigation_fresh,
    const RemoteAxes& remote, bool remote_fresh, bool remote_enabled,
    float max_linear_speed, float max_angular_speed, float deadzone = .05f) {
  ArbitrationResult result;
  if (!std::isfinite(max_linear_speed) || !std::isfinite(max_angular_speed) ||
      max_linear_speed <= 0 || max_angular_speed <= 0 ||
      !std::isfinite(deadzone) || deadzone < 0 || deadzone >= 1) {
    return result;
  }
  if (remote_enabled && remote_fresh && remote.valid) {
    const float vx = std::abs(remote.ly) < deadzone ? 0.f :
        std::clamp(remote.ly, -1.f, 1.f) * max_linear_speed;
    // Match Unitree's existing gamepad mapping (positive stick-right means
    // negative command yaw), as used by the SDK state-machine example.
    const float wz = std::abs(remote.rx) < deadzone ? 0.f :
        -std::clamp(remote.rx, -1.f, 1.f) * max_angular_speed;
    result.remote_valid = true;
    result.remote_active = std::abs(vx) > 1e-5f || std::abs(wz) > 1e-5f;
    if (result.remote_active) {
      result.command = {vx, 0.f, wz};
      result.source = VelocitySource::Remote;
      return result;
    }
  }
  if (navigation_fresh) {
    result.command = navigation;
    result.source = VelocitySource::Navigation;
  }
  return result;
}

}  // namespace h1_policy
