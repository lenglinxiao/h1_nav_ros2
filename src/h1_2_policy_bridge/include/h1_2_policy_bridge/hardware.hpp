#pragma once
#include "h1_2_policy_bridge/core.hpp"
#include <unitree/dds_wrapper/common/crc.h>
#include <cstdint>
#include <cmath>
#include <string>
namespace h1_policy {
// Bound the age of the actual sample used to compute a hardware command,
// rather than only the age of the latest packet received during inference.
inline bool command_snapshot_fresh(double received, double now) {
  return std::isfinite(received) && std::isfinite(now) &&
         now >= received && now - received <= .02;
}
template<class Message> bool valid_crc(Message message, uint32_t expected) {
  static_assert(sizeof(Message) % 4 == 0);
  return crc32_core(reinterpret_cast<uint32_t*>(&message), (sizeof(message) >> 2) - 1) == expected;
}
inline bool valid_profile(const std::string& profile, const std::string& interface, int domain) {
  return (profile == "sim" && interface == "lo" && domain == 42) ||
         (profile == "hardware" && interface == "eth0" && domain == 0);
}
inline bool valid_low_state(const State& s, const std::array<float, 4>& quat,
                            uint8_t machine, uint8_t pr, uint8_t expected_machine, bool crc_ok) {
  if (!crc_ok || machine != expected_machine || pr != 0) return false;
  for (auto x : s.q) if (!std::isfinite(x)) return false;
  for (auto x : s.dq) if (!std::isfinite(x)) return false;
  for (auto x : s.gyro) if (!std::isfinite(x)) return false;
  float norm2 = 0;
  for (auto x : quat) { if (!std::isfinite(x)) return false; norm2 += x*x; }
  return std::isfinite(norm2) && std::abs(std::sqrt(norm2) - 1) <= .05f;
}
class HardwareGate {
 public:
  template<class Factory> bool authorize(bool enabled, bool debug_confirmed, bool exclusive_confirmed, bool state_valid, Factory factory) {
    if (authorized_ || !enabled || !debug_confirmed || !exclusive_confirmed || !state_valid) return false;
    factory();
    authorized_ = true;
    return true;
  }
  template<class Destroy> void revoke(Destroy destroy) { authorized_ = false; destroy(); }
  bool can_control() const { return authorized_; }
  bool authorized() const { return authorized_; }
 private:
  bool authorized_ = false;
};
class FaultLatch {
 public:
  void mark() { pending_ = true; }
  bool pending() const { return pending_; }
  void clear_after_inhibit() { pending_ = false; }
 private: bool pending_ = false;
};
class PolicyBudget {
 public:
  void reset(int max_steps) { remaining_ = max_steps; used_ = false; }
  bool can_begin() const { return !used_ && remaining_ > 0; }
  bool begin() { if (!can_begin()) return false; used_ = true; return true; }
  bool step() { if (remaining_ > 0) --remaining_; return remaining_ == 0; }
 private: int remaining_ = 0; bool used_ = false;
};
enum class TickResult { First, Advance, Duplicate, Backward };
class TickProgress {
 public:
  TickResult update(uint32_t tick, double now) {
    if (!seen_) { tick_ = tick; seen_ = true; return TickResult::First; }
    const uint32_t delta = tick - tick_;
    if (delta == 0) return TickResult::Duplicate;
    if (delta >= 0x80000000u) return TickResult::Backward;
    tick_ = tick; last_progress_ = now; progressed_ = true; return TickResult::Advance;
  }
  bool fresh(double now) const { return progressed_ && now >= last_progress_ && now - last_progress_ <= .1; }
 private:
  uint32_t tick_ = 0;
  double last_progress_ = 0;
  bool seen_ = false, progressed_ = false;
};
}
