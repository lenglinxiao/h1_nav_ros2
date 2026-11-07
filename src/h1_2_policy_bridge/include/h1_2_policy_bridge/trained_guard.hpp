#pragma once
#include "h1_2_policy_bridge/core.hpp"
#include <cstdint>
#include <optional>

namespace h1_policy {
// All joint arrays passed to this guard are in SDK motor order.
const std::array<const char*, 27>& sdk_joint_names();
struct TrainedGuardConfig {
  Joints defaults{}, joint_speed_limits{}, pos_lower{}, pos_upper{};
  unsigned consecutive = 3;
  double warmup_seconds = 2.0, warmup_multiplier = 2.0;
  double base_gyro_limit = 2.5, tilt_angle = .2, tilt_seconds = .5;
  double prearm_max_pos_error = .10;
};
enum class TrainedGuardReason { JointSpeed, JointPosition, BaseAngularSpeed, Tilt };
struct TrainedGuardEvent {
  TrainedGuardReason reason;
  int sdk_index; // -1 for base events
  double value, limit;
  uint64_t tick;
  double elapsed;
  bool warmup;
  unsigned consecutive;
};
struct TrainedGuardPrearm {
  bool ready;
  double max_pos_error;
  int sdk_index;
};
class TrainedGuard {
public:
  explicit TrainedGuard(const TrainedGuardConfig& config);
  TrainedGuardPrearm prearm(const State& sdk_state) const;
  void arm(double now);
  void reset();
  std::optional<TrainedGuardEvent> observe(const State& sdk_state, uint64_t tick, double now);
  bool tripped() const { return last_event_.has_value(); }
  const std::optional<TrainedGuardEvent>& last_event() const { return last_event_; }
private:
  TrainedGuardConfig config_;
  std::array<unsigned, 27> speed_counts_{}, position_counts_{};
  unsigned gyro_count_ = 0;
  bool armed_ = false, has_tick_ = false, tilt_active_ = false;
  uint64_t last_tick_ = 0;
  double started_ = 0, last_time_ = 0, tilt_since_ = 0;
  std::optional<TrainedGuardEvent> last_event_;
};
}
