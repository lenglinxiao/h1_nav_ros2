#include "h1_2_policy_bridge/trained_guard.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace h1_policy {
const std::array<const char*, 27>& sdk_joint_names() {
  static const std::array<const char*, 27> names = {
    "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_yaw_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "torso_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
  return names;
}
TrainedGuard::TrainedGuard(const TrainedGuardConfig& c) : config_(c) {
  auto positive = [](double v) { return std::isfinite(v) && v > 0; };
  if (!c.consecutive || !std::isfinite(c.warmup_seconds) || c.warmup_seconds < 0 ||
      !positive(c.warmup_multiplier) || c.warmup_multiplier < 1 || !positive(c.base_gyro_limit) ||
      !positive(c.tilt_angle) || !positive(c.tilt_seconds) || !positive(c.prearm_max_pos_error))
    throw std::invalid_argument("invalid trained guard scalar configuration");
  for (size_t i = 0; i < 27; ++i) {
    if (!std::isfinite(c.defaults[i]) || !positive(c.joint_speed_limits[i]) ||
        !std::isfinite(c.pos_lower[i]) || !std::isfinite(c.pos_upper[i]) ||
        c.pos_lower[i] > c.pos_upper[i] || c.defaults[i] < c.pos_lower[i] || c.defaults[i] > c.pos_upper[i])
      throw std::invalid_argument("invalid trained guard joint configuration");
  }
}
TrainedGuardPrearm TrainedGuard::prearm(const State& s) const {
  double maximum = 0; int index = 0;
  for (int i = 0; i < 27; ++i) {
    if (!std::isfinite(s.q[i])) throw std::invalid_argument("nonfinite joint position");
    double error = std::abs(double(s.q[i]) - config_.defaults[i]);
    if (error > maximum) { maximum = error; index = i; }
  }
  return {maximum < config_.prearm_max_pos_error, maximum, index};
}
void TrainedGuard::reset() {
  speed_counts_.fill(0); position_counts_.fill(0); gyro_count_ = 0;
  armed_ = has_tick_ = tilt_active_ = false; last_event_.reset();
}
void TrainedGuard::arm(double now) {
  if (!std::isfinite(now)) throw std::invalid_argument("invalid arm time");
  reset(); armed_ = true; started_ = last_time_ = now;
}
std::optional<TrainedGuardEvent> TrainedGuard::observe(const State& s, uint64_t tick, double now) {
  if (!armed_) throw std::logic_error("trained guard is not armed");
  if (!std::isfinite(now) || now < last_time_) throw std::invalid_argument("trained guard time regressed");
  last_time_ = now;
  if (last_event_) return last_event_;
  if (has_tick_ && tick == last_tick_) return std::nullopt;
  for (size_t i = 0; i < 27; ++i)
    if (!std::isfinite(s.q[i]) || !std::isfinite(s.dq[i])) throw std::invalid_argument("nonfinite joint state");
  for (size_t i = 0; i < 3; ++i)
    if (!std::isfinite(s.gyro[i]) || !std::isfinite(s.gravity[i])) throw std::invalid_argument("nonfinite base state");
  has_tick_ = true; last_tick_ = tick;
  const double elapsed = now - started_;
  const bool warm = elapsed < config_.warmup_seconds;
  const double multiplier = warm ? config_.warmup_multiplier : 1.;
  auto event = [&](TrainedGuardReason reason, int index, double value, double limit, unsigned count) {
    last_event_ = TrainedGuardEvent{reason, index, value, limit, tick, elapsed, warm, count};
    return last_event_;
  };
  for (int i = 0; i < 27; ++i) {
    const double speed = std::abs(double(s.dq[i]));
    const double limit = config_.joint_speed_limits[i] * multiplier;
    speed_counts_[i] = speed > limit ? speed_counts_[i] + 1 : 0;
    if (speed_counts_[i] >= config_.consecutive) return event(TrainedGuardReason::JointSpeed, i, speed, limit, speed_counts_[i]);
    const double q = s.q[i];
    const bool bad = q < config_.pos_lower[i] || q > config_.pos_upper[i];
    position_counts_[i] = bad ? position_counts_[i] + 1 : 0;
    if (position_counts_[i] >= config_.consecutive) return event(TrainedGuardReason::JointPosition, i, q, q < config_.pos_lower[i] ? config_.pos_lower[i] : config_.pos_upper[i], position_counts_[i]);
  }
  const double gyro = std::hypot(std::hypot(double(s.gyro[0]), double(s.gyro[1])), double(s.gyro[2]));
  const double gyro_limit = config_.base_gyro_limit * multiplier;
  gyro_count_ = gyro > gyro_limit ? gyro_count_ + 1 : 0;
  if (gyro_count_ >= config_.consecutive) return event(TrainedGuardReason::BaseAngularSpeed, -1, gyro, gyro_limit, gyro_count_);
  const double roll = std::atan2(-double(s.gravity[1]), -double(s.gravity[2]));
  const double pitch = std::atan2(double(s.gravity[0]), std::hypot(double(s.gravity[1]), double(s.gravity[2])));
  const double angle = std::abs(roll) >= std::abs(pitch) ? roll : pitch;
  if (std::abs(angle) > config_.tilt_angle) {
    if (!tilt_active_) { tilt_active_ = true; tilt_since_ = now; }
    if (now - tilt_since_ >= config_.tilt_seconds) return event(TrainedGuardReason::Tilt, -1, angle, config_.tilt_angle, 0);
  } else tilt_active_ = false;
  return std::nullopt;
}
}
