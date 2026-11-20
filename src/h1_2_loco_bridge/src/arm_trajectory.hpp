#ifndef H1_2_LOCO_BRIDGE_ARM_TRAJECTORY_HPP_
#define H1_2_LOCO_BRIDGE_ARM_TRAJECTORY_HPP_

#include <array>
#include <cmath>
#include <algorithm>
#include <vector>

namespace h1_2_arm_trajectory {

// 关节索引常量
constexpr int kWaistIndex = 12;
constexpr int kLeftArmStart = 13;
constexpr int kRightArmStart = 20;
constexpr int kArmJoints = 7;
constexpr int kWeightIndex = 27;

// 五次多项式插值 s(t) = 10t^3 - 15t^4 + 6t^5, t ∈ [0,1]
// 两端速度和加速度为0，中点速度最大为平均速度的1.875倍
inline double quintic_blend(double t) {
  if (t <= 0.0) return 0.0;
  if (t >= 1.0) return 1.0;
  return 10.0 * t * t * t - 15.0 * t * t * t * t + 6.0 * t * t * t * t * t;
}

// 五次多项式的导数 ds/dt = 30t^2 - 60t^3 + 30t^4
inline double quintic_blend_derivative(double t) {
  if (t <= 0.0 || t >= 1.0) return 0.0;
  return 30.0 * t * t - 60.0 * t * t * t + 30.0 * t * t * t * t;
}

// 限位裁剪
inline double clamp_joint(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

// 关节限位（rad）
struct JointLimits {
  std::array<double, kArmJoints> lower;
  std::array<double, kArmJoints> upper;

  static JointLimits right_arm_defaults() {
    JointLimits limits;
    // 肩pitch, 肩roll, 肩yaw, 肘, 腕roll, 腕pitch, 腕yaw
    limits.lower = {-3.14, -3.4, -3.01, -0.95, -2.75, -0.4625, -1.27};
    limits.upper = {1.57, 0.38, 2.66, 3.18, 3.01, 0.4625, 1.27};
    return limits;
  }

  void clamp(std::array<double, kArmJoints>& joints) const {
    for (size_t i = 0; i < kArmJoints; ++i) {
      joints[i] = clamp_joint(joints[i], lower[i], upper[i]);
    }
  }
};

// 单个关节的增益
struct JointGains {
  double kp;
  double kd;
  double tau_ff;  // 前馈力矩

  JointGains() : kp(0.0), kd(0.0), tau_ff(0.0) {}
  JointGains(double p, double d, double ff = 0.0) : kp(p), kd(d), tau_ff(ff) {}
};

// 手臂姿势（7个关节角度）
using ArmPose = std::array<double, kArmJoints>;

// 手型枚举
enum class HandShape {
  Open,
  Fist
};

// 动作段描述
struct MotionSegment {
  ArmPose target_pose;           // 目标姿势
  double nominal_duration;       // 名义用时（秒）
  HandShape hand_shape;          // 手型

  MotionSegment() : target_pose{}, nominal_duration(1.0), hand_shape(HandShape::Open) {}
};

// 轨迹生成参数
struct TrajectoryConfig {
  double control_period;         // 控制周期（秒），默认0.02
  double takeover_duration;      // 接管时长（秒）
  double handback_duration;      // 交还时长（秒）
  double abort_handback_duration; // 中止时交还时长（秒）
  double max_joint_velocity;     // 关节最大速度（rad/s）

  JointLimits limits;

  std::array<JointGains, kArmJoints> right_arm_gains;
  std::array<JointGains, kArmJoints> left_arm_gains;
  JointGains waist_gains;

  TrajectoryConfig()
    : control_period(0.02),
      takeover_duration(1.0),
      handback_duration(2.0),
      abort_handback_duration(1.0),
      max_joint_velocity(0.5),
      limits(JointLimits::right_arm_defaults()) {
    // 默认增益：参考官方例程
    for (int i = 0; i < 3; ++i) {
      right_arm_gains[i] = JointGains(120.0, 2.0);
      left_arm_gains[i] = JointGains(120.0, 2.0);
    }
    right_arm_gains[3] = JointGains(80.0, 1.5);
    left_arm_gains[3] = JointGains(80.0, 1.5);
    for (int i = 4; i < kArmJoints; ++i) {
      right_arm_gains[i] = JointGains(50.0, 1.0);
      left_arm_gains[i] = JointGains(50.0, 1.0);
    }
    waist_gains = JointGains(200.0, 2.0);
  }
};

// 单帧指令
struct FrameCommand {
  ArmPose right_arm;
  ArmPose left_arm;
  double waist;
  double weight;
  HandShape hand_shape;

  FrameCommand() : right_arm{}, left_arm{}, waist(0.0), weight(0.0), hand_shape(HandShape::Open) {}
};

// 轨迹生成器
class TrajectoryGenerator {
public:
  explicit TrajectoryGenerator(const TrajectoryConfig& config) : config_(config) {}

  // 计算段落的实际用时（考虑限速）
  double compute_actual_duration(const ArmPose& start, const ArmPose& end, double nominal_duration) const {
    double max_delta = 0.0;
    for (size_t i = 0; i < kArmJoints; ++i) {
      max_delta = std::max(max_delta, std::abs(end[i] - start[i]));
    }
    // 五次多项式中点速度最大，为平均速度的1.875倍
    const double min_duration = 1.875 * max_delta / config_.max_joint_velocity;
    return std::max(nominal_duration, min_duration);
  }

  // 生成单个段落的轨迹（不包括权重渐变）
  std::vector<FrameCommand> generate_segment(
      const ArmPose& start_right,
      const ArmPose& start_left,
      double start_waist,
      const MotionSegment& segment) const {

    // 裁剪目标到限位范围
    ArmPose clamped_target = segment.target_pose;
    config_.limits.clamp(clamped_target);

    const double actual_duration = compute_actual_duration(start_right, clamped_target, segment.nominal_duration);
    const int num_frames = static_cast<int>(std::ceil(actual_duration / config_.control_period));

    std::vector<FrameCommand> frames;
    frames.reserve(num_frames);

    for (int i = 0; i < num_frames; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(num_frames - 1);
      const double s = quintic_blend(t);

      FrameCommand cmd;
      for (size_t j = 0; j < kArmJoints; ++j) {
        cmd.right_arm[j] = start_right[j] + s * (clamped_target[j] - start_right[j]);
      }
      cmd.left_arm = start_left;  // 左臂保持不动
      cmd.waist = start_waist;    // 腰保持不动
      cmd.weight = 1.0;            // 段落执行时权重保持1
      cmd.hand_shape = segment.hand_shape;

      frames.push_back(cmd);
    }

    return frames;
  }

  // 生成权重渐变段
  std::vector<FrameCommand> generate_weight_ramp(
      const ArmPose& right_arm,
      const ArmPose& left_arm,
      double waist,
      HandShape hand_shape,
      double duration,
      bool ramp_up) const {

    const int num_frames = static_cast<int>(std::ceil(duration / config_.control_period));
    std::vector<FrameCommand> frames;
    frames.reserve(num_frames);

    for (int i = 0; i < num_frames; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(num_frames - 1);

      FrameCommand cmd;
      cmd.right_arm = right_arm;
      cmd.left_arm = left_arm;
      cmd.waist = waist;
      cmd.weight = ramp_up ? t : (1.0 - t);
      cmd.hand_shape = hand_shape;

      frames.push_back(cmd);
    }

    return frames;
  }

  // 计算总时长（用于超时检查）
  double compute_total_duration(
      const ArmPose& initial_right,
      const std::vector<MotionSegment>& segments) const {

    double total = config_.takeover_duration + config_.handback_duration;

    ArmPose current = initial_right;
    for (const auto& seg : segments) {
      ArmPose clamped = seg.target_pose;
      config_.limits.clamp(clamped);
      total += compute_actual_duration(current, clamped, seg.nominal_duration);
      current = clamped;
    }

    return total;
  }

  const TrajectoryConfig& config() const { return config_; }

private:
  TrajectoryConfig config_;
};

}  // namespace h1_2_arm_trajectory

#endif  // H1_2_LOCO_BRIDGE_ARM_TRAJECTORY_HPP_
