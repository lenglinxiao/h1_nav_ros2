#include <gtest/gtest.h>
#include <cmath>
#include "arm_trajectory.hpp"

using namespace h1_2_arm_trajectory;

// 测试五次多项式插值
TEST(ArmTrajectoryTest, QuinticBlend) {
  EXPECT_DOUBLE_EQ(quintic_blend(0.0), 0.0);
  EXPECT_DOUBLE_EQ(quintic_blend(1.0), 1.0);
  EXPECT_GT(quintic_blend(0.5), 0.4);
  EXPECT_LT(quintic_blend(0.5), 0.6);

  // 两端导数为0
  EXPECT_NEAR(quintic_blend_derivative(0.0), 0.0, 1e-6);
  EXPECT_NEAR(quintic_blend_derivative(1.0), 0.0, 1e-6);
}

// 测试限位裁剪
TEST(ArmTrajectoryTest, JointLimits) {
  JointLimits limits = JointLimits::right_arm_defaults();

  ArmPose pose = {-4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  limits.clamp(pose);
  EXPECT_GE(pose[0], limits.lower[0]);

  pose = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  limits.clamp(pose);
  EXPECT_LE(pose[1], limits.upper[1]);
}

// 测试轨迹生成：速度限制
TEST(ArmTrajectoryTest, VelocityLimit) {
  TrajectoryConfig config;
  config.control_period = 0.02;
  config.max_joint_velocity = 0.5;

  TrajectoryGenerator gen(config);

  ArmPose start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ArmPose end = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  MotionSegment seg;
  seg.target_pose = end;
  seg.nominal_duration = 0.5;
  seg.hand_shape = HandShape::Open;

  auto frames = gen.generate_segment(start, start, 0.0, seg);

  // 检查每帧速度不超过限速
  for (size_t i = 1; i < frames.size(); ++i) {
    for (size_t j = 0; j < kArmJoints; ++j) {
      double delta = std::abs(frames[i].right_arm[j] - frames[i-1].right_arm[j]);
      double velocity = delta / config.control_period;
      EXPECT_LE(velocity, config.max_joint_velocity * 1.875 + 0.01);  // 1.875是峰值因子，+0.01容差
    }
  }
}

// 测试轨迹生成：到达目标
TEST(ArmTrajectoryTest, ReachTarget) {
  TrajectoryConfig config;
  TrajectoryGenerator gen(config);

  ArmPose start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ArmPose end = {0.5, -0.3, 0.2, 1.0, 0.0, 0.0, 0.0};

  // 裁剪目标
  config.limits.clamp(end);

  MotionSegment seg;
  seg.target_pose = {0.5, -0.3, 0.2, 1.0, 0.0, 0.0, 0.0};
  seg.nominal_duration = 1.0;
  seg.hand_shape = HandShape::Open;

  auto frames = gen.generate_segment(start, start, 0.0, seg);

  ASSERT_GT(frames.size(), 0);

  // 最后一帧应该精确到达目标
  for (size_t j = 0; j < kArmJoints; ++j) {
    EXPECT_NEAR(frames.back().right_arm[j], end[j], 1e-6);
  }
}

// 测试权重渐变
TEST(ArmTrajectoryTest, WeightRamp) {
  TrajectoryConfig config;
  config.control_period = 0.02;
  TrajectoryGenerator gen(config);

  ArmPose pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // 上升
  auto ramp_up = gen.generate_weight_ramp(pose, pose, 0.0, HandShape::Open, 1.0, true);
  ASSERT_GT(ramp_up.size(), 0);
  EXPECT_NEAR(ramp_up.front().weight, 0.0, 0.1);
  EXPECT_NEAR(ramp_up.back().weight, 1.0, 0.1);

  // 下降
  auto ramp_down = gen.generate_weight_ramp(pose, pose, 0.0, HandShape::Open, 1.0, false);
  ASSERT_GT(ramp_down.size(), 0);
  EXPECT_NEAR(ramp_down.front().weight, 1.0, 0.1);
  EXPECT_NEAR(ramp_down.back().weight, 0.0, 0.1);
}

// 测试相同起点终点
TEST(ArmTrajectoryTest, SameStartEnd) {
  TrajectoryConfig config;
  TrajectoryGenerator gen(config);

  ArmPose pose = {0.5, -0.2, 0.0, 1.0, 0.0, 0.0, 0.0};

  MotionSegment seg;
  seg.target_pose = pose;
  seg.nominal_duration = 1.0;
  seg.hand_shape = HandShape::Open;

  auto frames = gen.generate_segment(pose, pose, 0.0, seg);

  ASSERT_GT(frames.size(), 0);

  // 所有帧应该保持不变
  for (const auto& frame : frames) {
    for (size_t j = 0; j < kArmJoints; ++j) {
      EXPECT_NEAR(frame.right_arm[j], pose[j], 1e-6);
    }
  }
}

// 测试总时长计算
TEST(ArmTrajectoryTest, TotalDuration) {
  TrajectoryConfig config;
  config.takeover_duration = 1.0;
  config.handback_duration = 2.0;
  TrajectoryGenerator gen(config);

  ArmPose start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::vector<MotionSegment> segments;
  MotionSegment seg1;
  seg1.target_pose = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  seg1.nominal_duration = 1.0;
  segments.push_back(seg1);

  MotionSegment seg2;
  seg2.target_pose = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  seg2.nominal_duration = 1.0;
  segments.push_back(seg2);

  double total = gen.compute_total_duration(start, segments);

  // 至少应该包含接管、两段动作和交还
  EXPECT_GE(total, 1.0 + 1.0 + 1.0 + 2.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
