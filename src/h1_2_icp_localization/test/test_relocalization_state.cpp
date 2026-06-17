#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "h1_2_icp_localization/cloud_accumulator.hpp"
#include "h1_2_icp_localization/relocalization_state.hpp"

namespace
{

Eigen::Matrix4d makeTransform(double x, double y, double z, double yaw)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform(0, 3) = x;
  transform(1, 3) = y;
  transform(2, 3) = z;
  return transform;
}

}  // namespace

TEST(RelocalizationStateMachine, StartsWithGlobalInitialization)
{
  h1_2_icp_localization::RelocalizationStateMachine machine;

  EXPECT_EQ(machine.state(), h1_2_icp_localization::RelocalizationState::GLOBAL_INIT);
  EXPECT_EQ(machine.consecutiveFailures(), 0);
}

TEST(RelocalizationStateMachine, SuccessfulGlobalInitializationEntersTracking)
{
  h1_2_icp_localization::RelocalizationStateMachine machine;

  machine.onGlobalInitializationSucceeded();

  EXPECT_EQ(machine.state(), h1_2_icp_localization::RelocalizationState::TRACKING);
  EXPECT_EQ(machine.consecutiveFailures(), 0);
}

TEST(RelocalizationStateMachine, ConsecutiveTrackingFailuresEnterRecovery)
{
  h1_2_icp_localization::RelocalizationStateMachine machine(2);
  machine.onGlobalInitializationSucceeded();

  machine.onTrackingFailed();
  EXPECT_EQ(machine.state(), h1_2_icp_localization::RelocalizationState::TRACKING);
  EXPECT_EQ(machine.consecutiveFailures(), 1);

  machine.onTrackingFailed();
  EXPECT_EQ(machine.state(), h1_2_icp_localization::RelocalizationState::GLOBAL_RECOVERY);
  EXPECT_EQ(machine.consecutiveFailures(), 2);
}

TEST(RelocalizationStateMachine, RecoverySuccessReturnsToTracking)
{
  h1_2_icp_localization::RelocalizationStateMachine machine(1);
  machine.onGlobalInitializationSucceeded();
  machine.onTrackingFailed();

  machine.onGlobalRecoverySucceeded();

  EXPECT_EQ(machine.state(), h1_2_icp_localization::RelocalizationState::TRACKING);
  EXPECT_EQ(machine.consecutiveFailures(), 0);
}

TEST(PoseMath, ComputesMapToOdomFromMapToBaseAndOdomToBase)
{
  const Eigen::Matrix4d map_to_base = makeTransform(2.0, 3.0, 0.4, 0.7);
  const Eigen::Matrix4d odom_to_base = makeTransform(0.5, -1.0, 0.2, -0.3);

  const Eigen::Matrix4d map_to_odom =
    h1_2_icp_localization::computeMapToOdom(map_to_base, odom_to_base);

  const Eigen::Matrix4d reconstructed_map_to_base = map_to_odom * odom_to_base;
  EXPECT_TRUE(reconstructed_map_to_base.isApprox(map_to_base, 1e-9));
}

TEST(CloudAccumulator, KeepsNewestPointsWhenAccumulationExceedsLimit)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int index = 0; index < 6; ++index) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(index);
    cloud.push_back(point);
  }

  h1_2_icp_localization::keepNewestPoints(cloud, 3);

  ASSERT_EQ(cloud.size(), 3u);
  EXPECT_FLOAT_EQ(cloud[0].x, 3.0f);
  EXPECT_FLOAT_EQ(cloud[1].x, 4.0f);
  EXPECT_FLOAT_EQ(cloud[2].x, 5.0f);
  EXPECT_EQ(cloud.width, 3u);
  EXPECT_EQ(cloud.height, 1u);
}
