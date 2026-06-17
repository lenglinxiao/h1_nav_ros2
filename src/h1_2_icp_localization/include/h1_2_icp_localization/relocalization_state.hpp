#ifndef H1_2_ICP_LOCALIZATION_RELOCALIZATION_STATE_HPP_
#define H1_2_ICP_LOCALIZATION_RELOCALIZATION_STATE_HPP_

#include <algorithm>

#include <Eigen/Core>

namespace h1_2_icp_localization
{

enum class RelocalizationState
{
  GLOBAL_INIT,
  TRACKING,
  GLOBAL_RECOVERY
};

class RelocalizationStateMachine
{
public:
  explicit RelocalizationStateMachine(int max_consecutive_failures = 3)
  : max_consecutive_failures_(std::max(1, max_consecutive_failures))
  {
  }

  RelocalizationState state() const { return state_; }

  int consecutiveFailures() const { return consecutive_failures_; }

  void onGlobalInitializationSucceeded()
  {
    state_ = RelocalizationState::TRACKING;
    consecutive_failures_ = 0;
  }

  void onTrackingSucceeded()
  {
    state_ = RelocalizationState::TRACKING;
    consecutive_failures_ = 0;
  }

  void onTrackingFailed()
  {
    ++consecutive_failures_;
    if (consecutive_failures_ >= max_consecutive_failures_) {
      state_ = RelocalizationState::GLOBAL_RECOVERY;
    }
  }

  void onGlobalRecoverySucceeded()
  {
    state_ = RelocalizationState::TRACKING;
    consecutive_failures_ = 0;
  }

  void forceGlobalRecovery()
  {
    state_ = RelocalizationState::GLOBAL_RECOVERY;
  }

  void resetToGlobalInitialization()
  {
    state_ = RelocalizationState::GLOBAL_INIT;
    consecutive_failures_ = 0;
  }

private:
  RelocalizationState state_ = RelocalizationState::GLOBAL_INIT;
  int max_consecutive_failures_;
  int consecutive_failures_ = 0;
};

inline Eigen::Matrix4d computeMapToOdom(
  const Eigen::Matrix4d & map_to_base,
  const Eigen::Matrix4d & odom_to_base)
{
  return map_to_base * odom_to_base.inverse();
}

}  // namespace h1_2_icp_localization

#endif  // H1_2_ICP_LOCALIZATION_RELOCALIZATION_STATE_HPP_
