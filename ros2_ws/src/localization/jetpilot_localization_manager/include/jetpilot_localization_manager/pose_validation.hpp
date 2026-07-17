#ifndef JETPILOT_LOCALIZATION_MANAGER__POSE_VALIDATION_HPP_
#define JETPILOT_LOCALIZATION_MANAGER__POSE_VALIDATION_HPP_

#include <cstdint>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace jetpilot_localization_manager
{

struct PoseValidationOptions
{
  std::string expected_frame_id{"map"};
  double quaternion_norm_tolerance{0.1};
  double max_pose_age_sec{0.0};
};

struct PoseValidationResult
{
  bool valid{false};
  std::string reason;
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
};

/// Validate an incoming localization pose and normalize its quaternion.
///
/// max_pose_age_sec <= 0 disables age validation. A zero timestamp is accepted
/// because it is commonly used for "latest" poses and during simulated time
/// startup.
PoseValidationResult validate_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & input,
  const PoseValidationOptions & options,
  std::int64_t now_nanoseconds);

}  // namespace jetpilot_localization_manager

#endif  // JETPILOT_LOCALIZATION_MANAGER__POSE_VALIDATION_HPP_
