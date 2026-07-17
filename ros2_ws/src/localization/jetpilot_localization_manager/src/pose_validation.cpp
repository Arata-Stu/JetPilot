#include "jetpilot_localization_manager/pose_validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "builtin_interfaces/msg/time.hpp"

namespace jetpilot_localization_manager
{
namespace
{

bool finite(double value)
{
  return std::isfinite(value);
}

std::int64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(stamp.nanosec);
}

}  // namespace

PoseValidationResult validate_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & input,
  const PoseValidationOptions & options,
  std::int64_t now_nanoseconds)
{
  PoseValidationResult result;
  result.pose = input;

  if (!options.expected_frame_id.empty() && input.header.frame_id != options.expected_frame_id) {
    result.reason = "unexpected_frame";
    return result;
  }

  const auto & position = input.pose.pose.position;
  if (!finite(position.x) || !finite(position.y) || !finite(position.z)) {
    result.reason = "non_finite_position";
    return result;
  }

  const auto & orientation = input.pose.pose.orientation;
  if (!finite(orientation.x) || !finite(orientation.y) ||
    !finite(orientation.z) || !finite(orientation.w))
  {
    result.reason = "non_finite_orientation";
    return result;
  }

  const double norm_squared = orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (!finite(norm_squared) || norm_squared <= std::numeric_limits<double>::epsilon()) {
    result.reason = "invalid_quaternion";
    return result;
  }

  const double norm = std::sqrt(norm_squared);
  const double tolerance = std::max(0.0, options.quaternion_norm_tolerance);
  if (std::abs(norm - 1.0) > tolerance) {
    result.reason = "quaternion_norm_out_of_range";
    return result;
  }

  for (std::size_t index = 0; index < input.pose.covariance.size(); ++index) {
    if (!finite(input.pose.covariance[index])) {
      result.reason = "non_finite_covariance";
      return result;
    }
  }
  constexpr std::array<std::size_t, 6> diagonal_indices{0, 7, 14, 21, 28, 35};
  for (const auto index : diagonal_indices) {
    if (input.pose.covariance[index] < 0.0) {
      result.reason = "negative_covariance_diagonal";
      return result;
    }
  }

  if (options.max_pose_age_sec > 0.0) {
    const auto stamp_nanoseconds = stamp_to_nanoseconds(input.header.stamp);
    if (stamp_nanoseconds > 0 && now_nanoseconds > 0) {
      const double age_sec = static_cast<double>(now_nanoseconds - stamp_nanoseconds) / 1.0e9;
      if (age_sec > options.max_pose_age_sec) {
        result.reason = "stale_pose";
        return result;
      }
      // A small clock skew is harmless, but a pose far in the future usually
      // indicates a clock-domain configuration error.
      if (age_sec < -options.max_pose_age_sec) {
        result.reason = "future_pose";
        return result;
      }
    }
  }

  auto & normalized = result.pose.pose.pose.orientation;
  normalized.x /= norm;
  normalized.y /= norm;
  normalized.z /= norm;
  normalized.w /= norm;
  result.valid = true;
  result.reason = "ok";
  return result;
}

}  // namespace jetpilot_localization_manager
