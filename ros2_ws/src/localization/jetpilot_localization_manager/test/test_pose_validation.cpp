#include <limits>

#include "gtest/gtest.h"
#include "jetpilot_localization_manager/pose_validation.hpp"

namespace
{

geometry_msgs::msg::PoseWithCovarianceStamped valid_pose()
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp.sec = 10;
  pose.pose.pose.orientation.w = 1.0;
  return pose;
}

TEST(PoseValidation, AcceptsValidPose)
{
  jetpilot_localization_manager::PoseValidationOptions options;
  const auto result = jetpilot_localization_manager::validate_pose(valid_pose(), options, 11000000000LL);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.reason, "ok");
}

TEST(PoseValidation, NormalizesQuaternionWithinTolerance)
{
  auto pose = valid_pose();
  pose.pose.pose.orientation.w = 1.05;
  jetpilot_localization_manager::PoseValidationOptions options;
  const auto result = jetpilot_localization_manager::validate_pose(pose, options, 11000000000LL);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.pose.pose.pose.orientation.w, 1.0);
}

TEST(PoseValidation, RejectsQuaternionOutsideTolerance)
{
  auto pose = valid_pose();
  pose.pose.pose.orientation.w = 1.2;
  jetpilot_localization_manager::PoseValidationOptions options;
  const auto result = jetpilot_localization_manager::validate_pose(pose, options, 11000000000LL);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "quaternion_norm_out_of_range");
}

TEST(PoseValidation, RejectsWrongFrame)
{
  auto pose = valid_pose();
  pose.header.frame_id = "odom";
  jetpilot_localization_manager::PoseValidationOptions options;
  const auto result = jetpilot_localization_manager::validate_pose(pose, options, 11000000000LL);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "unexpected_frame");
}

TEST(PoseValidation, RejectsNonFiniteValues)
{
  auto pose = valid_pose();
  pose.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  jetpilot_localization_manager::PoseValidationOptions options;
  const auto result = jetpilot_localization_manager::validate_pose(pose, options, 11000000000LL);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "non_finite_position");
}

TEST(PoseValidation, RejectsNegativeCovarianceDiagonal)
{
  auto pose = valid_pose();
  pose.pose.covariance[7] = -0.1;
  jetpilot_localization_manager::PoseValidationOptions options;
  const auto result = jetpilot_localization_manager::validate_pose(pose, options, 11000000000LL);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "negative_covariance_diagonal");
}

TEST(PoseValidation, RejectsStalePoseWhenEnabled)
{
  jetpilot_localization_manager::PoseValidationOptions options;
  options.max_pose_age_sec = 0.5;
  const auto result = jetpilot_localization_manager::validate_pose(valid_pose(), options, 11000000000LL);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "stale_pose");
}

TEST(PoseValidation, AllowsZeroTimestamp)
{
  auto pose = valid_pose();
  pose.header.stamp.sec = 0;
  jetpilot_localization_manager::PoseValidationOptions options;
  options.max_pose_age_sec = 0.5;
  const auto result = jetpilot_localization_manager::validate_pose(pose, options, 11000000000LL);
  EXPECT_TRUE(result.valid);
}

}  // namespace
