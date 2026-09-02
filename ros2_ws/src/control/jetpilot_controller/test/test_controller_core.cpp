#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "jetpilot_controller/longitudinal_controller.hpp"
#include "jetpilot_controller/kinematic_mpc.hpp"
#include "jetpilot_controller/map_pursuit.hpp"
#include "jetpilot_controller/motion_direction.hpp"
#include "jetpilot_controller/pure_pursuit.hpp"
#include "jetpilot_controller/trailing_controller.hpp"
#include "jetpilot_controller/trajectory_speed_profile.hpp"

namespace jetpilot_controller
{
namespace
{

TrackingInput straight_path()
{
  return {{{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}}, 0.0};
}

TEST(PurePursuit, TracksStraightPathWithoutSteering)
{
  PurePursuit controller(PurePursuitParams{});
  const auto result = controller.compute(straight_path());

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.steering_command, 0.0, 1.0e-9);
  EXPECT_NEAR(result.curvature, 0.0, 1.0e-9);
  EXPECT_EQ(result.target_index, 1U);
}

TEST(PurePursuit, SteersTowardLeftAndRightTargets)
{
  PurePursuitParams params;
  params.min_lookahead_m = 0.2;
  PurePursuit controller(params);

  const auto left = controller.compute({{{0.0, 0.0}, {1.0, 0.5}, {2.0, 1.0}}, 0.0});
  const auto right = controller.compute({{{0.0, 0.0}, {1.0, -0.5}, {2.0, -1.0}}, 0.0});

  ASSERT_TRUE(left.valid) << left.reason;
  ASSERT_TRUE(right.valid) << right.reason;
  EXPECT_GT(left.steering_command, 0.0);
  EXPECT_LT(right.steering_command, 0.0);
}

TEST(PurePursuit, GrowsAndClampsLookaheadWithSpeed)
{
  PurePursuitParams params;
  params.min_lookahead_m = 0.5;
  params.max_lookahead_m = 1.5;
  params.lookahead_speed_gain_s = 0.5;
  PurePursuit controller(params);

  auto input = straight_path();
  input.speed_mps = 10.0;
  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_DOUBLE_EQ(result.lookahead_distance_m, 1.5);
  EXPECT_EQ(result.target_index, 3U);
}

TEST(PurePursuit, DetectsAndWrapsClosedPath)
{
  PurePursuitParams params;
  params.min_lookahead_m = 0.75;
  params.closed_path_tolerance_m = 0.1;
  PurePursuit controller(params);
  TrackingInput input{{{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}, {1.0, 0.0}}, 0.0};

  const auto result = controller.compute(input);

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.path_closed);
}

TEST(PurePursuit, SupportsExplicitPathClosureMode)
{
  PurePursuitParams closed_params;
  closed_params.path_closure_mode = PathClosureMode::kClosed;
  PurePursuit closed_controller(closed_params);
  const auto closed = closed_controller.compute({{{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}}, 0.0});
  ASSERT_TRUE(closed.valid) << closed.reason;
  EXPECT_TRUE(closed.path_closed);

  PurePursuitParams open_params;
  open_params.path_closure_mode = PathClosureMode::kOpen;
  open_params.closed_path_tolerance_m = 10.0;
  PurePursuit open_controller(open_params);
  const auto open = open_controller.compute({{{0.0, 0.0}, {0.5, 0.0}, {0.0, 0.0}}, 0.0});
  EXPECT_FALSE(open.path_closed);
}

TEST(PurePursuit, TypedTrajectoryClosureOverridesConfiguredMode)
{
  PurePursuitParams params;
  params.path_closure_mode = PathClosureMode::kClosed;
  PurePursuit controller(params);
  TrackingInput input{{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}, 0.0};
  input.path_closed_override = false;

  const auto result = controller.compute(input);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_FALSE(result.path_closed);
}

TEST(PurePursuit, RejectsUnsafeInputs)
{
  PurePursuit controller(PurePursuitParams{});
  EXPECT_FALSE(controller.compute({{{0.0, 0.0}}, 0.0}).valid);
  EXPECT_FALSE(controller.compute({{{-0.5, 0.0}, {-1.0, 0.0}}, 0.0}).valid);
  EXPECT_FALSE(
    controller.compute({{{0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}}, 0.0})
    .valid);
  EXPECT_THROW((void)PurePursuit{PurePursuitParams{0.0}}, std::invalid_argument);

  PurePursuitParams invalid_params;
  invalid_params.max_lookahead_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)PurePursuit{invalid_params}, std::invalid_argument);
}

TEST(MapPursuit, SteersTowardLeftAndRightPaths)
{
  MapPursuitParams params;
  params.min_lookahead_m = 0.2;
  params.lateral_error_gain = 0.4;
  MapPursuit controller(params);

  const auto left = controller.compute({{{0.0, 0.2}, {1.0, 0.2}, {2.0, 0.2}}, 0.0});
  const auto right = controller.compute({{{0.0, -0.2}, {1.0, -0.2}, {2.0, -0.2}}, 0.0});

  ASSERT_TRUE(left.valid) << left.reason;
  ASSERT_TRUE(right.valid) << right.reason;
  EXPECT_GT(left.steering_command, 0.0);
  EXPECT_LT(right.steering_command, 0.0);
}

TEST(MapPursuit, DownscalesSteeringAtHighSpeed)
{
  MapPursuitParams params;
  params.min_lookahead_m = 0.2;
  params.lookahead_speed_gain_s = 0.0;
  params.speed_steering_downscale_start_mps = 1.0;
  params.speed_steering_downscale_end_mps = 2.0;
  params.speed_steering_downscale_factor = 0.5;
  MapPursuit controller(params);

  const TrackingInput input{{{0.0, 0.2}, {1.0, 0.2}, {2.0, 0.2}}, 0.5};
  auto fast_input = input;
  fast_input.speed_mps = 3.0;

  const auto slow = controller.compute(input);
  const auto fast = controller.compute(fast_input);

  ASSERT_TRUE(slow.valid) << slow.reason;
  ASSERT_TRUE(fast.valid) << fast.reason;
  EXPECT_LT(std::abs(fast.steering_command), std::abs(slow.steering_command));
}

TEST(KinematicMpc, TracksStraightPathWithoutSteering)
{
  KinematicMpcParams params;
  params.steering_samples = 15;
  KinematicMpc controller(params);

  const auto result = controller.compute(straight_path());

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.steering_command, 0.0, 1.0e-9);
}

TEST(KinematicMpc, SteersTowardLeftAndRightPaths)
{
  KinematicMpcParams params;
  params.steering_samples = 15;
  KinematicMpc controller(params);

  const auto left = controller.compute({{{0.0, 0.2}, {1.0, 0.2}, {2.0, 0.2}}, 1.0});
  const auto right = controller.compute({{{0.0, -0.2}, {1.0, -0.2}, {2.0, -0.2}}, 1.0});

  ASSERT_TRUE(left.valid) << left.reason;
  ASSERT_TRUE(right.valid) << right.reason;
  EXPECT_GT(left.steering_command, 0.0);
  EXPECT_LT(right.steering_command, 0.0);
}

TEST(LongitudinalController, AcceleratesAndRespectsThrottleLimit)
{
  LongitudinalParams params;
  params.max_throttle_command = 0.25;
  LongitudinalController controller(params);

  const auto command = controller.compute(2.0, 0.0);
  EXPECT_DOUBLE_EQ(command.throttle, 0.25);
  EXPECT_DOUBLE_EQ(command.brake, 0.0);
}

TEST(LongitudinalController, BrakesOverspeedAndRespectsLimit)
{
  LongitudinalParams params;
  params.max_brake_command = 0.2;
  LongitudinalController controller(params);

  const auto command = controller.compute(0.5, 2.0);
  EXPECT_DOUBLE_EQ(command.throttle, 0.0);
  EXPECT_DOUBLE_EQ(command.brake, 0.2);
}

TEST(LongitudinalController, HoldsFeedforwardInsideDeadband)
{
  LongitudinalController controller(LongitudinalParams{});
  const auto command = controller.compute(1.0, 1.01);
  EXPECT_DOUBLE_EQ(command.throttle, 0.05);
  EXPECT_DOUBLE_EQ(command.brake, 0.0);
}

TEST(LongitudinalController, IntegralTermBuildsAndResetClearsIt)
{
  LongitudinalParams params;
  params.throttle_kp = 0.0;
  params.throttle_ki = 0.5;
  params.throttle_kd = 0.0;
  params.throttle_feedforward = 0.2;
  params.speed_deadband_mps = 0.0;
  params.brake_activation_error_mps = 1.0;
  params.max_throttle_command = 1.0;
  LongitudinalController controller(params);

  EXPECT_NEAR(controller.compute(1.0, 0.5, 0.0, 1.0).throttle, 0.45, 1.0e-9);
  EXPECT_NEAR(controller.compute(1.0, 0.5, 0.0, 1.0).throttle, 0.70, 1.0e-9);
  EXPECT_DOUBLE_EQ(controller.compute(0.0, 0.0, 0.0, 1.0).throttle, 0.0);
  EXPECT_NEAR(controller.compute(1.0, 0.5, 0.0, 1.0).throttle, 0.45, 1.0e-9);
}

TEST(LongitudinalController, AppliesOptionalAccelerationFeedforward)
{
  LongitudinalParams params;
  params.throttle_acceleration_feedforward = 0.1;
  params.brake_deceleration_feedforward = 0.2;
  LongitudinalController controller(params);

  const auto accelerating = controller.compute(1.0, 1.0, 0.5);
  EXPECT_NEAR(accelerating.throttle, 0.1, 1.0e-9);
  EXPECT_DOUBLE_EQ(accelerating.brake, 0.0);

  const auto braking = controller.compute(1.0, 1.0, -0.5);
  EXPECT_DOUBLE_EQ(braking.throttle, 0.0);
  EXPECT_NEAR(braking.brake, 0.1, 1.0e-9);
}

TEST(TrajectorySpeedProfile, InterpolatesAndLooksAheadConservatively)
{
  const std::vector<TrajectoryProfilePoint> profile{
    {0.0, 2.0, -1.5}, {1.0, 1.0, 1.5}, {2.0, 2.0, 0.0}};

  const auto interpolated = sample_trajectory_profile(profile, 0.5, 0.0, false, 2.0);
  ASSERT_TRUE(interpolated.valid) << interpolated.reason;
  EXPECT_NEAR(interpolated.speed_mps, std::sqrt(2.5), 1.0e-9);
  EXPECT_DOUBLE_EQ(interpolated.acceleration_mps2, -1.5);

  const auto lookahead = sample_trajectory_profile(profile, 0.25, 1.0, false, 2.0);
  ASSERT_TRUE(lookahead.valid) << lookahead.reason;
  EXPECT_DOUBLE_EQ(lookahead.speed_mps, 1.0);
  EXPECT_DOUBLE_EQ(lookahead.acceleration_mps2, -1.5);
}

TEST(TrajectorySpeedProfile, WrapsClosedProfileLookahead)
{
  const std::vector<TrajectoryProfilePoint> profile{
    {0.0, 0.5, 1.875}, {1.0, 2.0, 0.0}, {2.0, 2.0, -1.875}};

  const auto sample = sample_trajectory_profile(profile, 2.4, 0.8, true, 3.0);
  ASSERT_TRUE(sample.valid) << sample.reason;
  EXPECT_DOUBLE_EQ(sample.speed_mps, 0.5);
  EXPECT_DOUBLE_EQ(sample.acceleration_mps2, -1.875);
}

TEST(TrajectorySpeedProfile, InterpolatesClosedSeamWithConstantAcceleration)
{
  const std::vector<TrajectoryProfilePoint> profile{
    {0.0, 1.0, 1.5}, {1.0, 2.0, 2.5}, {2.0, 3.0, -4.0}};

  const auto sample = sample_trajectory_profile(profile, 2.5, 0.0, true, 3.0);
  ASSERT_TRUE(sample.valid) << sample.reason;
  EXPECT_NEAR(sample.speed_mps, std::sqrt(5.0), 1.0e-9);
  EXPECT_DOUBLE_EQ(sample.acceleration_mps2, -4.0);
}

TEST(TrajectorySpeedProfile, RejectsInvalidOrUnorderedPoints)
{
  EXPECT_FALSE(sample_trajectory_profile(
    {{0.0, 1.0, 0.0}, {1.0, -1.0, 0.0}}, 0.0, 0.5, false, 1.0).valid);
  EXPECT_FALSE(sample_trajectory_profile(
    {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}}, 0.0, 0.5, false, 1.0).valid);
}

TEST(LongitudinalController, RejectsNonFiniteParameters)
{
  LongitudinalParams params;
  params.brake_kp = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)LongitudinalController{params}, std::invalid_argument);
}

TEST(TrailingController, LimitsSpeedToMaintainGap)
{
  TrailingParams params;
  params.enabled = true;
  params.trailing_gap_m = 1.5;
  params.kp = 0.5;
  params.ki = 0.0;
  params.kd = 0.2;
  TrailingController controller(params);

  TrailingInput input;
  input.planned_speed_mps = 3.0;
  input.ego_speed_mps = 2.0;
  input.ego_station_m = 10.0;
  input.opponent_station_m = 11.0;
  input.opponent_speed_mps = 1.0;
  input.track_length_m = 20.0;
  input.path_closed = true;
  input.dt_s = 0.1;

  const auto result = controller.compute(input);

  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.gap_m, 1.0, 1.0e-9);
  EXPECT_LT(result.target_speed_mps, input.opponent_speed_mps);
  EXPECT_GE(result.target_speed_mps, 0.0);
}

TEST(TrailingController, IgnoresOpponentBehindOnOpenPath)
{
  TrailingParams params;
  params.enabled = true;
  TrailingController controller(params);

  TrailingInput input;
  input.planned_speed_mps = 2.0;
  input.ego_speed_mps = 1.0;
  input.ego_station_m = 5.0;
  input.opponent_station_m = 4.0;
  input.opponent_speed_mps = 1.0;
  input.track_length_m = 10.0;
  input.path_closed = false;

  const auto result = controller.compute(input);

  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.target_speed_mps, input.planned_speed_mps);
}

TEST(TrailingController, WrapsGapOnClosedPath)
{
  TrailingParams params;
  params.enabled = true;
  params.max_gap_m = 3.0;
  params.trailing_gap_m = 1.5;
  params.kp = 0.5;
  params.ki = 0.0;
  params.kd = 0.0;
  TrailingController controller(params);

  TrailingInput input;
  input.planned_speed_mps = 2.0;
  input.ego_speed_mps = 1.0;
  input.ego_station_m = 9.0;
  input.opponent_station_m = 0.5;
  input.opponent_speed_mps = 1.0;
  input.track_length_m = 10.0;
  input.path_closed = true;

  const auto result = controller.compute(input);

  EXPECT_TRUE(result.active);
  EXPECT_NEAR(result.gap_m, 1.5, 1.0e-9);
}

TEST(MotionDirection, ConvertsReversePathAndSteeringBackToVehicleFrame)
{
  std::vector<Point2d> path{{0.0, 0.0}, {-1.0, 0.2}};
  orient_path_for_motion(path, true);
  EXPECT_DOUBLE_EQ(path[1].x, 1.0);
  EXPECT_DOUBLE_EQ(path[1].y, -0.2);

  PurePursuit controller(PurePursuitParams{});
  TrackingInput input;
  input.path = path;
  input.path_closed_override = false;
  const auto tracking = controller.compute(input);
  ASSERT_TRUE(tracking.valid) << tracking.reason;
  EXPECT_LT(tracking.steering_command, 0.0);
  EXPECT_GT(steering_for_motion(tracking.steering_command, true), 0.0);

  const auto lookahead = point_from_motion_frame(tracking.target_point, true);
  EXPECT_LT(lookahead.x, 0.0);
  EXPECT_GT(lookahead.y, 0.0);
}

}  // namespace
}  // namespace jetpilot_controller
