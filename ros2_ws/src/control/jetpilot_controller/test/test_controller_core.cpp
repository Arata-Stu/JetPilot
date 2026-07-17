#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "jetpilot_controller/longitudinal_controller.hpp"
#include "jetpilot_controller/pure_pursuit.hpp"

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

TEST(LongitudinalController, HoldsNeutralInsideDeadband)
{
  LongitudinalController controller(LongitudinalParams{});
  const auto command = controller.compute(1.0, 1.01);
  EXPECT_DOUBLE_EQ(command.throttle, 0.0);
  EXPECT_DOUBLE_EQ(command.brake, 0.0);
}

TEST(LongitudinalController, RejectsNonFiniteParameters)
{
  LongitudinalParams params;
  params.brake_kp = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)LongitudinalController{params}, std::invalid_argument);
}

}  // namespace
}  // namespace jetpilot_controller
