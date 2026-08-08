#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "jetpilot_teleop_tools/button_rising_edge.hpp"
#include "jetpilot_teleop_tools/held_mode_selector.hpp"

namespace
{

using jetpilot_teleop_tools::ButtonRisingEdge;
using jetpilot_teleop_tools::ButtonManagerAssignments;
using jetpilot_teleop_tools::axis_direction_pressed;
using jetpilot_teleop_tools::find_button_conflict;
using jetpilot_teleop_tools::find_localization_button_conflict;
using jetpilot_teleop_tools::HeldMode;
using jetpilot_teleop_tools::HeldModeSelector;

TEST(ButtonRisingEdgeTest, EmitsOncePerPress)
{
  ButtonRisingEdge edge;
  edge.configure(1);

  EXPECT_FALSE(edge.update({0, 0}));
  EXPECT_TRUE(edge.update({0, 1}));
  EXPECT_FALSE(edge.update({0, 1}));
  EXPECT_FALSE(edge.update({0, 0}));
  EXPECT_TRUE(edge.update({0, 1}));
}

TEST(ButtonRisingEdgeTest, NegativeIndexDisablesBinding)
{
  ButtonRisingEdge edge;
  edge.configure(-1);

  EXPECT_FALSE(edge.enabled());
  EXPECT_FALSE(edge.update({1, 1}));
}

TEST(ButtonRisingEdgeTest, ExplicitDisablePreventsConflictingBinding)
{
  ButtonRisingEdge edge;
  edge.configure(7, false);

  EXPECT_FALSE(edge.enabled());
  EXPECT_FALSE(edge.update({0, 0, 0, 0, 0, 0, 0, 1}));
}

TEST(ButtonRisingEdgeTest, OutOfRangeIndexIsSafe)
{
  ButtonRisingEdge edge;
  edge.configure(10);

  EXPECT_FALSE(edge.update({1, 1}));
  EXPECT_FALSE(edge.update({}));
}

TEST(AxisDirectionTest, DetectsEitherDirectionPastThreshold)
{
  EXPECT_TRUE(axis_direction_pressed({0.0F, 0.8F}, 1, 1.0, 0.5));
  EXPECT_FALSE(axis_direction_pressed({0.0F, 0.2F}, 1, 1.0, 0.5));
  EXPECT_TRUE(axis_direction_pressed({0.0F, -0.8F}, 1, -1.0, 0.5));
  EXPECT_FALSE(axis_direction_pressed({0.0F, -0.2F}, 1, -1.0, 0.5));
}

TEST(AxisDirectionTest, InvalidBindingIsSafe)
{
  EXPECT_FALSE(axis_direction_pressed({1.0F}, -1, 1.0, 0.5));
  EXPECT_FALSE(axis_direction_pressed({1.0F}, 2, 1.0, 0.5));
  EXPECT_FALSE(axis_direction_pressed({1.0F}, 0, 0.0, 0.5));
}

TEST(ButtonRisingEdgeTest, FindsOnlyEnabledConflicts)
{
  const auto conflict = find_button_conflict(
    7, {{"auto_button", 0}, {"bag_start_button", 7}, {"disabled_button", -1}});
  ASSERT_TRUE(conflict.has_value());
  EXPECT_EQ(*conflict, "bag_start_button");

  EXPECT_FALSE(find_button_conflict(-1, {{"bag_start_button", -1}}).has_value());
  EXPECT_FALSE(find_button_conflict(7, {{"bag_start_button", -1}}).has_value());
}

TEST(ButtonRisingEdgeTest, LocalizationConflictPolicyAllowsBackModifierSharing)
{
  ButtonManagerAssignments assignments;
  assignments.auto_button = 0;
  assignments.back_button = 7;
  assignments.bag_start_button = 5;

  EXPECT_FALSE(find_localization_button_conflict(7, assignments).has_value());

  assignments.bag_start_button = 7;
  const auto conflict = find_localization_button_conflict(7, assignments);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_EQ(*conflict, "bag_start_button");
}

TEST(HeldModeSelectorTest, AutoIsActiveOnlyWhileAutoButtonIsHeld)
{
  HeldModeSelector selector;
  selector.set_hold_time(0.1);

  EXPECT_EQ(selector.update(true, false, false, 1.0), HeldMode::STOP);
  EXPECT_EQ(selector.update(true, false, false, 1.11), HeldMode::AUTO);
  EXPECT_EQ(selector.update(true, false, false, 1.2), HeldMode::AUTO);
  EXPECT_EQ(selector.update(false, false, false, 1.3), HeldMode::STOP);
}

TEST(HeldModeSelectorTest, ManualIsActiveOnlyWhileManualButtonIsHeld)
{
  HeldModeSelector selector;
  selector.set_hold_time(0.1);

  EXPECT_EQ(selector.update(false, true, false, 2.0), HeldMode::STOP);
  EXPECT_EQ(selector.update(false, true, false, 2.11), HeldMode::MANUAL);
  EXPECT_EQ(selector.update(false, false, false, 2.2), HeldMode::STOP);
}

TEST(HeldModeSelectorTest, StopAndConflictingModeButtonsFailSafeToStop)
{
  HeldModeSelector selector;
  selector.set_hold_time(0.1);

  EXPECT_EQ(selector.update(true, false, false, 3.0), HeldMode::STOP);
  EXPECT_EQ(selector.update(true, false, false, 3.11), HeldMode::AUTO);
  EXPECT_EQ(selector.update(true, true, false, 3.2), HeldMode::STOP);
  EXPECT_EQ(selector.update(true, false, true, 3.3), HeldMode::AUTO);
  EXPECT_EQ(selector.update(true, false, true, 3.41), HeldMode::STOP);
}

}  // namespace
