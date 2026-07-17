#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "jetpilot_teleop_tools/button_rising_edge.hpp"

namespace
{

using jetpilot_teleop_tools::ButtonRisingEdge;
using jetpilot_teleop_tools::ButtonManagerAssignments;
using jetpilot_teleop_tools::find_button_conflict;
using jetpilot_teleop_tools::find_localization_button_conflict;

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

}  // namespace
