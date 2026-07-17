#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"
#include "jetpilot_planning/lane_selection_policy.hpp"

namespace
{

using jetpilot_planning::LaneSelectionPolicy;
using jetpilot_planning::SelectionSource;

TEST(LaneSelectionPolicy, UsesDefaultLaneWithoutConditions)
{
  LaneSelectionPolicy policy("primary", {}, true);
  const auto result = policy.select("", "straight", {"primary"});
  EXPECT_TRUE(result.ready());
  EXPECT_EQ(result.lane_id, "primary");
  EXPECT_EQ(result.source, SelectionSource::kDefaultLane);
}

TEST(LaneSelectionPolicy, RequestedLaneHasHighestPriority)
{
  LaneSelectionPolicy policy("primary", {{"shortcut_entry", "shortcut"}}, true);
  const auto result = policy.select(
    "avoidance", "shortcut_entry", {"primary", "shortcut", "avoidance"});
  EXPECT_EQ(result.lane_id, "avoidance");
  EXPECT_EQ(result.source, SelectionSource::kRequestedLane);
}

TEST(LaneSelectionPolicy, SectionRuleSelectsShortcut)
{
  LaneSelectionPolicy policy("primary", {{"shortcut_entry", "shortcut"}}, true);
  const auto result = policy.select("", "shortcut_entry", {"primary", "shortcut"});
  EXPECT_EQ(result.lane_id, "shortcut");
  EXPECT_EQ(result.source, SelectionSource::kSectionRule);
}

TEST(LaneSelectionPolicy, MissingConditionalLaneFallsBackToDefault)
{
  LaneSelectionPolicy policy("primary", {{"shortcut_entry", "shortcut"}}, true);
  const auto result = policy.select("", "shortcut_entry", {"primary"});
  EXPECT_EQ(result.lane_id, "primary");
  EXPECT_EQ(result.source, SelectionSource::kDefaultLane);
  EXPECT_EQ(result.reason, "section_lane_unavailable_fallback_to_default");
}

TEST(LaneSelectionPolicy, CanFailClosedWhenRequestedLaneIsUnavailable)
{
  LaneSelectionPolicy policy("primary", {}, false);
  const auto result = policy.select("avoidance", "", {"primary"});
  EXPECT_FALSE(result.ready());
  EXPECT_EQ(result.reason, "requested_lane_unavailable");
}

TEST(LaneSelectionPolicy, DoesNotChooseAnArbitraryLaneWhenDefaultIsMissing)
{
  LaneSelectionPolicy policy("primary", {}, true);
  const auto result = policy.select("", "", {"shortcut"});
  EXPECT_FALSE(result.ready());
  EXPECT_EQ(result.reason, "default_lane_unavailable");
}

TEST(LaneSelectionPolicy, ParsesTrimmedSectionRules)
{
  const auto rules = jetpilot_planning::parse_section_lane_rules(
    std::vector<std::string>{" shortcut_entry = shortcut ", "signal_red=pit_lane"});
  EXPECT_EQ(rules.at("shortcut_entry"), "shortcut");
  EXPECT_EQ(rules.at("signal_red"), "pit_lane");
}

TEST(LaneSelectionPolicy, RejectsMalformedOrDuplicateRules)
{
  EXPECT_THROW(
    jetpilot_planning::parse_section_lane_rules(std::vector<std::string>{"missing_separator"}),
    std::invalid_argument);
  EXPECT_THROW(
    jetpilot_planning::parse_section_lane_rules(
      std::vector<std::string>{"section=primary", "section=shortcut"}),
    std::invalid_argument);
}

}  // namespace
