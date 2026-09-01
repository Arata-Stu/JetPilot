#include <gtest/gtest.h>
#include "jetpilot_signal_detection/signal_vote_filter.hpp"
using jetpilot_signal_detection::SignalVoteFilter;
TEST(SignalVoteFilter, RequiresConfiguredConsensus)
{
  SignalVoteFilter filter(5U, 3U);
  EXPECT_FALSE(filter.add({1U, 0.8}).stable);
  EXPECT_FALSE(filter.add({2U, 0.9}).stable);
  EXPECT_FALSE(filter.add({1U, 0.7}).stable);
  const auto result = filter.add({1U, 0.9});
  EXPECT_TRUE(result.stable);
  EXPECT_EQ(result.direction, 1U);
  EXPECT_EQ(result.votes, 3U);
}
TEST(SignalVoteFilter, WindowForgetsOldVotes)
{
  SignalVoteFilter filter(3U, 2U);
  filter.add({1U, 0.9});
  filter.add({1U, 0.9});
  filter.add({2U, 0.9});
  filter.add({2U, 0.9});
  const auto result = filter.add({2U, 0.9});
  EXPECT_TRUE(result.stable);
  EXPECT_EQ(result.direction, 2U);
}
