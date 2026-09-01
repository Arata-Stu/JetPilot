#include "jetpilot_signal_detection/signal_vote_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace jetpilot_signal_detection
{

SignalVoteFilter::SignalVoteFilter(std::size_t window_size, std::size_t minimum_votes)
: window_size_(window_size), minimum_votes_(minimum_votes)
{
  if (window_size_ == 0U || minimum_votes_ == 0U || minimum_votes_ > window_size_)
  {
    throw std::invalid_argument("vote window requires 0 < minimum_votes <= window_size");
  }
}

void SignalVoteFilter::reset() { votes_.clear(); }

SignalDecision SignalVoteFilter::add(SignalVote vote)
{
  if (vote.direction > 3U || !std::isfinite(vote.confidence))
  {
    throw std::invalid_argument("invalid signal vote");
  }
  votes_.push_back(vote);
  while (votes_.size() > window_size_) votes_.pop_front();
  return decision();
}

SignalDecision SignalVoteFilter::decision() const
{
  std::array<std::size_t, 4U> counts{};
  std::array<double, 4U> confidence_sums{};
  for (const auto & vote : votes_)
  {
    if (vote.direction == 0U) continue;
    ++counts[vote.direction];
    confidence_sums[vote.direction] += vote.confidence;
  }
  std::uint8_t best = 0U;
  for (std::uint8_t direction = 1U; direction <= 3U; ++direction)
  {
    if (counts[direction] > counts[best] ||
        (counts[direction] == counts[best] && confidence_sums[direction] > confidence_sums[best]))
    {
      best = direction;
    }
  }
  SignalDecision output;
  output.direction = best;
  output.votes = counts[best];
  output.stable = best != 0U && output.votes >= minimum_votes_;
  output.confidence = output.votes == 0U ? 0.0 : confidence_sums[best] / output.votes;
  return output;
}

}  // namespace jetpilot_signal_detection
