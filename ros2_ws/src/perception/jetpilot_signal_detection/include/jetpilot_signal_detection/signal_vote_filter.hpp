#ifndef JETPILOT_SIGNAL_DETECTION__SIGNAL_VOTE_FILTER_HPP_
#define JETPILOT_SIGNAL_DETECTION__SIGNAL_VOTE_FILTER_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>

namespace jetpilot_signal_detection
{

struct SignalVote
{
  std::uint8_t direction{0U};
  double confidence{0.0};
};

struct SignalDecision
{
  std::uint8_t direction{0U};
  double confidence{0.0};
  bool stable{false};
  std::size_t votes{0U};
};

class SignalVoteFilter
{
public:
  SignalVoteFilter(std::size_t window_size, std::size_t minimum_votes);
  void reset();
  SignalDecision add(SignalVote vote);
  SignalDecision decision() const;

private:
  std::size_t window_size_;
  std::size_t minimum_votes_;
  std::deque<SignalVote> votes_;
};

}  // namespace jetpilot_signal_detection
#endif
