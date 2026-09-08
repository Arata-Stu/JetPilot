#ifndef JETPILOT_LOCALIZATION_MANAGER__STREAM_OBSERVATION_HPP_
#define JETPILOT_LOCALIZATION_MANAGER__STREAM_OBSERVATION_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace jetpilot_localization_manager
{
// A repeated old sample must not make a stopped stream look healthy.
struct StreamObservation
{
  using Clock = std::chrono::steady_clock;
  std::optional<Clock::time_point> advanced_at;
  std::int64_t stamp_ns{0};

  void observe(std::int64_t stamp, Clock::time_point received)
  {
    if (!advanced_at || stamp != stamp_ns)
    {
      stamp_ns = stamp;
      advanced_at = received;
    }
  }

  bool fresh(std::int64_t now_ns, Clock::time_point steady_now, double timeout) const
  {
    const double stamp_age = (static_cast<double>(now_ns) - stamp_ns) / 1e9;
    return advanced_at && stamp_age >= 0.0 && stamp_age <= timeout &&
           std::chrono::duration<double>(steady_now - *advanced_at).count() <= timeout;
  }
};
}  // namespace jetpilot_localization_manager
#endif
