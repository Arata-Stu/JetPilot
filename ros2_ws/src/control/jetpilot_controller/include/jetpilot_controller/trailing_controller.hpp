#pragma once

#include <string>

namespace jetpilot_controller
{

struct TrailingParams
{
  bool enabled{false};
  double trailing_gap_m{1.5};
  double kp{0.5};
  double ki{0.001};
  double kd{0.2};
  double max_gap_m{8.0};
  double min_command_speed_mps{0.0};
};

struct TrailingInput
{
  double planned_speed_mps{0.0};
  double ego_speed_mps{0.0};
  double ego_station_m{0.0};
  double opponent_station_m{0.0};
  double opponent_speed_mps{0.0};
  double track_length_m{0.0};
  bool path_closed{false};
  double dt_s{0.0};
};

struct TrailingResult
{
  bool active{false};
  double target_speed_mps{0.0};
  double gap_m{0.0};
  std::string reason;
};

class TrailingController
{
public:
  explicit TrailingController(TrailingParams params);

  TrailingResult compute(const TrailingInput & input);
  const TrailingParams & params() const noexcept;
  void reset() noexcept;

private:
  TrailingParams params_;
  double integral_gap_error_{0.0};
};

}  // namespace jetpilot_controller
