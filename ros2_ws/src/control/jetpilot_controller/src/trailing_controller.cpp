#include "jetpilot_controller/trailing_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace jetpilot_controller
{

TrailingController::TrailingController(TrailingParams params)
: params_(params)
{
  for (const double value : {
      params_.trailing_gap_m, params_.kp, params_.ki, params_.kd, params_.max_gap_m,
      params_.min_command_speed_mps})
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("trailing parameters must be finite");
    }
  }
  if (params_.trailing_gap_m < 0.0 || params_.max_gap_m <= 0.0 ||
    params_.min_command_speed_mps < 0.0)
  {
    throw std::invalid_argument(
            "trailing gap and minimum speed must be non-negative, max_gap_m must be > 0");
  }
}

TrailingResult TrailingController::compute(const TrailingInput & input)
{
  TrailingResult result;
  result.target_speed_mps = input.planned_speed_mps;

  if (!params_.enabled) {
    result.reason = "disabled";
    reset();
    return result;
  }

  for (const double value : {
      input.planned_speed_mps, input.ego_speed_mps, input.ego_station_m, input.opponent_station_m,
      input.opponent_speed_mps, input.track_length_m, input.dt_s})
  {
    if (!std::isfinite(value)) {
      result.reason = "invalid trailing input";
      reset();
      return result;
    }
  }
  if (
    input.planned_speed_mps < 0.0 || input.ego_speed_mps < 0.0 ||
    input.opponent_speed_mps < 0.0)
  {
    result.reason = "negative speed";
    reset();
    return result;
  }

  double gap = input.opponent_station_m - input.ego_station_m;
  if (input.path_closed) {
    if (input.track_length_m <= 0.0) {
      result.reason = "closed path has no length";
      reset();
      return result;
    }
    while (gap <= 0.0) {
      gap += input.track_length_m;
    }
  }

  result.gap_m = gap;
  if (gap <= 0.0) {
    result.reason = "opponent is not ahead";
    reset();
    return result;
  }
  if (gap > params_.max_gap_m) {
    result.reason = "opponent is beyond trailing range";
    reset();
    return result;
  }

  const double dt = std::max(input.dt_s, 0.0);
  if (dt > 0.0) {
    const double gap_error = params_.trailing_gap_m - gap;
    integral_gap_error_ = std::clamp(integral_gap_error_ + gap_error * dt, -10.0, 10.0);
  }

  const double gap_error = params_.trailing_gap_m - gap;
  const double speed_error = input.ego_speed_mps - input.opponent_speed_mps;
  const double pid =
    params_.kp * gap_error + params_.ki * integral_gap_error_ + params_.kd * speed_error;
  const double command = input.opponent_speed_mps - pid;

  result.active = true;
  result.reason = "trailing";
  result.target_speed_mps = std::clamp(
    command, params_.min_command_speed_mps, input.planned_speed_mps);
  return result;
}

const TrailingParams & TrailingController::params() const noexcept
{
  return params_;
}

void TrailingController::reset() noexcept
{
  integral_gap_error_ = 0.0;
}

}  // namespace jetpilot_controller
