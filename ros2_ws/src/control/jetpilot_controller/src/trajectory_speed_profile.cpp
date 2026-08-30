#include "jetpilot_controller/trajectory_speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace jetpilot_controller
{
namespace
{

TrajectoryProfileSample interpolate(
  const std::vector<TrajectoryProfilePoint> & profile, double station_m, const bool closed,
  const double loop_length_m)
{
  if (closed)
  {
    station_m = std::fmod(station_m, loop_length_m);
    if (station_m < 0.0)
    {
      station_m += loop_length_m;
    }
  }
  else
  {
    station_m = std::clamp(station_m, profile.front().station_m, profile.back().station_m);
  }

  auto upper = std::upper_bound(
    profile.begin(), profile.end(), station_m,
    [](const double station, const TrajectoryProfilePoint & point)
    { return station < point.station_m; });

  const TrajectoryProfilePoint * lower_point = nullptr;
  const TrajectoryProfilePoint * upper_point = nullptr;
  double lower_station = 0.0;
  double upper_station = 0.0;
  if (upper == profile.begin())
  {
    lower_point = &profile.front();
    upper_point = &profile.front();
    lower_station = upper_station = profile.front().station_m;
  }
  else if (upper == profile.end())
  {
    lower_point = &profile.back();
    lower_station = profile.back().station_m;
    if (closed && loop_length_m > lower_station)
    {
      upper_point = &profile.front();
      upper_station = loop_length_m + profile.front().station_m;
    }
    else
    {
      upper_point = lower_point;
      upper_station = lower_station;
    }
  }
  else
  {
    lower_point = &*(upper - 1);
    upper_point = &*upper;
    lower_station = lower_point->station_m;
    upper_station = upper_point->station_m;
  }

  double ratio = 0.0;
  if (upper_station > lower_station)
  {
    ratio = std::clamp(
      (station_m - lower_station) / (upper_station - lower_station), 0.0, 1.0);
  }
  TrajectoryProfileSample result;
  result.valid = true;
  // The compiled CSV defines ax as constant over the outgoing segment:
  // ax = (v1^2 - v0^2) / (2 * ds). Therefore v^2, rather than v, is linear
  // in station. hypot evaluates the equivalent square-root expression while
  // avoiding overflow for otherwise finite input speeds.
  result.speed_mps = std::hypot(
    std::sqrt(1.0 - ratio) * lower_point->speed_mps,
    std::sqrt(ratio) * upper_point->speed_mps);
  result.acceleration_mps2 = lower_point->acceleration_mps2;
  result.reason = "ok";
  return result;
}

}  // namespace

TrajectoryProfileSample sample_trajectory_profile(
  const std::vector<TrajectoryProfilePoint> & profile, const double station_m,
  const double lookahead_m, const bool closed, const double loop_length_m)
{
  if (profile.size() < 2U || !std::isfinite(station_m) || !std::isfinite(lookahead_m) ||
      lookahead_m < 0.0 || !std::isfinite(loop_length_m))
  {
    return {false, 0.0, 0.0, "invalid_profile_input"};
  }
  for (std::size_t index = 0U; index < profile.size(); ++index)
  {
    const auto & point = profile[index];
    if (!std::isfinite(point.station_m) || !std::isfinite(point.speed_mps) ||
        !std::isfinite(point.acceleration_mps2) || point.speed_mps < 0.0 ||
        (index > 0U && point.station_m <= profile[index - 1U].station_m))
    {
      return {false, 0.0, 0.0, "invalid_profile_point"};
    }
  }
  if ((closed && (loop_length_m < profile.back().station_m || loop_length_m <= 0.0)) ||
      (!closed && profile.back().station_m <= profile.front().station_m))
  {
    return {false, 0.0, 0.0, "invalid_profile_length"};
  }

  auto sample = interpolate(profile, station_m, closed, loop_length_m);
  if (!sample.valid || lookahead_m == 0.0)
  {
    return sample;
  }

  const double capped_lookahead = closed ? std::min(lookahead_m, loop_length_m) : lookahead_m;
  const auto end_sample = interpolate(profile, station_m + capped_lookahead, closed, loop_length_m);
  if (end_sample.speed_mps < sample.speed_mps)
  {
    sample.speed_mps = end_sample.speed_mps;
    sample.acceleration_mps2 = std::min(
      {sample.acceleration_mps2, end_sample.acceleration_mps2, 0.0});
  }

  double normalized_station = station_m;
  if (closed)
  {
    normalized_station = std::fmod(normalized_station, loop_length_m);
    if (normalized_station < 0.0)
    {
      normalized_station += loop_length_m;
    }
  }
  for (const auto & point : profile)
  {
    double forward_distance = point.station_m - normalized_station;
    if (closed && forward_distance < 0.0)
    {
      forward_distance += loop_length_m;
    }
    if (forward_distance >= 0.0 && forward_distance <= capped_lookahead)
    {
      if (point.speed_mps < sample.speed_mps)
      {
        sample.speed_mps = point.speed_mps;
        sample.acceleration_mps2 = std::min(
          {sample.acceleration_mps2, point.acceleration_mps2, 0.0});
      }
    }
  }
  return sample;
}

}  // namespace jetpilot_controller
