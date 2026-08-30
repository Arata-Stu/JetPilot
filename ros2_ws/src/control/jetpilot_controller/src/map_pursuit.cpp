#include "jetpilot_controller/map_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace jetpilot_controller
{
namespace
{

double distance(const Point2d & lhs, const Point2d & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

bool finite(const Point2d & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool is_closed_path(
  const std::vector<Point2d> & path, PathClosureMode mode, double tolerance_m)
{
  if (mode == PathClosureMode::kClosed) {
    return true;
  }
  if (mode == PathClosureMode::kOpen) {
    return false;
  }
  return tolerance_m > 0.0 && distance(path.front(), path.back()) <= tolerance_m;
}

double steering_speed_scale(
  double speed_mps, double start_mps, double end_mps, double downscale_factor)
{
  if (end_mps <= start_mps) {
    return 1.0;
  }
  const double ratio = std::clamp(
    (std::abs(speed_mps) - start_mps) / (end_mps - start_mps), 0.0, 1.0);
  return 1.0 - ratio * downscale_factor;
}

}  // namespace

MapPursuit::MapPursuit(MapPursuitParams params)
: params_(params)
{
  for (const double value : {
      params_.wheelbase_m, params_.min_lookahead_m, params_.max_lookahead_m,
      params_.lookahead_speed_gain_s, params_.lateral_error_gain,
      params_.max_steering_angle_rad, params_.max_steering_command,
      params_.speed_steering_downscale_start_mps, params_.speed_steering_downscale_end_mps,
      params_.speed_steering_downscale_factor, params_.closed_path_tolerance_m})
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Map Pursuit parameters must be finite");
    }
  }
  if (!(params_.wheelbase_m > 0.0)) {
    throw std::invalid_argument("wheelbase_m must be > 0");
  }
  if (!(params_.min_lookahead_m > 0.0)) {
    throw std::invalid_argument("min_lookahead_m must be > 0");
  }
  if (params_.max_lookahead_m < params_.min_lookahead_m) {
    throw std::invalid_argument("max_lookahead_m must be >= min_lookahead_m");
  }
  if (params_.lookahead_speed_gain_s < 0.0) {
    throw std::invalid_argument("lookahead_speed_gain_s must be >= 0");
  }
  if (!(params_.max_steering_angle_rad > 0.0)) {
    throw std::invalid_argument("max_steering_angle_rad must be > 0");
  }
  if (!(params_.max_steering_command > 0.0 && params_.max_steering_command <= 1.0)) {
    throw std::invalid_argument("max_steering_command must be in (0, 1]");
  }
  if (params_.speed_steering_downscale_factor < 0.0 ||
    params_.speed_steering_downscale_factor > 1.0)
  {
    throw std::invalid_argument("speed_steering_downscale_factor must be in [0, 1]");
  }
  if (params_.closed_path_tolerance_m < 0.0) {
    throw std::invalid_argument("closed_path_tolerance_m must be >= 0");
  }
}

const MapPursuitParams & MapPursuit::params() const noexcept
{
  return params_;
}

TrackingResult MapPursuit::compute(const TrackingInput & input) const
{
  TrackingResult result;
  if (!std::isfinite(input.speed_mps)) {
    result.reason = "speed is not finite";
    return result;
  }
  if (input.path.size() < 2U) {
    result.reason = "path requires at least two points";
    return result;
  }
  if (!std::all_of(input.path.begin(), input.path.end(), finite)) {
    result.reason = "path contains a non-finite point";
    return result;
  }

  const auto closest = std::min_element(
    input.path.begin(), input.path.end(), [](const Point2d & lhs, const Point2d & rhs) {
      return std::hypot(lhs.x, lhs.y) < std::hypot(rhs.x, rhs.y);
    });
  result.nearest_index = static_cast<std::size_t>(std::distance(input.path.begin(), closest));
  result.path_closed = input.path_closed_override.value_or(is_closed_path(
    input.path, params_.path_closure_mode, params_.closed_path_tolerance_m));
  result.lookahead_distance_m = std::clamp(
    params_.min_lookahead_m + params_.lookahead_speed_gain_s * std::abs(input.speed_mps),
    params_.min_lookahead_m, params_.max_lookahead_m);

  std::size_t current = result.nearest_index;
  double travelled = 0.0;
  const std::size_t maximum_segments = result.path_closed ? input.path.size() :
    input.path.size() - 1U - std::min(result.nearest_index, input.path.size() - 1U);
  for (std::size_t segment_count = 0; segment_count < maximum_segments; ++segment_count) {
    std::size_t next = current + 1U;
    if (next >= input.path.size()) {
      next = 0U;
    }
    travelled += distance(input.path[current], input.path[next]);
    current = next;
    if (travelled >= result.lookahead_distance_m) {
      break;
    }
  }

  result.target_index = current;
  result.target_point = input.path[current];
  if (!result.path_closed && result.target_point.x < 0.0) {
    result.reason = "open-path lookahead target is behind the vehicle";
    return result;
  }

  const double compensated_y = result.target_point.y + params_.lateral_error_gain * closest->y;
  const double target_distance_squared =
    result.target_point.x * result.target_point.x + compensated_y * compensated_y;
  if (target_distance_squared <= std::numeric_limits<double>::epsilon()) {
    result.reason = "lookahead target is at the vehicle origin";
    return result;
  }

  result.curvature = 2.0 * compensated_y / target_distance_squared;
  result.steering_angle_rad = std::atan(params_.wheelbase_m * result.curvature);
  const double normalized_steering = result.steering_angle_rad / params_.max_steering_angle_rad;
  const double scaled_steering = normalized_steering *
    steering_speed_scale(
      input.speed_mps, params_.speed_steering_downscale_start_mps,
      params_.speed_steering_downscale_end_mps, params_.speed_steering_downscale_factor);
  result.steering_command = std::clamp(
    scaled_steering, -params_.max_steering_command, params_.max_steering_command);
  result.valid = true;
  result.reason = "tracking";
  return result;
}

}  // namespace jetpilot_controller
