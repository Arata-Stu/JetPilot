#include "jetpilot_controller/kinematic_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace jetpilot_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Projection
{
  bool valid{false};
  std::size_t index{0};
  double distance_sq{0.0};
  double heading_rad{0.0};
  Point2d point;
};

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

double normalize_angle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

Projection project_to_path(
  const std::vector<Point2d> & path, const Point2d & point, bool path_closed)
{
  Projection best;
  if (path.size() < 2U) {
    return best;
  }

  best.distance_sq = std::numeric_limits<double>::infinity();
  const std::size_t segment_count = path_closed ? path.size() : path.size() - 1U;
  for (std::size_t index = 0; index < segment_count; ++index) {
    const auto & a = path[index];
    const auto & b = path[(index + 1U) % path.size()];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq <= 1.0e-12) {
      continue;
    }

    const double t = std::clamp(
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / length_sq, 0.0, 1.0);
    const Point2d projected{a.x + t * dx, a.y + t * dy};
    const double distance_sq =
      (point.x - projected.x) * (point.x - projected.x) +
      (point.y - projected.y) * (point.y - projected.y);
    if (distance_sq < best.distance_sq) {
      best.valid = true;
      best.index = index;
      best.distance_sq = distance_sq;
      best.heading_rad = std::atan2(dy, dx);
      best.point = projected;
    }
  }
  return best;
}

}  // namespace

KinematicMpc::KinematicMpc(KinematicMpcParams params)
: params_(params)
{
  for (const double value : {
      params_.wheelbase_m, params_.max_steering_angle_rad, params_.max_steering_command,
      params_.time_step_s, params_.min_prediction_speed_mps, params_.path_error_weight,
      params_.heading_error_weight, params_.steering_weight, params_.terminal_path_error_weight,
      params_.closed_path_tolerance_m})
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Kinematic MPC parameters must be finite");
    }
  }
  if (!(params_.wheelbase_m > 0.0)) {
    throw std::invalid_argument("wheelbase_m must be > 0");
  }
  if (!(params_.max_steering_angle_rad > 0.0)) {
    throw std::invalid_argument("max_steering_angle_rad must be > 0");
  }
  if (!(params_.max_steering_command > 0.0 && params_.max_steering_command <= 1.0)) {
    throw std::invalid_argument("max_steering_command must be in (0, 1]");
  }
  if (params_.horizon_steps < 1U) {
    throw std::invalid_argument("kinematic MPC horizon_steps must be >= 1");
  }
  if (params_.steering_samples < 3U) {
    throw std::invalid_argument("kinematic MPC steering_samples must be >= 3");
  }
  if (!(params_.time_step_s > 0.0)) {
    throw std::invalid_argument("kinematic MPC time_step_s must be > 0");
  }
  if (params_.min_prediction_speed_mps < 0.0) {
    throw std::invalid_argument("min_prediction_speed_mps must be >= 0");
  }
  if (
    params_.path_error_weight < 0.0 || params_.heading_error_weight < 0.0 ||
    params_.steering_weight < 0.0 || params_.terminal_path_error_weight < 0.0)
  {
    throw std::invalid_argument("kinematic MPC cost weights must be >= 0");
  }
  if (params_.closed_path_tolerance_m < 0.0) {
    throw std::invalid_argument("closed_path_tolerance_m must be >= 0");
  }
}

const KinematicMpcParams & KinematicMpc::params() const noexcept
{
  return params_;
}

TrackingResult KinematicMpc::compute(const TrackingInput & input) const
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

  result.path_closed = is_closed_path(
    input.path, params_.path_closure_mode, params_.closed_path_tolerance_m);
  const auto ego_projection = project_to_path(input.path, Point2d{0.0, 0.0}, result.path_closed);
  if (!ego_projection.valid) {
    result.reason = "could not project ego onto path";
    return result;
  }
  result.nearest_index = ego_projection.index;
  result.lookahead_distance_m =
    std::max(std::abs(input.speed_mps), params_.min_prediction_speed_mps) *
    params_.time_step_s * static_cast<double>(params_.horizon_steps);

  double best_cost = std::numeric_limits<double>::infinity();
  double best_steering_angle = 0.0;
  Projection best_terminal_projection;
  const double speed = std::max(std::abs(input.speed_mps), params_.min_prediction_speed_mps);
  const double max_steering = params_.max_steering_angle_rad;
  const double sample_denominator = static_cast<double>(params_.steering_samples - 1U);

  for (std::size_t sample = 0; sample < params_.steering_samples; ++sample) {
    const double ratio = static_cast<double>(sample) / sample_denominator;
    const double steering_angle = -max_steering + 2.0 * max_steering * ratio;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double cost = params_.steering_weight * steering_angle * steering_angle;
    Projection terminal_projection;

    for (std::size_t step = 0; step < params_.horizon_steps; ++step) {
      x += speed * std::cos(yaw) * params_.time_step_s;
      y += speed * std::sin(yaw) * params_.time_step_s;
      yaw = normalize_angle(
        yaw + speed / params_.wheelbase_m * std::tan(steering_angle) * params_.time_step_s);

      terminal_projection = project_to_path(input.path, Point2d{x, y}, result.path_closed);
      if (!terminal_projection.valid) {
        cost = std::numeric_limits<double>::infinity();
        break;
      }
      const double heading_error = normalize_angle(yaw - terminal_projection.heading_rad);
      cost += params_.path_error_weight * terminal_projection.distance_sq +
        params_.heading_error_weight * heading_error * heading_error +
        params_.steering_weight * steering_angle * steering_angle;
    }

    if (terminal_projection.valid) {
      cost += params_.terminal_path_error_weight * terminal_projection.distance_sq;
    }
    if (cost < best_cost) {
      best_cost = cost;
      best_steering_angle = steering_angle;
      best_terminal_projection = terminal_projection;
    }
  }

  if (!std::isfinite(best_cost) || !best_terminal_projection.valid) {
    result.reason = "kinematic MPC could not find a finite rollout";
    return result;
  }

  result.target_index = best_terminal_projection.index;
  result.target_point = best_terminal_projection.point;
  if (!result.path_closed && result.target_point.x < 0.0) {
    result.reason = "open-path MPC target is behind the vehicle";
    return result;
  }
  result.steering_angle_rad = best_steering_angle;
  result.curvature = std::tan(best_steering_angle) / params_.wheelbase_m;
  result.steering_command = std::clamp(
    best_steering_angle / params_.max_steering_angle_rad * params_.max_steering_command,
    -params_.max_steering_command, params_.max_steering_command);
  result.valid = true;
  result.reason = "tracking";
  return result;
}

}  // namespace jetpilot_controller
