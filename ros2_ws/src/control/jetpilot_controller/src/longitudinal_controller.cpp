#include "jetpilot_controller/longitudinal_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace jetpilot_controller
{

LongitudinalController::LongitudinalController(LongitudinalParams params)
: params_(params)
{
  if (
    !std::isfinite(params_.throttle_kp) ||
    !std::isfinite(params_.throttle_feedforward) ||
    !std::isfinite(params_.throttle_acceleration_feedforward) ||
    !std::isfinite(params_.brake_kp) ||
    !std::isfinite(params_.brake_deceleration_feedforward) ||
    !std::isfinite(params_.speed_deadband_mps) ||
    !std::isfinite(params_.max_throttle_command) ||
    !std::isfinite(params_.max_brake_command))
  {
    throw std::invalid_argument("longitudinal parameters must be finite");
  }
  if (
    params_.throttle_kp < 0.0 || params_.throttle_feedforward < 0.0 ||
    params_.throttle_acceleration_feedforward < 0.0 || params_.brake_kp < 0.0 ||
    params_.brake_deceleration_feedforward < 0.0 || params_.speed_deadband_mps < 0.0)
  {
    throw std::invalid_argument("longitudinal gains and deadband must be >= 0");
  }
  if (
    params_.max_throttle_command < 0.0 || params_.max_throttle_command > 1.0 ||
    params_.max_brake_command < 0.0 || params_.max_brake_command > 1.0)
  {
    throw std::invalid_argument("longitudinal command limits must be in [0, 1]");
  }
}

const LongitudinalParams & LongitudinalController::params() const noexcept
{
  return params_;
}

LongitudinalCommand LongitudinalController::compute(
  double target_speed_mps, double current_speed_mps, double target_acceleration_mps2) const
{
  LongitudinalCommand command;
  if (!std::isfinite(target_speed_mps) || !std::isfinite(current_speed_mps) ||
    !std::isfinite(target_acceleration_mps2)) {
    return command;
  }

  const double target = std::max(0.0, target_speed_mps);
  const double speed = std::max(0.0, current_speed_mps);
  const double error = target - speed;
  if (error > params_.speed_deadband_mps ||
    (std::abs(error) <= params_.speed_deadband_mps && target_acceleration_mps2 > 0.0)) {
    command.throttle = std::clamp(
      (error > params_.speed_deadband_mps ? params_.throttle_feedforward : 0.0) +
      params_.throttle_kp * std::max(0.0, error) +
      params_.throttle_acceleration_feedforward * std::max(0.0, target_acceleration_mps2),
      0.0, params_.max_throttle_command);
  } else if (error < -params_.speed_deadband_mps || target_acceleration_mps2 < 0.0) {
    command.brake = std::clamp(
      params_.brake_kp * std::max(0.0, -error) +
      params_.brake_deceleration_feedforward * std::max(0.0, -target_acceleration_mps2),
      0.0, params_.max_brake_command);
  }
  return command;
}

}  // namespace jetpilot_controller
