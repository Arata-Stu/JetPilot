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
    !std::isfinite(params_.throttle_ki) ||
    !std::isfinite(params_.throttle_kd) ||
    !std::isfinite(params_.throttle_feedforward) ||
    !std::isfinite(params_.throttle_integral_error_limit) ||
    !std::isfinite(params_.throttle_acceleration_feedforward) ||
    !std::isfinite(params_.brake_kp) ||
    !std::isfinite(params_.brake_deceleration_feedforward) ||
    !std::isfinite(params_.speed_deadband_mps) ||
    !std::isfinite(params_.brake_activation_error_mps) ||
    !std::isfinite(params_.minimum_moving_throttle_command) ||
    !std::isfinite(params_.max_throttle_command) ||
    !std::isfinite(params_.max_brake_command))
  {
    throw std::invalid_argument("longitudinal parameters must be finite");
  }
  if (
    params_.throttle_kp < 0.0 || params_.throttle_ki < 0.0 || params_.throttle_kd < 0.0 ||
    params_.throttle_feedforward < 0.0 || params_.throttle_integral_error_limit < 0.0 ||
    params_.throttle_acceleration_feedforward < 0.0 || params_.brake_kp < 0.0 ||
    params_.brake_deceleration_feedforward < 0.0 || params_.speed_deadband_mps < 0.0 ||
    params_.minimum_moving_throttle_command < 0.0 ||
    params_.brake_activation_error_mps < params_.speed_deadband_mps)
  {
    throw std::invalid_argument(
      "longitudinal gains and limits must be >= 0 and brake activation must cover deadband");
  }
  if (
    params_.minimum_moving_throttle_command > params_.max_throttle_command ||
    params_.max_throttle_command < 0.0 || params_.max_throttle_command > 1.0 ||
    params_.max_brake_command < 0.0 || params_.max_brake_command > 1.0)
  {
    throw std::invalid_argument("longitudinal command limits must be in [0, 1]");
  }
  if (params_.throttle_feedforward_speeds_mps.size() !=
    params_.throttle_feedforward_commands.size())
  {
    throw std::invalid_argument("throttle feedforward map arrays must have equal lengths");
  }
  for (std::size_t index = 0; index < params_.throttle_feedforward_speeds_mps.size(); ++index)
  {
    const double speed = params_.throttle_feedforward_speeds_mps[index];
    const double command = params_.throttle_feedforward_commands[index];
    if (!std::isfinite(speed) || speed < 0.0 || !std::isfinite(command) || command < 0.0 ||
      command > 1.0 || (index > 0 && speed <= params_.throttle_feedforward_speeds_mps[index - 1]))
    {
      throw std::invalid_argument(
        "throttle feedforward speeds must increase and commands must be in [0, 1]");
    }
  }
}

const LongitudinalParams & LongitudinalController::params() const noexcept
{
  return params_;
}

double LongitudinalController::feedforward_for_speed(const double target_speed_mps) const noexcept
{
  const auto & speeds = params_.throttle_feedforward_speeds_mps;
  const auto & commands = params_.throttle_feedforward_commands;
  if (speeds.empty())
  {
    return params_.throttle_feedforward;
  }
  if (target_speed_mps <= speeds.front())
  {
    return commands.front();
  }
  if (target_speed_mps >= speeds.back())
  {
    return commands.back();
  }
  const auto upper = std::upper_bound(speeds.begin(), speeds.end(), target_speed_mps);
  const std::size_t upper_index = static_cast<std::size_t>(upper - speeds.begin());
  const std::size_t lower_index = upper_index - 1U;
  const double ratio = (target_speed_mps - speeds[lower_index]) /
    (speeds[upper_index] - speeds[lower_index]);
  return commands[lower_index] + ratio * (commands[upper_index] - commands[lower_index]);
}

LongitudinalCommand LongitudinalController::compute(
  double target_speed_mps, double current_speed_mps, double target_acceleration_mps2,
  double dt_s)
{
  LongitudinalCommand command;
  if (!std::isfinite(target_speed_mps) || !std::isfinite(current_speed_mps) ||
    !std::isfinite(target_acceleration_mps2) || !std::isfinite(dt_s) || dt_s <= 0.0) {
    reset();
    return command;
  }

  const double target = std::max(0.0, target_speed_mps);
  const double speed = std::max(0.0, current_speed_mps);
  if (target <= 1.0e-6)
  {
    reset();
    return command;
  }

  const double error = target - speed;
  const double feedforward = feedforward_for_speed(target);
  if (params_.active_braking_enabled &&
    (error < -params_.brake_activation_error_mps || target_acceleration_mps2 < 0.0))
  {
    reset();
    command.brake = std::clamp(
      params_.brake_kp * std::max(0.0, -error) +
      params_.brake_deceleration_feedforward * std::max(0.0, -target_acceleration_mps2),
      0.0, params_.max_brake_command);
    return command;
  }

  const double controlled_error = std::abs(error) <= params_.speed_deadband_mps ? 0.0 : error;
  const double derivative = previous_error_valid_ ?
    (controlled_error - previous_error_) / dt_s : 0.0;
  const double candidate_integral = std::clamp(
    integral_error_ + controlled_error * dt_s,
    -params_.throttle_integral_error_limit, params_.throttle_integral_error_limit);
  const auto throttle_for_integral = [&](const double integral) {
      return feedforward + params_.throttle_kp * controlled_error +
             params_.throttle_ki * integral + params_.throttle_kd * derivative +
             params_.throttle_acceleration_feedforward *
             std::max(0.0, target_acceleration_mps2);
    };
  const double candidate_throttle = throttle_for_integral(candidate_integral);
  const bool saturating_high =
    candidate_throttle > params_.max_throttle_command && controlled_error > 0.0;
  const bool saturating_low =
    candidate_throttle < params_.minimum_moving_throttle_command && controlled_error < 0.0;
  if (!saturating_high && !saturating_low)
  {
    integral_error_ = candidate_integral;
  }
  previous_error_ = controlled_error;
  previous_error_valid_ = true;
  command.throttle = std::clamp(throttle_for_integral(integral_error_),
                                params_.minimum_moving_throttle_command,
                                params_.max_throttle_command);
  return command;
}

void LongitudinalController::reset() noexcept
{
  integral_error_ = 0.0;
  previous_error_ = 0.0;
  previous_error_valid_ = false;
}

}  // namespace jetpilot_controller
