#ifndef JETPILOT_CONTROLLER__LONGITUDINAL_CONTROLLER_HPP_
#define JETPILOT_CONTROLLER__LONGITUDINAL_CONTROLLER_HPP_

namespace jetpilot_controller
{

struct LongitudinalParams
{
  double throttle_kp{0.5};
  double throttle_ki{0.0};
  double throttle_kd{0.0};
  double throttle_feedforward{0.05};
  double throttle_integral_error_limit{1.0};
  double brake_kp{0.5};
  double speed_deadband_mps{0.05};
  double brake_activation_error_mps{0.05};
  double minimum_moving_throttle_command{0.0};
  double max_throttle_command{0.35};
  double max_brake_command{0.3};
  bool active_braking_enabled{true};
  double throttle_acceleration_feedforward{0.0};
  double brake_deceleration_feedforward{0.0};
};

struct LongitudinalCommand
{
  double throttle{0.0};
  double brake{0.0};
};

class LongitudinalController
{
public:
  explicit LongitudinalController(LongitudinalParams params);

  LongitudinalCommand compute(
    double target_speed_mps, double current_speed_mps,
    double target_acceleration_mps2 = 0.0, double dt_s = 0.02);
  void reset() noexcept;
  const LongitudinalParams & params() const noexcept;

private:
  LongitudinalParams params_;
  double integral_error_{0.0};
  double previous_error_{0.0};
  bool previous_error_valid_{false};
};

}  // namespace jetpilot_controller

#endif  // JETPILOT_CONTROLLER__LONGITUDINAL_CONTROLLER_HPP_
