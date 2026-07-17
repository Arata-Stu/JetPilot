#ifndef JETPILOT_CONTROLLER__LONGITUDINAL_CONTROLLER_HPP_
#define JETPILOT_CONTROLLER__LONGITUDINAL_CONTROLLER_HPP_

namespace jetpilot_controller
{

struct LongitudinalParams
{
  double throttle_kp{0.5};
  double throttle_feedforward{0.05};
  double brake_kp{0.5};
  double speed_deadband_mps{0.05};
  double max_throttle_command{0.35};
  double max_brake_command{0.3};
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

  LongitudinalCommand compute(double target_speed_mps, double current_speed_mps) const;
  const LongitudinalParams & params() const noexcept;

private:
  LongitudinalParams params_;
};

}  // namespace jetpilot_controller

#endif  // JETPILOT_CONTROLLER__LONGITUDINAL_CONTROLLER_HPP_
