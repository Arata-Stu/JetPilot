#ifndef JETPILOT_CONTROLLER__KINEMATIC_MPC_HPP_
#define JETPILOT_CONTROLLER__KINEMATIC_MPC_HPP_

#include <cstddef>

#include "jetpilot_controller/path_tracking_controller.hpp"

namespace jetpilot_controller
{

struct KinematicMpcParams
{
  double wheelbase_m{0.26};
  double max_steering_angle_rad{0.45};
  double max_steering_command{1.0};
  std::size_t horizon_steps{12};
  double time_step_s{0.05};
  std::size_t steering_samples{15};
  double min_prediction_speed_mps{0.2};
  double path_error_weight{4.0};
  double heading_error_weight{0.8};
  double steering_weight{0.15};
  double terminal_path_error_weight{2.0};
  double closed_path_tolerance_m{0.3};
  PathClosureMode path_closure_mode{PathClosureMode::kAuto};
};

class KinematicMpc final : public PathTrackingController
{
public:
  explicit KinematicMpc(KinematicMpcParams params);

  TrackingResult compute(const TrackingInput & input) const override;
  const KinematicMpcParams & params() const noexcept;

private:
  KinematicMpcParams params_;
};

}  // namespace jetpilot_controller

#endif  // JETPILOT_CONTROLLER__KINEMATIC_MPC_HPP_
