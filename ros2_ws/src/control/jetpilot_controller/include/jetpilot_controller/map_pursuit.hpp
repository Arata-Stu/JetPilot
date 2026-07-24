#ifndef JETPILOT_CONTROLLER__MAP_PURSUIT_HPP_
#define JETPILOT_CONTROLLER__MAP_PURSUIT_HPP_

#include "jetpilot_controller/path_tracking_controller.hpp"

namespace jetpilot_controller
{

struct MapPursuitParams
{
  double wheelbase_m{0.26};
  double min_lookahead_m{0.5};
  double max_lookahead_m{2.0};
  double lookahead_speed_gain_s{0.4};
  double lateral_error_gain{0.4};
  double max_steering_angle_rad{0.45};
  double max_steering_command{1.0};
  double speed_steering_downscale_start_mps{1.5};
  double speed_steering_downscale_end_mps{3.0};
  double speed_steering_downscale_factor{0.25};
  double closed_path_tolerance_m{0.3};
  PathClosureMode path_closure_mode{PathClosureMode::kAuto};
};

class MapPursuit final : public PathTrackingController
{
public:
  explicit MapPursuit(MapPursuitParams params);

  TrackingResult compute(const TrackingInput & input) const override;
  const MapPursuitParams & params() const noexcept;

private:
  MapPursuitParams params_;
};

}  // namespace jetpilot_controller

#endif  // JETPILOT_CONTROLLER__MAP_PURSUIT_HPP_
