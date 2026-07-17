#ifndef JETPILOT_CONTROLLER__PURE_PURSUIT_HPP_
#define JETPILOT_CONTROLLER__PURE_PURSUIT_HPP_

#include "jetpilot_controller/path_tracking_controller.hpp"

namespace jetpilot_controller
{

enum class PathClosureMode
{
  kAuto,
  kOpen,
  kClosed,
};

struct PurePursuitParams
{
  double wheelbase_m{0.26};
  double min_lookahead_m{0.5};
  double max_lookahead_m{2.0};
  double lookahead_speed_gain_s{0.4};
  double max_steering_angle_rad{0.45};
  double max_steering_command{1.0};
  double closed_path_tolerance_m{0.3};
  PathClosureMode path_closure_mode{PathClosureMode::kAuto};
};

class PurePursuit final : public PathTrackingController
{
public:
  explicit PurePursuit(PurePursuitParams params);

  TrackingResult compute(const TrackingInput & input) const override;
  const PurePursuitParams & params() const noexcept;

private:
  PurePursuitParams params_;
};

}  // namespace jetpilot_controller

#endif  // JETPILOT_CONTROLLER__PURE_PURSUIT_HPP_
