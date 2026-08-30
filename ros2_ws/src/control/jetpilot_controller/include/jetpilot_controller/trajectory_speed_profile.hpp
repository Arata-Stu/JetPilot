#ifndef JETPILOT_CONTROLLER__TRAJECTORY_SPEED_PROFILE_HPP_
#define JETPILOT_CONTROLLER__TRAJECTORY_SPEED_PROFILE_HPP_

#include <string>
#include <vector>

namespace jetpilot_controller
{

struct TrajectoryProfilePoint
{
  double station_m{0.0};
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
};

struct TrajectoryProfileSample
{
  bool valid{false};
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
  std::string reason;
};

// Interpolates v^2 at station_m under the CSV's segment-constant acceleration
// convention and conservatively returns the lowest requested speed encountered
// over lookahead_m. For a closed trajectory, loop_length_m includes the closing
// segment between the last and first points.
TrajectoryProfileSample sample_trajectory_profile(
  const std::vector<TrajectoryProfilePoint> & profile, double station_m, double lookahead_m,
  bool closed, double loop_length_m);

}  // namespace jetpilot_controller

#endif  // JETPILOT_CONTROLLER__TRAJECTORY_SPEED_PROFILE_HPP_
