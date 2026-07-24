#ifndef JETPILOT_CONTROLLER__PATH_TRACKING_CONTROLLER_HPP_
#define JETPILOT_CONTROLLER__PATH_TRACKING_CONTROLLER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace jetpilot_controller
{

enum class PathClosureMode
{
  kAuto,
  kOpen,
  kClosed,
};

struct Point2d
{
  double x{0.0};
  double y{0.0};
};

struct TrackingInput
{
  // The path must already be expressed in the vehicle base frame.
  std::vector<Point2d> path;
  double speed_mps{0.0};
};

struct TrackingResult
{
  bool valid{false};
  std::string reason;
  double steering_command{0.0};
  double steering_angle_rad{0.0};
  double curvature{0.0};
  double lookahead_distance_m{0.0};
  Point2d target_point;
  std::size_t nearest_index{0};
  std::size_t target_index{0};
  bool path_closed{false};
};

class PathTrackingController
{
public:
  virtual ~PathTrackingController() = default;
  virtual TrackingResult compute(const TrackingInput & input) const = 0;
};

}  // namespace jetpilot_controller

#endif  // JETPILOT_CONTROLLER__PATH_TRACKING_CONTROLLER_HPP_
