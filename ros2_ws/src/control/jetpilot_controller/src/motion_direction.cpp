#include "jetpilot_controller/motion_direction.hpp"

namespace jetpilot_controller
{
void orient_path_for_motion(std::vector<Point2d> & path, bool reverse_motion)
{
  if (!reverse_motion) return;
  for (auto & point : path)
  {
    point.x = -point.x;
    point.y = -point.y;
  }
}

double steering_for_motion(double motion_frame_steering, bool reverse_motion)
{
  return reverse_motion ? -motion_frame_steering : motion_frame_steering;
}

Point2d point_from_motion_frame(Point2d point, bool reverse_motion)
{
  if (reverse_motion)
  {
    point.x = -point.x;
    point.y = -point.y;
  }
  return point;
}
}  // namespace jetpilot_controller
