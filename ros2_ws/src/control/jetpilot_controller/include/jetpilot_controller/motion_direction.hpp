#ifndef JETPILOT_CONTROLLER__MOTION_DIRECTION_HPP_
#define JETPILOT_CONTROLLER__MOTION_DIRECTION_HPP_

#include <vector>
#include "jetpilot_controller/path_tracking_controller.hpp"

namespace jetpilot_controller
{
void orient_path_for_motion(std::vector<Point2d> & path, bool reverse_motion);
double steering_for_motion(double motion_frame_steering, bool reverse_motion);
Point2d point_from_motion_frame(Point2d point, bool reverse_motion);
}  // namespace jetpilot_controller
#endif
