// Standalone regression: can run without ROS or gtest on a development host.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "jetpilot_controller/pure_pursuit.hpp"

using namespace jetpilot_controller;

int main()
{
  const double pi = std::acos(-1.0);
  std::vector<Point2d> authored;
  for (int i = 0; i < 16; ++i) {
    const double angle = 2.0 * pi * i / 16.0;
    authored.push_back({4.0 * std::cos(angle), 4.0 * std::sin(angle)});
  }
  PurePursuit controller(PurePursuitParams{});
  // The authored endpoint gap exceeds the legacy auto-closure tolerance.
  // Legacy HD-map Path now repeats the start; typed raceline/custom profiles
  // instead carry closed=true and need no duplicated station.
  for (const bool typed : {false, true}) {
    auto world_path = authored;
    if (!typed) world_path.push_back(authored.front());
    for (int step = 0; step < 600; ++step) {
      const double angle = 2.0 * pi * step / 200.0;
      const double yaw = angle + pi / 2.0;
      const Point2d ego{4.0 * std::cos(angle), 4.0 * std::sin(angle)};
      TrackingInput input;
      input.speed_mps = 1.0;
      if (typed) input.path_closed_override = true;
      for (const auto & point : world_path) {
        const double dx = point.x - ego.x, dy = point.y - ego.y;
        input.path.push_back({std::cos(yaw) * dx + std::sin(yaw) * dy,
                             -std::sin(yaw) * dx + std::cos(yaw) * dy});
      }
      const auto result = controller.compute(input);
      if (!result.valid || !result.path_closed || !std::isfinite(result.steering_command) ||
          std::hypot(result.target_point.x, result.target_point.y) < 0.1) {
        throw std::runtime_error("lost tracking or closed-loop semantics during repeated laps");
      }
    }
  }
  // Open trajectories must still be classified as open for goal stopping.
  TrackingInput open{{{0., 0.}, {1., 0.}, {2., 0.}}, 1.0};
  open.path_closed_override = false;
  if (controller.compute(open).path_closed) throw std::runtime_error("open path became closed");
  std::cout << "Repeated-lap tracking: 1200 samples passed\n";
}
