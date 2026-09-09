// Standard-library-only checks, runnable without ROS/gtest.
#include <cassert>
#include <limits>
#include "jetpilot_controller/controller_tuning.hpp"
using namespace jetpilot_controller;
int main() {
  ControllerTuning initial{"pure_pursuit", {}, {}, {}, {}, 3.0, 4.0};
  for (const auto & algorithm : {"pure_pursuit", "map_pursuit", "kinematic_mpc"}) {
    auto candidate = initial;
    candidate.set_algorithm(algorithm);
    candidate.validate();
    auto controller = candidate.make_lateral();
    TrackingInput input{};
    input.path = {{0,0},{0.5,0},{1,0},{2,0}};
    input.speed_mps = 1.0;
    const auto output = controller->compute(input);
    assert(output.valid && std::isfinite(output.steering_command));
    assert(candidate.lateral_changed);
  }
  auto updated = initial;
  updated.set("min_lookahead_m", 0.8);
  assert(updated.pure.min_lookahead_m == 0.8 && updated.map.min_lookahead_m == 0.8);
  updated.set("throttle_kp", 0.3);
  assert(updated.longitudinal_changed && updated.longitudinal.throttle_kp == 0.3);
  updated.validate();
  assert(initial.longitudinal.throttle_kp != updated.longitudinal.throttle_kp);
  bool rejected = false;
  try { updated.set("min_lookahead_m", 3.0); updated.validate(); }
  catch (const std::invalid_argument &) {rejected = true;}
  assert(rejected && initial.pure.min_lookahead_m == 0.5);
  rejected = false;
  try {initial.set("throttle_kp", std::numeric_limits<double>::quiet_NaN());}
  catch(const std::invalid_argument &) {rejected = true;}
  assert(rejected);
  rejected = false;
  try {initial.set_algorithm("unknown");}
  catch(const std::invalid_argument &) {rejected = true;}
  assert(rejected && initial.algorithm == "pure_pursuit");
}
