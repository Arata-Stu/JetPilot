#ifndef JETPILOT_CONTROLLER__CONTROLLER_TUNING_HPP_
#define JETPILOT_CONTROLLER__CONTROLLER_TUNING_HPP_
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include "jetpilot_controller/pure_pursuit.hpp"
#include "jetpilot_controller/map_pursuit.hpp"
#include "jetpilot_controller/kinematic_mpc.hpp"
#include "jetpilot_controller/longitudinal_controller.hpp"

namespace jetpilot_controller {
struct ControllerTuning {
  std::string algorithm;
  PurePursuitParams pure;
  MapPursuitParams map;
  KinematicMpcParams mpc;
  LongitudinalParams longitudinal;
  double max_speed, steering_rate;
  bool lateral_changed{false}, longitudinal_changed{false};

  void set_algorithm(const std::string & value) {
    if (value != "pure_pursuit" && value != "map_pursuit" && value != "kinematic_mpc")
      throw std::invalid_argument("unsupported controller algorithm");
    algorithm = value;
    lateral_changed = true;
  }
  void set(const std::string & name, double value) {
    if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument("value must be finite and non-negative");
    if (name == "max_target_speed_mps") {max_speed = value; return;}
    if (name == "max_steering_rate_per_s") {steering_rate = value; return;}
    if (name == "throttle_kp") longitudinal.throttle_kp = value;
    else if (name == "throttle_ki") longitudinal.throttle_ki = value;
    else if (name == "throttle_kd") longitudinal.throttle_kd = value;
    else if (name == "throttle_feedforward") longitudinal.throttle_feedforward = value;
    else if (name == "brake_kp") longitudinal.brake_kp = value;
    else if (name == "max_throttle_command") longitudinal.max_throttle_command = value;
    else if (name == "max_brake_command") longitudinal.max_brake_command = value;
    else {
      if (name == "min_lookahead_m") pure.min_lookahead_m = map.min_lookahead_m = value;
      else if (name == "max_lookahead_m") pure.max_lookahead_m = map.max_lookahead_m = value;
      else if (name == "lookahead_speed_gain_s") pure.lookahead_speed_gain_s = map.lookahead_speed_gain_s = value;
      else if (name == "max_steering_angle_rad") pure.max_steering_angle_rad = map.max_steering_angle_rad = mpc.max_steering_angle_rad = value;
      else if (name == "max_steering_command") pure.max_steering_command = map.max_steering_command = mpc.max_steering_command = value;
      else if (name == "map_lateral_error_gain") map.lateral_error_gain = value;
      else if (name == "mpc_path_error_weight") mpc.path_error_weight = value;
      else if (name == "mpc_heading_error_weight") mpc.heading_error_weight = value;
      else if (name == "mpc_steering_weight") mpc.steering_weight = value;
      else throw std::invalid_argument("parameter is not dynamically adjustable");
      lateral_changed = true;
      return;
    }
    longitudinal_changed = true;
  }
  void validate() const {
    if (!std::isfinite(max_speed) || max_speed <= 0 || !std::isfinite(steering_rate) || steering_rate < 0)
      throw std::invalid_argument("invalid speed/steering rate limit");
    PurePursuit pp(pure); MapPursuit mp(map); KinematicMpc km(mpc);
    LongitudinalController lc(longitudinal);
  }
  std::unique_ptr<PathTrackingController> make_lateral() const {
    if (algorithm == "pure_pursuit") return std::make_unique<PurePursuit>(pure);
    if (algorithm == "map_pursuit" || algorithm == "map") return std::make_unique<MapPursuit>(map);
    if (algorithm == "kinematic_mpc" || algorithm == "mpc") return std::make_unique<KinematicMpc>(mpc);
    throw std::invalid_argument("unsupported controller algorithm");
  }
};
}  // namespace jetpilot_controller
#endif
