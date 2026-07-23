#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jetpilot_msgs/msg/control_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "jetpilot_controller/longitudinal_controller.hpp"
#include "jetpilot_controller/path_tracking_controller.hpp"
#include "jetpilot_controller/pure_pursuit.hpp"

namespace jetpilot_controller
{
namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;

double seconds_since(const SteadyTime & timestamp)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - timestamp).count();
}

diagnostic_msgs::msg::KeyValue diagnostic_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = std::move(key);
  output.value = std::move(value);
  return output;
}

}  // namespace

class PathTrackingControllerNode : public rclcpp::Node
{
public:
  PathTrackingControllerNode()
  : Node("path_tracking_controller_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_and_read_parameters();
    create_controller();
    create_interfaces();

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { control_cycle(); });

    RCLCPP_INFO(
      get_logger(),
      "Controller ready: algorithm=%s, trajectory=%s, target_speed=%s, odometry=%s, output=%s",
      algorithm_.c_str(), trajectory_topic_.c_str(), target_speed_topic_.c_str(),
      odometry_topic_.c_str(), command_topic_.c_str());
  }

private:
  void declare_and_read_parameters()
  {
    algorithm_ = declare_parameter<std::string>("algorithm", "pure_pursuit");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    trajectory_topic_ =
      declare_parameter<std::string>("trajectory_topic", "/planning/trajectory");
    target_speed_topic_ =
      declare_parameter<std::string>("target_speed_topic", "/planning/target_speed");
    planning_ready_topic_ =
      declare_parameter<std::string>("planning_ready_topic", "/planning/ready");
    localization_state_topic_ = declare_parameter<std::string>(
      "localization_state_topic", "/localization/pose_hint_state");
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/visual_slam/tracking/odometry");
    command_topic_ = declare_parameter<std::string>("command_topic", "/auto/control_cmd");
    diagnostics_topic_ =
      declare_parameter<std::string>("diagnostics_topic", "/controller/diagnostics");

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 30.0);
    trajectory_timeout_s_ = declare_parameter<double>("trajectory_timeout_s", 0.5);
    odometry_timeout_s_ = declare_parameter<double>("odometry_timeout_s", 0.3);
    target_speed_timeout_s_ = declare_parameter<double>("target_speed_timeout_s", 0.5);
    transform_timeout_s_ = declare_parameter<double>("transform_timeout_s", 0.05);
    transform_max_age_s_ = declare_parameter<double>("transform_max_age_s", 0.5);
    require_planning_ready_ = declare_parameter<bool>("require_planning_ready", true);
    require_localization_state_ = declare_parameter<bool>("require_localization_state", true);
    localization_state_timeout_s_ = declare_parameter<double>(
      "localization_state_timeout_s", 1.5);
    require_target_speed_ = declare_parameter<bool>("require_target_speed", true);
    fallback_target_speed_mps_ = declare_parameter<double>("fallback_target_speed_mps", 0.5);
    max_target_speed_mps_ = declare_parameter<double>("max_target_speed_mps", 3.0);
    goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.25);
    stop_at_path_end_ = declare_parameter<bool>("stop_at_path_end", true);
    safety_brake_command_ = declare_parameter<double>("safety_brake_command", 0.0);
    max_steering_rate_per_s_ = declare_parameter<double>("max_steering_rate_per_s", 4.0);
    max_lateral_accel_mps2_ = declare_parameter<double>("max_lateral_accel_mps2", 2.0);

    pure_pursuit_params_.wheelbase_m = declare_parameter<double>("wheelbase_m", 0.26);
    pure_pursuit_params_.min_lookahead_m =
      declare_parameter<double>("min_lookahead_m", 0.5);
    pure_pursuit_params_.max_lookahead_m =
      declare_parameter<double>("max_lookahead_m", 2.0);
    pure_pursuit_params_.lookahead_speed_gain_s =
      declare_parameter<double>("lookahead_speed_gain_s", 0.4);
    pure_pursuit_params_.max_steering_angle_rad =
      declare_parameter<double>("max_steering_angle_rad", 0.45);
    pure_pursuit_params_.max_steering_command =
      declare_parameter<double>("max_steering_command", 1.0);
    pure_pursuit_params_.closed_path_tolerance_m =
      declare_parameter<double>("closed_path_tolerance_m", 0.3);
    const auto path_closure_mode =
      declare_parameter<std::string>("path_closure_mode", "auto");
    if (path_closure_mode == "auto") {
      pure_pursuit_params_.path_closure_mode = PathClosureMode::kAuto;
    } else if (path_closure_mode == "open") {
      pure_pursuit_params_.path_closure_mode = PathClosureMode::kOpen;
    } else if (path_closure_mode == "closed") {
      pure_pursuit_params_.path_closure_mode = PathClosureMode::kClosed;
    } else {
      throw std::invalid_argument("path_closure_mode must be auto, open, or closed");
    }

    LongitudinalParams longitudinal_params;
    longitudinal_params.throttle_kp = declare_parameter<double>("throttle_kp", 0.5);
    longitudinal_params.throttle_feedforward =
      declare_parameter<double>("throttle_feedforward", 0.05);
    longitudinal_params.brake_kp = declare_parameter<double>("brake_kp", 0.5);
    longitudinal_params.speed_deadband_mps =
      declare_parameter<double>("speed_deadband_mps", 0.05);
    longitudinal_params.max_throttle_command =
      declare_parameter<double>("max_throttle_command", 0.35);
    longitudinal_params.max_brake_command =
      declare_parameter<double>("max_brake_command", 0.3);
    longitudinal_controller_ = std::make_unique<LongitudinalController>(longitudinal_params);

    for (const double value : {
        control_rate_hz_, trajectory_timeout_s_, odometry_timeout_s_,
        target_speed_timeout_s_, transform_timeout_s_, transform_max_age_s_,
        localization_state_timeout_s_, fallback_target_speed_mps_,
        max_target_speed_mps_, goal_tolerance_m_, safety_brake_command_,
        max_steering_rate_per_s_, max_lateral_accel_mps2_})
    {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("controller numeric parameters must be finite");
      }
    }

    if (base_frame_.empty()) {
      throw std::invalid_argument("base_frame must not be empty");
    }
    if (!(control_rate_hz_ > 0.0)) {
      throw std::invalid_argument("control_rate_hz must be > 0");
    }
    if (
      !(trajectory_timeout_s_ > 0.0) || !(odometry_timeout_s_ > 0.0) ||
      !(target_speed_timeout_s_ > 0.0) || transform_timeout_s_ < 0.0 ||
      transform_max_age_s_ < 0.0 || !(localization_state_timeout_s_ > 0.0))
    {
      throw std::invalid_argument(
              "input timeouts must be positive and transform limits non-negative");
    }
    if (fallback_target_speed_mps_ < 0.0 || !(max_target_speed_mps_ > 0.0)) {
      throw std::invalid_argument("speed limits must be non-negative with max_target_speed_mps > 0");
    }
    if (goal_tolerance_m_ < 0.0 || max_steering_rate_per_s_ < 0.0) {
      throw std::invalid_argument("goal tolerance and steering rate limit must be >= 0");
    }
    if (safety_brake_command_ < 0.0 || safety_brake_command_ > 1.0) {
      throw std::invalid_argument("safety_brake_command must be in [0, 1]");
    }
    if (max_lateral_accel_mps2_ < 0.0) {
      throw std::invalid_argument("max_lateral_accel_mps2 must be >= 0");
    }
  }

  void create_controller()
  {
    if (algorithm_ == "pure_pursuit") {
      lateral_controller_ = std::make_unique<PurePursuit>(pure_pursuit_params_);
      return;
    }
    throw std::invalid_argument(
            "Unsupported controller algorithm '" + algorithm_ +
            "'. Available algorithms: pure_pursuit");
  }

  void create_interfaces()
  {
    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
      trajectory_topic_, latched_qos,
      [this](const nav_msgs::msg::Path::SharedPtr message) {
        trajectory_ = *message;
        trajectory_received_at_ = std::chrono::steady_clock::now();
      });
    target_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      target_speed_topic_, latched_qos,
      [this](const std_msgs::msg::Float32::SharedPtr message) {
        target_speed_mps_ = static_cast<double>(message->data);
        target_speed_received_at_ = std::chrono::steady_clock::now();
      });
    planning_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      planning_ready_topic_, latched_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        planning_ready_ = message->data;
        planning_ready_received_ = true;
      });
    localization_state_sub_ = create_subscription<std_msgs::msg::String>(
      localization_state_topic_, latched_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {
        localization_confirmed_ =
          message->data.find("\"state\":\"localized\"") != std::string::npos;
        localization_state_received_at_ = std::chrono::steady_clock::now();
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS().keep_last(5),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        odometry_ = *message;
        odometry_received_at_ = std::chrono::steady_clock::now();
      });

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    command_pub_ = create_publisher<jetpilot_msgs::msg::ControlCommand>(
      command_topic_, command_qos);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/controller/ready", rclcpp::QoS(1).transient_local().reliable());
    lookahead_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/controller/lookahead_point", command_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);
  }

  bool transform_trajectory(
    const nav_msgs::msg::Path & trajectory, std::vector<Point2d> & points,
    std::string & reason)
  {
    std::string source_frame = trajectory.header.frame_id;
    if (source_frame.empty()) {
      for (const auto & pose : trajectory.poses) {
        if (!pose.header.frame_id.empty()) {
          source_frame = pose.header.frame_id;
          break;
        }
      }
    }
    if (source_frame.empty()) {
      reason = "trajectory frame_id is empty";
      return false;
    }
    for (const auto & pose : trajectory.poses) {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != source_frame) {
        reason = "trajectory contains mixed frame_ids";
        return false;
      }
    }

    tf2::Transform base_from_path;
    base_from_path.setIdentity();
    if (source_frame != base_frame_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          base_frame_, source_frame, tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_s_));
        if (transform_max_age_s_ > 0.0) {
          const rclcpp::Time transform_stamp(transform.header.stamp, get_clock()->get_clock_type());
          if (transform_stamp.nanoseconds() <= 0) {
            reason = "trajectory TF has no usable timestamp";
            return false;
          }
          const double transform_age = (now() - transform_stamp).seconds();
          if (!std::isfinite(transform_age) || transform_age < -transform_max_age_s_ ||
            transform_age > transform_max_age_s_)
          {
            reason = "trajectory TF is stale or from a different clock domain";
            return false;
          }
        }
        const auto & rotation = transform.transform.rotation;
        const auto & translation = transform.transform.translation;
        base_from_path = tf2::Transform(
          tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
          tf2::Vector3(translation.x, translation.y, translation.z));
      } catch (const tf2::TransformException & error) {
        reason = "TF " + source_frame + " -> " + base_frame_ + " unavailable: " + error.what();
        return false;
      }
    }

    points.clear();
    points.reserve(trajectory.poses.size());
    for (const auto & pose : trajectory.poses) {
      const auto & position = pose.pose.position;
      if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
        reason = "trajectory contains a non-finite position";
        return false;
      }
      const auto transformed = base_from_path * tf2::Vector3(position.x, position.y, position.z);
      points.push_back({transformed.x(), transformed.y()});
    }
    return true;
  }

  std::optional<double> current_speed(std::string & reason) const
  {
    if (!odometry_ || !odometry_received_at_) {
      reason = "waiting for odometry";
      return std::nullopt;
    }
    if (seconds_since(*odometry_received_at_) > odometry_timeout_s_) {
      reason = "odometry is stale";
      return std::nullopt;
    }
    const auto & linear = odometry_->twist.twist.linear;
    const double speed = std::hypot(linear.x, linear.y);
    if (!std::isfinite(speed)) {
      reason = "odometry speed is not finite";
      return std::nullopt;
    }
    return speed;
  }

  std::optional<double> requested_target_speed(std::string & reason) const
  {
    if (!target_speed_received_at_) {
      if (require_target_speed_) {
        reason = "waiting for planning target speed";
        return std::nullopt;
      }
      return std::clamp(fallback_target_speed_mps_, 0.0, max_target_speed_mps_);
    }
    if (seconds_since(*target_speed_received_at_) > target_speed_timeout_s_) {
      if (require_target_speed_) {
        reason = "planning target speed is stale";
        return std::nullopt;
      }
      return std::clamp(fallback_target_speed_mps_, 0.0, max_target_speed_mps_);
    }
    if (!std::isfinite(target_speed_mps_) || target_speed_mps_ < 0.0) {
      reason = "planning target speed is invalid";
      return std::nullopt;
    }
    return std::clamp(target_speed_mps_, 0.0, max_target_speed_mps_);
  }

  double apply_steering_rate_limit(double requested)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!last_control_at_ || max_steering_rate_per_s_ <= 0.0) {
      last_control_at_ = now;
      previous_steering_command_ = requested;
      return requested;
    }
    const double elapsed = std::chrono::duration<double>(now - *last_control_at_).count();
    last_control_at_ = now;
    const double maximum_delta = max_steering_rate_per_s_ * std::max(0.0, elapsed);
    previous_steering_command_ = std::clamp(
      requested,
      previous_steering_command_ - maximum_delta,
      previous_steering_command_ + maximum_delta);
    return previous_steering_command_;
  }

  void control_cycle()
  {
    std::string reason;
    if (require_localization_state_) {
      if (!localization_state_received_at_) {
        publish_safety_stop("waiting for confirmed localization state");
        return;
      }
      if (seconds_since(*localization_state_received_at_) > localization_state_timeout_s_) {
        publish_safety_stop("localization state is stale");
        return;
      }
      if (!localization_confirmed_) {
        publish_safety_stop("localization is not confirmed");
        return;
      }
    }
    if (require_planning_ready_ && (!planning_ready_received_ || !planning_ready_)) {
      publish_safety_stop("planning is not ready");
      return;
    }
    if (!trajectory_ || !trajectory_received_at_) {
      publish_safety_stop("waiting for trajectory");
      return;
    }
    if (seconds_since(*trajectory_received_at_) > trajectory_timeout_s_) {
      publish_safety_stop("trajectory is stale");
      return;
    }
    if (trajectory_->poses.size() < 2U) {
      publish_safety_stop("trajectory is empty or too short");
      return;
    }

    const auto speed = current_speed(reason);
    if (!speed) {
      publish_safety_stop(reason);
      return;
    }
    const auto target_speed = requested_target_speed(reason);
    if (!target_speed) {
      publish_safety_stop(reason);
      return;
    }

    TrackingInput input;
    input.speed_mps = *speed;
    if (!transform_trajectory(*trajectory_, input.path, reason)) {
      publish_safety_stop(reason);
      return;
    }
    const auto tracking = lateral_controller_->compute(input);
    if (!tracking.valid) {
      publish_safety_stop(tracking.reason);
      return;
    }

    const double goal_distance = std::hypot(input.path.back().x, input.path.back().y);
    if (
      stop_at_path_end_ && !tracking.path_closed && goal_distance <= goal_tolerance_m_)
    {
      publish_safety_stop("goal reached");
      return;
    }

    double limited_target_speed = *target_speed;
    if (max_lateral_accel_mps2_ > 0.0 && std::abs(tracking.curvature) > 1.0e-6) {
      const double curve_speed = std::sqrt(max_lateral_accel_mps2_ / std::abs(tracking.curvature));
      limited_target_speed = std::min(limited_target_speed, curve_speed);
    }
    const auto longitudinal = longitudinal_controller_->compute(limited_target_speed, *speed);

    jetpilot_msgs::msg::ControlCommand command;
    command.header.stamp = now();
    command.header.frame_id = base_frame_;
    command.steering = static_cast<float>(apply_steering_rate_limit(tracking.steering_command));
    command.throttle = static_cast<float>(longitudinal.throttle);
    command.brake = static_cast<float>(longitudinal.brake);
    command.reverse = 0.0F;
    command_pub_->publish(command);

    geometry_msgs::msg::PoseStamped lookahead;
    lookahead.header = command.header;
    lookahead.pose.position.x = tracking.target_point.x;
    lookahead.pose.position.y = tracking.target_point.y;
    lookahead.pose.orientation.w = 1.0;
    lookahead_pub_->publish(lookahead);

    publish_state(
      true, "tracking", *speed, limited_target_speed,
      tracking.steering_command, tracking.curvature);
  }

  void publish_safety_stop(const std::string & reason)
  {
    jetpilot_msgs::msg::ControlCommand command;
    command.header.stamp = now();
    command.header.frame_id = base_frame_;
    command.steering = 0.0F;
    command.throttle = 0.0F;
    command.brake = static_cast<float>(safety_brake_command_);
    command.reverse = 0.0F;
    command_pub_->publish(command);
    previous_steering_command_ = 0.0;
    last_control_at_ = std::chrono::steady_clock::now();

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Controller safety stop: %s", reason.c_str());
    publish_state(false, reason, 0.0, 0.0, 0.0, 0.0);
  }

  void publish_state(
    bool ready, const std::string & message, double current_speed_mps,
    double target_speed_mps, double steering_command, double curvature)
  {
    std_msgs::msg::Bool ready_message;
    ready_message.data = ready;
    ready_pub_->publish(ready_message);

    const auto steady_now = std::chrono::steady_clock::now();
    const bool state_changed = message != last_diagnostic_message_ || ready != last_ready_;
    if (
      !state_changed && last_diagnostic_at_ &&
      std::chrono::duration<double>(steady_now - *last_diagnostic_at_).count() < 1.0)
    {
      return;
    }
    last_diagnostic_at_ = steady_now;
    last_diagnostic_message_ = message;
    last_ready_ = ready;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "jetpilot_controller/path_tracking";
    status.hardware_id = "jetpilot_controller";
    status.level = ready ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = message;
    status.values.push_back(diagnostic_value("algorithm", algorithm_));
    status.values.push_back(diagnostic_value("current_speed_mps", std::to_string(current_speed_mps)));
    status.values.push_back(diagnostic_value("target_speed_mps", std::to_string(target_speed_mps)));
    status.values.push_back(diagnostic_value("steering_command", std::to_string(steering_command)));
    status.values.push_back(diagnostic_value("curvature", std::to_string(curvature)));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::string algorithm_;
  std::string base_frame_;
  std::string trajectory_topic_;
  std::string target_speed_topic_;
  std::string planning_ready_topic_;
  std::string localization_state_topic_;
  std::string odometry_topic_;
  std::string command_topic_;
  std::string diagnostics_topic_;

  double control_rate_hz_{30.0};
  double trajectory_timeout_s_{0.5};
  double odometry_timeout_s_{0.3};
  double target_speed_timeout_s_{0.5};
  double transform_timeout_s_{0.05};
  double transform_max_age_s_{0.5};
  bool require_planning_ready_{true};
  bool require_localization_state_{true};
  double localization_state_timeout_s_{1.5};
  bool require_target_speed_{true};
  double fallback_target_speed_mps_{0.5};
  double max_target_speed_mps_{3.0};
  double goal_tolerance_m_{0.25};
  bool stop_at_path_end_{true};
  double safety_brake_command_{0.0};
  double max_steering_rate_per_s_{4.0};
  double max_lateral_accel_mps2_{2.0};
  PurePursuitParams pure_pursuit_params_;

  std::unique_ptr<PathTrackingController> lateral_controller_;
  std::unique_ptr<LongitudinalController> longitudinal_controller_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::optional<nav_msgs::msg::Path> trajectory_;
  std::optional<nav_msgs::msg::Odometry> odometry_;
  std::optional<SteadyTime> trajectory_received_at_;
  std::optional<SteadyTime> odometry_received_at_;
  std::optional<SteadyTime> target_speed_received_at_;
  std::optional<SteadyTime> localization_state_received_at_;
  bool planning_ready_{false};
  bool planning_ready_received_{false};
  bool localization_confirmed_{false};
  double target_speed_mps_{0.0};
  double previous_steering_command_{0.0};
  std::optional<SteadyTime> last_control_at_;
  std::optional<SteadyTime> last_diagnostic_at_;
  std::string last_diagnostic_message_;
  bool last_ready_{false};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planning_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<jetpilot_msgs::msg::ControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace jetpilot_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<jetpilot_controller::PathTrackingControllerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("path_tracking_controller_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
