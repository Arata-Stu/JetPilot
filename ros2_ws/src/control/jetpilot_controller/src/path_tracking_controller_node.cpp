#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "jetpilot_controller/path_tracking_controller_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"

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

PathClosureMode parse_path_closure_mode(const std::string & mode)
{
  if (mode == "auto")
  {
    return PathClosureMode::kAuto;
  }
  if (mode == "open")
  {
    return PathClosureMode::kOpen;
  }
  if (mode == "closed")
  {
    return PathClosureMode::kClosed;
  }
  throw std::invalid_argument("path_closure_mode must be auto, open, or closed");
}

}  // namespace

PathTrackingControllerNode::PathTrackingControllerNode()
    : Node("path_tracking_controller_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  declare_and_read_parameters();
  create_controller();
  create_interfaces();

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                             [this]() { control_cycle(); });

  RCLCPP_INFO(get_logger(),
              "Controller ready: algorithm=%s, trajectory=%s, target_speed=%s, "
              "odometry=%s, output=%s",
              algorithm_.c_str(), trajectory_topic_.c_str(), target_speed_topic_.c_str(),
              odometry_topic_.c_str(), command_topic_.c_str());
}

void PathTrackingControllerNode::declare_and_read_parameters()
{
  algorithm_ = declare_parameter<std::string>("algorithm", "pure_pursuit");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  trajectory_topic_ = declare_parameter<std::string>("trajectory_topic", "/planning/trajectory");
  target_speed_topic_ =
    declare_parameter<std::string>("target_speed_topic", "/planning/target_speed");
  planning_ready_topic_ = declare_parameter<std::string>("planning_ready_topic", "/planning/ready");
  localization_state_topic_ =
    declare_parameter<std::string>("localization_state_topic", "/localization/pose_hint_state");
  odometry_topic_ =
    declare_parameter<std::string>("odometry_topic", "/visual_slam/tracking/odometry");
  opponent_odometry_topic_ =
    declare_parameter<std::string>("opponent_odometry_topic", "/perception/opponent/odometry");
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
  localization_state_timeout_s_ = declare_parameter<double>("localization_state_timeout_s", 1.5);
  require_target_speed_ = declare_parameter<bool>("require_target_speed", true);
  fallback_target_speed_mps_ = declare_parameter<double>("fallback_target_speed_mps", 0.5);
  max_target_speed_mps_ = declare_parameter<double>("max_target_speed_mps", 3.0);
  goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.25);
  stop_at_path_end_ = declare_parameter<bool>("stop_at_path_end", true);
  safety_brake_command_ = declare_parameter<double>("safety_brake_command", 0.0);
  max_steering_rate_per_s_ = declare_parameter<double>("max_steering_rate_per_s", 4.0);
  max_lateral_accel_mps2_ = declare_parameter<double>("max_lateral_accel_mps2", 2.0);
  opponent_odometry_timeout_s_ = declare_parameter<double>("opponent_odometry_timeout_s", 0.3);

  pure_pursuit_params_.wheelbase_m = declare_parameter<double>("wheelbase_m", 0.26);
  pure_pursuit_params_.min_lookahead_m = declare_parameter<double>("min_lookahead_m", 0.5);
  pure_pursuit_params_.max_lookahead_m = declare_parameter<double>("max_lookahead_m", 2.0);
  pure_pursuit_params_.lookahead_speed_gain_s =
    declare_parameter<double>("lookahead_speed_gain_s", 0.4);
  pure_pursuit_params_.max_steering_angle_rad =
    declare_parameter<double>("max_steering_angle_rad", 0.45);
  pure_pursuit_params_.max_steering_command =
    declare_parameter<double>("max_steering_command", 1.0);
  pure_pursuit_params_.closed_path_tolerance_m =
    declare_parameter<double>("closed_path_tolerance_m", 0.3);
  const auto path_closure_mode = declare_parameter<std::string>("path_closure_mode", "auto");
  pure_pursuit_params_.path_closure_mode = parse_path_closure_mode(path_closure_mode);

  map_pursuit_params_.wheelbase_m = pure_pursuit_params_.wheelbase_m;
  map_pursuit_params_.min_lookahead_m = pure_pursuit_params_.min_lookahead_m;
  map_pursuit_params_.max_lookahead_m = pure_pursuit_params_.max_lookahead_m;
  map_pursuit_params_.lookahead_speed_gain_s = pure_pursuit_params_.lookahead_speed_gain_s;
  map_pursuit_params_.max_steering_angle_rad = pure_pursuit_params_.max_steering_angle_rad;
  map_pursuit_params_.max_steering_command = pure_pursuit_params_.max_steering_command;
  map_pursuit_params_.closed_path_tolerance_m = pure_pursuit_params_.closed_path_tolerance_m;
  map_pursuit_params_.path_closure_mode = pure_pursuit_params_.path_closure_mode;
  map_pursuit_params_.lateral_error_gain = declare_parameter<double>("map_lateral_error_gain", 0.4);
  map_pursuit_params_.speed_steering_downscale_start_mps =
    declare_parameter<double>("map_speed_steering_downscale_start_mps", 1.5);
  map_pursuit_params_.speed_steering_downscale_end_mps =
    declare_parameter<double>("map_speed_steering_downscale_end_mps", 3.0);
  map_pursuit_params_.speed_steering_downscale_factor =
    declare_parameter<double>("map_speed_steering_downscale_factor", 0.25);

  kinematic_mpc_params_.wheelbase_m = pure_pursuit_params_.wheelbase_m;
  kinematic_mpc_params_.max_steering_angle_rad = pure_pursuit_params_.max_steering_angle_rad;
  kinematic_mpc_params_.max_steering_command = pure_pursuit_params_.max_steering_command;
  kinematic_mpc_params_.closed_path_tolerance_m = pure_pursuit_params_.closed_path_tolerance_m;
  kinematic_mpc_params_.path_closure_mode = pure_pursuit_params_.path_closure_mode;
  const auto mpc_horizon_steps =
    std::max<int64_t>(declare_parameter<int64_t>("mpc_horizon_steps", 12), 1);
  const auto mpc_steering_samples =
    std::max<int64_t>(declare_parameter<int64_t>("mpc_steering_samples", 15), 3);
  kinematic_mpc_params_.horizon_steps = static_cast<std::size_t>(mpc_horizon_steps);
  kinematic_mpc_params_.steering_samples = static_cast<std::size_t>(mpc_steering_samples);
  kinematic_mpc_params_.time_step_s = declare_parameter<double>("mpc_time_step_s", 0.05);
  kinematic_mpc_params_.min_prediction_speed_mps =
    declare_parameter<double>("mpc_min_prediction_speed_mps", 0.2);
  kinematic_mpc_params_.path_error_weight = declare_parameter<double>("mpc_path_error_weight", 4.0);
  kinematic_mpc_params_.heading_error_weight =
    declare_parameter<double>("mpc_heading_error_weight", 0.8);
  kinematic_mpc_params_.steering_weight = declare_parameter<double>("mpc_steering_weight", 0.15);
  kinematic_mpc_params_.terminal_path_error_weight =
    declare_parameter<double>("mpc_terminal_path_error_weight", 2.0);

  LongitudinalParams longitudinal_params;
  longitudinal_params.throttle_kp = declare_parameter<double>("throttle_kp", 0.5);
  longitudinal_params.throttle_feedforward =
    declare_parameter<double>("throttle_feedforward", 0.05);
  longitudinal_params.brake_kp = declare_parameter<double>("brake_kp", 0.5);
  longitudinal_params.speed_deadband_mps = declare_parameter<double>("speed_deadband_mps", 0.05);
  longitudinal_params.max_throttle_command =
    declare_parameter<double>("max_throttle_command", 0.35);
  longitudinal_params.max_brake_command = declare_parameter<double>("max_brake_command", 0.3);
  longitudinal_controller_ = std::make_unique<LongitudinalController>(longitudinal_params);

  TrailingParams trailing_params;
  trailing_params.enabled = declare_parameter<bool>("trailing_enabled", false);
  trailing_params.trailing_gap_m = declare_parameter<double>("trailing_gap_m", 1.5);
  trailing_params.kp = declare_parameter<double>("trailing_kp", 0.5);
  trailing_params.ki = declare_parameter<double>("trailing_ki", 0.001);
  trailing_params.kd = declare_parameter<double>("trailing_kd", 0.2);
  trailing_params.max_gap_m = declare_parameter<double>("trailing_max_gap_m", 8.0);
  trailing_params.min_command_speed_mps =
    declare_parameter<double>("trailing_min_command_speed_mps", 0.0);
  trailing_controller_ = std::make_unique<TrailingController>(trailing_params);

  for (const double value :
       {control_rate_hz_, trajectory_timeout_s_, odometry_timeout_s_, target_speed_timeout_s_,
        transform_timeout_s_, transform_max_age_s_, localization_state_timeout_s_,
        fallback_target_speed_mps_, max_target_speed_mps_, goal_tolerance_m_, safety_brake_command_,
        max_steering_rate_per_s_, max_lateral_accel_mps2_, opponent_odometry_timeout_s_})
  {
    if (!std::isfinite(value))
    {
      throw std::invalid_argument("controller numeric parameters must be finite");
    }
  }

  if (base_frame_.empty())
  {
    throw std::invalid_argument("base_frame must not be empty");
  }
  if (!(control_rate_hz_ > 0.0))
  {
    throw std::invalid_argument("control_rate_hz must be > 0");
  }
  if (!(trajectory_timeout_s_ > 0.0) || !(odometry_timeout_s_ > 0.0) ||
      !(target_speed_timeout_s_ > 0.0) || transform_timeout_s_ < 0.0 ||
      transform_max_age_s_ < 0.0 || !(localization_state_timeout_s_ > 0.0))
  {
    throw std::invalid_argument(
      "input timeouts must be positive and transform limits non-negative");
  }
  if (fallback_target_speed_mps_ < 0.0 || !(max_target_speed_mps_ > 0.0))
  {
    throw std::invalid_argument("speed limits must be non-negative with max_target_speed_mps > 0");
  }
  if (goal_tolerance_m_ < 0.0 || max_steering_rate_per_s_ < 0.0)
  {
    throw std::invalid_argument("goal tolerance and steering rate limit must be >= 0");
  }
  if (safety_brake_command_ < 0.0 || safety_brake_command_ > 1.0)
  {
    throw std::invalid_argument("safety_brake_command must be in [0, 1]");
  }
  if (max_lateral_accel_mps2_ < 0.0)
  {
    throw std::invalid_argument("max_lateral_accel_mps2 must be >= 0");
  }
  if (opponent_odometry_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument("opponent_odometry_timeout_s must be > 0");
  }
}

void PathTrackingControllerNode::create_controller()
{
  if (algorithm_ == "pure_pursuit")
  {
    lateral_controller_ = std::make_unique<PurePursuit>(pure_pursuit_params_);
    return;
  }
  if (algorithm_ == "map_pursuit" || algorithm_ == "map")
  {
    lateral_controller_ = std::make_unique<MapPursuit>(map_pursuit_params_);
    return;
  }
  if (algorithm_ == "kinematic_mpc" || algorithm_ == "mpc")
  {
    lateral_controller_ = std::make_unique<KinematicMpc>(kinematic_mpc_params_);
    return;
  }
  throw std::invalid_argument("Unsupported controller algorithm '" + algorithm_ +
                              "'. Available algorithms: pure_pursuit, map_pursuit, kinematic_mpc");
}

void PathTrackingControllerNode::create_interfaces()
{
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
    trajectory_topic_, latched_qos,
    [this](const nav_msgs::msg::Path::SharedPtr message)
    {
      trajectory_ = *message;
      trajectory_received_at_ = std::chrono::steady_clock::now();
    });
  target_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
    target_speed_topic_, latched_qos,
    [this](const std_msgs::msg::Float32::SharedPtr message)
    {
      target_speed_mps_ = static_cast<double>(message->data);
      target_speed_received_at_ = std::chrono::steady_clock::now();
    });
  planning_ready_sub_ =
    create_subscription<std_msgs::msg::Bool>(planning_ready_topic_, latched_qos,
                                             [this](const std_msgs::msg::Bool::SharedPtr message)
                                             {
                                               planning_ready_ = message->data;
                                               planning_ready_received_ = true;
                                             });
  localization_state_sub_ = create_subscription<std_msgs::msg::String>(
    localization_state_topic_, latched_qos,
    [this](const std_msgs::msg::String::SharedPtr message)
    {
      localization_confirmed_ = message->data.find("\"state\":\"localized\"") != std::string::npos;
      localization_state_received_at_ = std::chrono::steady_clock::now();
    });
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, rclcpp::SensorDataQoS().keep_last(5),
    [this](const nav_msgs::msg::Odometry::SharedPtr message)
    {
      odometry_ = *message;
      odometry_received_at_ = std::chrono::steady_clock::now();
    });
  opponent_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    opponent_odometry_topic_, rclcpp::SensorDataQoS().keep_last(5),
    [this](const nav_msgs::msg::Odometry::SharedPtr message)
    {
      opponent_odometry_ = *message;
      opponent_odometry_received_at_ = std::chrono::steady_clock::now();
    });

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  command_pub_ = create_publisher<jetpilot_msgs::msg::ControlCommand>(command_topic_, command_qos);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("/controller/ready",
                                                     rclcpp::QoS(1).transient_local().reliable());
  lookahead_pub_ =
    create_publisher<geometry_msgs::msg::PoseStamped>("/controller/lookahead_point", command_qos);
  diagnostics_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
}

bool PathTrackingControllerNode::transform_trajectory(const nav_msgs::msg::Path & trajectory,
                                                      std::vector<Point2d> & points,
                                                      std::string & reason)
{
  std::string source_frame = trajectory.header.frame_id;
  if (source_frame.empty())
  {
    for (const auto & pose : trajectory.poses)
    {
      if (!pose.header.frame_id.empty())
      {
        source_frame = pose.header.frame_id;
        break;
      }
    }
  }
  if (source_frame.empty())
  {
    reason = "trajectory frame_id is empty";
    return false;
  }
  for (const auto & pose : trajectory.poses)
  {
    if (!pose.header.frame_id.empty() && pose.header.frame_id != source_frame)
    {
      reason = "trajectory contains mixed frame_ids";
      return false;
    }
  }

  tf2::Transform base_from_path;
  base_from_path.setIdentity();
  if (source_frame != base_frame_)
  {
    try
    {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, source_frame, tf2::TimePointZero, tf2::durationFromSec(transform_timeout_s_));
      if (transform_max_age_s_ > 0.0)
      {
        const rclcpp::Time transform_stamp(transform.header.stamp, get_clock()->get_clock_type());
        if (transform_stamp.nanoseconds() <= 0)
        {
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
      base_from_path =
        tf2::Transform(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
                       tf2::Vector3(translation.x, translation.y, translation.z));
    }
    catch (const tf2::TransformException & error)
    {
      reason = "TF " + source_frame + " -> " + base_frame_ + " unavailable: " + error.what();
      return false;
    }
  }

  points.clear();
  points.reserve(trajectory.poses.size());
  for (const auto & pose : trajectory.poses)
  {
    const auto & position = pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z))
    {
      reason = "trajectory contains a non-finite position";
      return false;
    }
    const auto transformed = base_from_path * tf2::Vector3(position.x, position.y, position.z);
    points.push_back({transformed.x(), transformed.y()});
  }
  return true;
}

bool PathTrackingControllerNode::transform_point_to_base(const geometry_msgs::msg::Point & point,
                                                         const std::string & source_frame,
                                                         Point2d & transformed_point,
                                                         std::string & reason)
{
  if (source_frame.empty())
  {
    reason = "point frame_id is empty";
    return false;
  }
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
  {
    reason = "point contains a non-finite position";
    return false;
  }

  tf2::Transform base_from_source;
  base_from_source.setIdentity();
  if (source_frame != base_frame_)
  {
    try
    {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, source_frame, tf2::TimePointZero, tf2::durationFromSec(transform_timeout_s_));
      const auto & rotation = transform.transform.rotation;
      const auto & translation = transform.transform.translation;
      base_from_source =
        tf2::Transform(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
                       tf2::Vector3(translation.x, translation.y, translation.z));
    }
    catch (const tf2::TransformException & error)
    {
      reason = "TF " + source_frame + " -> " + base_frame_ + " unavailable: " + error.what();
      return false;
    }
  }

  const auto transformed = base_from_source * tf2::Vector3(point.x, point.y, point.z);
  transformed_point = {transformed.x(), transformed.y()};
  return true;
}

PathTrackingControllerNode::StationProjection PathTrackingControllerNode::project_station(
  const std::vector<Point2d> & path, const Point2d & point, bool path_closed) const
{
  StationProjection projection;
  if (path.size() < 2U)
  {
    return projection;
  }

  double station_at_segment_start = 0.0;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  const std::size_t segment_count = path_closed ? path.size() : path.size() - 1U;
  for (std::size_t index = 0; index < segment_count; ++index)
  {
    const auto & a = path[index];
    const auto & b = path[(index + 1U) % path.size()];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq <= 1.0e-12)
    {
      continue;
    }

    const double t =
      std::clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / length_sq, 0.0, 1.0);
    const double projected_x = a.x + t * dx;
    const double projected_y = a.y + t * dy;
    const double distance_sq = (point.x - projected_x) * (point.x - projected_x) +
                               (point.y - projected_y) * (point.y - projected_y);
    if (distance_sq < best_distance_sq)
    {
      best_distance_sq = distance_sq;
      projection.station_m = station_at_segment_start + t * std::sqrt(length_sq);
      projection.valid = true;
    }
    station_at_segment_start += std::sqrt(length_sq);
  }
  return projection;
}

double PathTrackingControllerNode::path_length(const std::vector<Point2d> & path,
                                               bool path_closed) const
{
  if (path.size() < 2U)
  {
    return 0.0;
  }
  double length = 0.0;
  const std::size_t segment_count = path_closed ? path.size() : path.size() - 1U;
  for (std::size_t index = 0; index < segment_count; ++index)
  {
    const auto & a = path[index];
    const auto & b = path[(index + 1U) % path.size()];
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}

std::optional<double> PathTrackingControllerNode::current_speed(std::string & reason) const
{
  if (!odometry_ || !odometry_received_at_)
  {
    reason = "waiting for odometry";
    return std::nullopt;
  }
  if (seconds_since(*odometry_received_at_) > odometry_timeout_s_)
  {
    reason = "odometry is stale";
    return std::nullopt;
  }
  const auto & linear = odometry_->twist.twist.linear;
  const double speed = std::hypot(linear.x, linear.y);
  if (!std::isfinite(speed))
  {
    reason = "odometry speed is not finite";
    return std::nullopt;
  }
  return speed;
}

std::optional<double> PathTrackingControllerNode::requested_target_speed(std::string & reason) const
{
  if (!target_speed_received_at_)
  {
    if (require_target_speed_)
    {
      reason = "waiting for planning target speed";
      return std::nullopt;
    }
    return std::clamp(fallback_target_speed_mps_, 0.0, max_target_speed_mps_);
  }
  if (seconds_since(*target_speed_received_at_) > target_speed_timeout_s_)
  {
    if (require_target_speed_)
    {
      reason = "planning target speed is stale";
      return std::nullopt;
    }
    return std::clamp(fallback_target_speed_mps_, 0.0, max_target_speed_mps_);
  }
  if (!std::isfinite(target_speed_mps_) || target_speed_mps_ < 0.0)
  {
    reason = "planning target speed is invalid";
    return std::nullopt;
  }
  return std::clamp(target_speed_mps_, 0.0, max_target_speed_mps_);
}

double PathTrackingControllerNode::apply_steering_rate_limit(double requested)
{
  const auto now = std::chrono::steady_clock::now();
  if (!last_control_at_ || max_steering_rate_per_s_ <= 0.0)
  {
    last_control_at_ = now;
    previous_steering_command_ = requested;
    return requested;
  }
  const double elapsed = std::chrono::duration<double>(now - *last_control_at_).count();
  last_control_at_ = now;
  const double maximum_delta = max_steering_rate_per_s_ * std::max(0.0, elapsed);
  previous_steering_command_ = std::clamp(requested, previous_steering_command_ - maximum_delta,
                                          previous_steering_command_ + maximum_delta);
  return previous_steering_command_;
}

void PathTrackingControllerNode::control_cycle()
{
  std::string reason;
  if (require_localization_state_)
  {
    if (!localization_state_received_at_)
    {
      publish_safety_stop("waiting for confirmed localization state");
      return;
    }
    if (seconds_since(*localization_state_received_at_) > localization_state_timeout_s_)
    {
      publish_safety_stop("localization state is stale");
      return;
    }
    if (!localization_confirmed_)
    {
      publish_safety_stop("localization is not confirmed");
      return;
    }
  }
  if (require_planning_ready_ && (!planning_ready_received_ || !planning_ready_))
  {
    publish_safety_stop("planning is not ready");
    return;
  }
  if (!trajectory_ || !trajectory_received_at_)
  {
    publish_safety_stop("waiting for trajectory");
    return;
  }
  if (seconds_since(*trajectory_received_at_) > trajectory_timeout_s_)
  {
    publish_safety_stop("trajectory is stale");
    return;
  }
  if (trajectory_->poses.size() < 2U)
  {
    publish_safety_stop("trajectory is empty or too short");
    return;
  }

  const auto speed = current_speed(reason);
  if (!speed)
  {
    publish_safety_stop(reason);
    return;
  }
  const auto target_speed = requested_target_speed(reason);
  if (!target_speed)
  {
    publish_safety_stop(reason);
    return;
  }

  TrackingInput input;
  input.speed_mps = *speed;
  if (!transform_trajectory(*trajectory_, input.path, reason))
  {
    publish_safety_stop(reason);
    return;
  }
  const auto tracking = lateral_controller_->compute(input);
  if (!tracking.valid)
  {
    publish_safety_stop(tracking.reason);
    return;
  }

  const double goal_distance = std::hypot(input.path.back().x, input.path.back().y);
  if (stop_at_path_end_ && !tracking.path_closed && goal_distance <= goal_tolerance_m_)
  {
    publish_safety_stop("goal reached");
    return;
  }

  double limited_target_speed = *target_speed;
  if (max_lateral_accel_mps2_ > 0.0 && std::abs(tracking.curvature) > 1.0e-6)
  {
    const double curve_speed = std::sqrt(max_lateral_accel_mps2_ / std::abs(tracking.curvature));
    limited_target_speed = std::min(limited_target_speed, curve_speed);
  }

  TrailingResult trailing;
  trailing.target_speed_mps = limited_target_speed;
  if (trailing_controller_->params().enabled)
  {
    trailing = apply_trailing_limit(limited_target_speed, *speed, input.path, tracking.path_closed);
    limited_target_speed = trailing.target_speed_mps;
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

  publish_state(true, "tracking", *speed, limited_target_speed, tracking.steering_command,
                tracking.curvature, trailing);
}

TrailingResult PathTrackingControllerNode::apply_trailing_limit(double planned_speed_mps,
                                                                double current_speed_mps,
                                                                const std::vector<Point2d> & path,
                                                                bool path_closed)
{
  TrailingResult result;
  result.target_speed_mps = planned_speed_mps;
  if (!opponent_odometry_ || !opponent_odometry_received_at_)
  {
    result.reason = "waiting for opponent odometry";
    trailing_controller_->reset();
    return result;
  }
  if (seconds_since(*opponent_odometry_received_at_) > opponent_odometry_timeout_s_)
  {
    result.reason = "opponent odometry is stale";
    trailing_controller_->reset();
    return result;
  }

  std::string reason;
  Point2d opponent_position;
  const std::string opponent_frame =
    opponent_odometry_->header.frame_id.empty() ? base_frame_ : opponent_odometry_->header.frame_id;
  if (!transform_point_to_base(opponent_odometry_->pose.pose.position, opponent_frame,
                               opponent_position, reason))
  {
    result.reason = reason;
    trailing_controller_->reset();
    return result;
  }

  const auto ego_projection = project_station(path, Point2d{0.0, 0.0}, path_closed);
  const auto opponent_projection = project_station(path, opponent_position, path_closed);
  if (!ego_projection.valid || !opponent_projection.valid)
  {
    result.reason = "could not project ego or opponent onto trajectory";
    trailing_controller_->reset();
    return result;
  }

  const auto & linear = opponent_odometry_->twist.twist.linear;
  const double opponent_speed = std::hypot(linear.x, linear.y);
  const auto now_steady = std::chrono::steady_clock::now();
  double dt = 0.0;
  if (last_trailing_update_at_)
  {
    dt = std::chrono::duration<double>(now_steady - *last_trailing_update_at_).count();
  }
  last_trailing_update_at_ = now_steady;

  TrailingInput input;
  input.planned_speed_mps = planned_speed_mps;
  input.ego_speed_mps = current_speed_mps;
  input.ego_station_m = ego_projection.station_m;
  input.opponent_station_m = opponent_projection.station_m;
  input.opponent_speed_mps = opponent_speed;
  input.track_length_m = path_length(path, path_closed);
  input.path_closed = path_closed;
  input.dt_s = dt;
  return trailing_controller_->compute(input);
}

void PathTrackingControllerNode::publish_safety_stop(const std::string & reason)
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

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Controller safety stop: %s",
                       reason.c_str());
  publish_state(false, reason, 0.0, 0.0, 0.0, 0.0, TrailingResult{});
}

void PathTrackingControllerNode::publish_state(bool ready, const std::string & message,
                                               double current_speed_mps, double target_speed_mps,
                                               double steering_command, double curvature,
                                               const TrailingResult & trailing)
{
  std_msgs::msg::Bool ready_message;
  ready_message.data = ready;
  ready_pub_->publish(ready_message);

  const auto steady_now = std::chrono::steady_clock::now();
  const bool state_changed = message != last_diagnostic_message_ || ready != last_ready_;
  if (!state_changed && last_diagnostic_at_ &&
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
  status.level = ready ? diagnostic_msgs::msg::DiagnosticStatus::OK
                       : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = message;
  status.values.push_back(diagnostic_value("algorithm", algorithm_));
  status.values.push_back(diagnostic_value("current_speed_mps", std::to_string(current_speed_mps)));
  status.values.push_back(diagnostic_value("target_speed_mps", std::to_string(target_speed_mps)));
  status.values.push_back(diagnostic_value("steering_command", std::to_string(steering_command)));
  status.values.push_back(diagnostic_value("curvature", std::to_string(curvature)));
  status.values.push_back(diagnostic_value(
    "trailing_enabled", trailing_controller_->params().enabled ? "true" : "false"));
  status.values.push_back(diagnostic_value("trailing_active", trailing.active ? "true" : "false"));
  status.values.push_back(diagnostic_value("trailing_gap_m", std::to_string(trailing.gap_m)));
  status.values.push_back(diagnostic_value("trailing_reason", trailing.reason));
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);
}

}  // namespace jetpilot_controller
