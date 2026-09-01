#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jetpilot_planning/raceline_path_publisher_node.hpp"

namespace jetpilot_planning
{

RacelinePathPublisherNode::RacelinePathPublisherNode() : Node("raceline_path_publisher")
{
  raceline_root_ = declare_parameter<std::string>("raceline_root", "");
  raceline_csv_ = declare_parameter<std::string>("raceline_csv", "");
  output_topic_ =
    declare_parameter<std::string>("output_topic", "/planning/raceline_path");
  trajectory_output_topic_ = declare_parameter<std::string>(
    "trajectory_output_topic", "/planning/raceline_trajectory");
  publish_typed_trajectory_ = declare_parameter<bool>("publish_typed_trajectory", true);
  line_id_ = declare_parameter<std::string>("line_id", "raceline");
  line_name_ = declare_parameter<std::string>("line_name", "");
  const auto startup_source_hash = declare_parameter<std::string>("source_hash", "");
  closed_ = declare_parameter<bool>("closed", true);
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  reload_interval_sec_ = declare_parameter<double>("reload_interval_sec", 0.5);
  const auto max_file_bytes = declare_parameter<int64_t>("max_file_bytes", 16 * 1024 * 1024);
  const auto max_points = declare_parameter<int64_t>("max_points", 200000);

  if (raceline_csv_.empty())
  {
    throw std::invalid_argument(
      "raceline_csv is required; keep "
      "enable_raceline_publisher=false when unused");
  }
  if (output_topic_.empty())
  {
    throw std::invalid_argument("output_topic must not be empty");
  }
  if (line_name_.empty())
  {
    line_name_ = line_id_;
  }
  if (publish_typed_trajectory_ &&
      (trajectory_output_topic_.empty() || line_id_.empty() || line_name_.empty()))
  {
    throw std::invalid_argument(
      "trajectory_output_topic, line_id and line_name must not be empty when typed publishing is enabled");
  }
  if (frame_id_.empty())
  {
    throw std::invalid_argument("frame_id must not be empty");
  }
  if (max_file_bytes <= 0 || max_points < 2 ||
      !std::isfinite(reload_interval_sec_) || reload_interval_sec_ <= 0.0)
  {
    throw std::invalid_argument("raceline CSV limits and reload_interval_sec must be positive");
  }

  limits_.max_file_bytes = static_cast<std::uintmax_t>(max_file_bytes);
  limits_.max_points = static_cast<std::size_t>(max_points);
  auto initial = load_stable_raceline_csv(raceline_root_, raceline_csv_, limits_);
  if (!startup_source_hash.empty() && startup_source_hash != initial.data.source_hash)
  {
    throw std::runtime_error(
      "source_hash does not match the selected trajectory CSV; refusing mixed geometry/speed");
  }
  raceline_ = std::move(initial.data);
  loaded_signature_ = initial.signature;
  source_available_ = true;

  const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  path_pub_ = create_publisher<nav_msgs::msg::Path>(output_topic_, path_qos);
  if (publish_typed_trajectory_)
  {
    trajectory_pub_ = create_publisher<jetpilot_msgs::msg::Trajectory>(
      trajectory_output_topic_, path_qos);
  }
  publish_outputs();
  reload_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(reload_interval_sec_)),
    [this]() { check_for_reload(); });

  RCLCPP_INFO(
    get_logger(),
    "Published %zu trajectory poses from '%s' on '%s' (typed='%s', line='%s', hash='%s')",
    raceline_.points.size(), raceline_.source_path.c_str(), output_topic_.c_str(),
    publish_typed_trajectory_ ? trajectory_output_topic_.c_str() : "disabled", line_id_.c_str(),
    raceline_.source_hash.c_str());
}

void RacelinePathPublisherNode::publish_outputs()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id_;
  path.header.stamp = now();
  jetpilot_msgs::msg::Trajectory trajectory;
  trajectory.header = path.header;
  trajectory.line_id = line_id_;
  trajectory.display_name = line_name_;
  trajectory.source_hash = raceline_.source_hash;
  trajectory.closed = closed_;
  trajectory.motion_direction = jetpilot_msgs::msg::Trajectory::MOTION_FORWARD;
  path.poses.reserve(raceline_.points.size());
  trajectory.points.reserve(raceline_.points.size());
  for (const auto & point : raceline_.points)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    const auto normalized_psi = std::remainder(point.psi, 2.0 * std::acos(-1.0));
    pose.pose.orientation.z = std::sin(normalized_psi * 0.5);
    pose.pose.orientation.w = std::cos(normalized_psi * 0.5);
    jetpilot_msgs::msg::TrajectoryPoint trajectory_point;
    trajectory_point.s_m = point.s;
    trajectory_point.pose = pose.pose;
    trajectory_point.curvature_radpm = point.kappa;
    trajectory_point.longitudinal_velocity_mps = point.vx;
    trajectory_point.longitudinal_acceleration_mps2 = point.ax;
    trajectory.points.push_back(std::move(trajectory_point));
    path.poses.push_back(std::move(pose));
  }
  path_pub_->publish(path);
  if (trajectory_pub_)
  {
    trajectory_pub_->publish(trajectory);
  }
}

void RacelinePathPublisherNode::publish_invalidation()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id_;
  path.header.stamp = now();
  path_pub_->publish(path);
  if (trajectory_pub_)
  {
    jetpilot_msgs::msg::Trajectory trajectory;
    trajectory.header = path.header;
    trajectory.line_id = line_id_;
    trajectory.display_name = line_name_;
    trajectory.closed = closed_;
    trajectory.motion_direction = jetpilot_msgs::msg::Trajectory::MOTION_FORWARD;
    trajectory_pub_->publish(trajectory);
  }
}

void RacelinePathPublisherNode::fail_closed(
  const std::string & issue_key, const std::string & message)
{
  if (source_available_)
  {
    source_available_ = false;
    publish_invalidation();
  }
  if (issue_key != last_reload_issue_key_)
  {
    last_reload_issue_key_ = issue_key;
    RCLCPP_ERROR(
      get_logger(), "Trajectory source became unavailable; published an empty invalidation: %s",
      message.c_str());
  }
}

void RacelinePathPublisherNode::log_reload_success(const bool content_changed)
{
  last_reload_issue_key_.clear();
  RCLCPP_INFO(
    get_logger(), "%s trajectory CSV '%s' (%zu poses, line='%s', hash='%s')",
    content_changed ? "Reloaded" : "Revalidated", raceline_.source_path.c_str(),
    raceline_.points.size(), line_id_.c_str(), raceline_.source_hash.c_str());
}

void RacelinePathPublisherNode::check_for_reload()
{
  const auto observed_signature = raceline_file_signature(raceline_.source_path);
  if (source_available_ && observed_signature && loaded_signature_ &&
      *observed_signature == *loaded_signature_)
  {
    return;
  }
  if (observed_signature && rejected_signature_ &&
      *observed_signature == *rejected_signature_)
  {
    return;
  }

  try
  {
    auto candidate = load_stable_raceline_csv(raceline_root_, raceline_csv_, limits_);
    const bool content_changed = candidate.data.source_hash != raceline_.source_hash;
    raceline_ = std::move(candidate.data);
    loaded_signature_ = candidate.signature;
    rejected_signature_.reset();
    source_available_ = true;
    publish_outputs();
    log_reload_success(content_changed);
  }
  catch (const std::exception & error)
  {
    const auto signature_after = raceline_file_signature(raceline_.source_path);
    if (observed_signature && signature_after && *observed_signature == *signature_after)
    {
      rejected_signature_ = *observed_signature;
    }
    else
    {
      rejected_signature_.reset();
    }
    std::ostringstream issue_key;
    issue_key << error.what();
    if (observed_signature)
    {
      issue_key << ':' << observed_signature->device << ':' << observed_signature->inode << ':'
                << observed_signature->size << ':' << observed_signature->modified_seconds << ':'
                << observed_signature->modified_nanoseconds << ':'
                << observed_signature->changed_seconds << ':'
                << observed_signature->changed_nanoseconds;
    }
    fail_closed(issue_key.str(), error.what());
  }
}

}  // namespace jetpilot_planning
