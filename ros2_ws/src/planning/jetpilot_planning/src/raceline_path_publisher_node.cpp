#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jetpilot_planning/raceline_path_publisher_node.hpp"

namespace jetpilot_planning
{

RacelinePathPublisherNode::RacelinePathPublisherNode() : Node("raceline_path_publisher")
{
  const auto raceline_root = declare_parameter<std::string>("raceline_root", "");
  const auto raceline_csv = declare_parameter<std::string>("raceline_csv", "");
  const auto output_topic =
    declare_parameter<std::string>("output_topic", "/planning/raceline_path");
  const auto trajectory_output_topic = declare_parameter<std::string>(
    "trajectory_output_topic", "/planning/raceline_trajectory");
  const auto publish_typed_trajectory = declare_parameter<bool>("publish_typed_trajectory", true);
  const auto line_id = declare_parameter<std::string>("line_id", "raceline");
  auto line_name = declare_parameter<std::string>("line_name", "");
  auto source_hash = declare_parameter<std::string>("source_hash", "");
  const auto closed = declare_parameter<bool>("closed", true);
  const auto frame_id = declare_parameter<std::string>("frame_id", "map");
  const auto max_file_bytes = declare_parameter<int64_t>("max_file_bytes", 16 * 1024 * 1024);
  const auto max_points = declare_parameter<int64_t>("max_points", 200000);

  if (raceline_csv.empty())
  {
    throw std::invalid_argument(
      "raceline_csv is required; keep "
      "enable_raceline_publisher=false when unused");
  }
  if (output_topic.empty())
  {
    throw std::invalid_argument("output_topic must not be empty");
  }
  if (line_name.empty())
  {
    line_name = line_id;
  }
  if (publish_typed_trajectory &&
      (trajectory_output_topic.empty() || line_id.empty() || line_name.empty()))
  {
    throw std::invalid_argument(
      "trajectory_output_topic, line_id and line_name must not be empty when typed publishing is enabled");
  }
  if (frame_id.empty())
  {
    throw std::invalid_argument("frame_id must not be empty");
  }
  if (max_file_bytes <= 0 || max_points < 2)
  {
    throw std::invalid_argument("raceline CSV limits must be positive");
  }

  RacelineCsvLimits limits;
  limits.max_file_bytes = static_cast<std::uintmax_t>(max_file_bytes);
  limits.max_points = static_cast<std::size_t>(max_points);
  raceline_ = load_raceline_csv(raceline_root, raceline_csv, limits);
  if (source_hash.empty())
  {
    source_hash = raceline_.source_hash;
  }
  else if (source_hash != raceline_.source_hash)
  {
    throw std::runtime_error(
      "source_hash does not match the selected trajectory CSV; refusing mixed geometry/speed");
  }

  const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  path_pub_ = create_publisher<nav_msgs::msg::Path>(output_topic, path_qos);
  if (publish_typed_trajectory)
  {
    trajectory_pub_ = create_publisher<jetpilot_msgs::msg::Trajectory>(
      trajectory_output_topic, path_qos);
  }
  publish_outputs(frame_id, line_id, line_name, source_hash, closed);

  RCLCPP_INFO(
    get_logger(),
    "Published %zu trajectory poses from '%s' on '%s' (typed='%s', line='%s', hash='%s')",
    raceline_.points.size(), raceline_.source_path.c_str(), output_topic.c_str(),
    publish_typed_trajectory ? trajectory_output_topic.c_str() : "disabled", line_id.c_str(),
    source_hash.c_str());
}

void RacelinePathPublisherNode::publish_outputs(
  const std::string & frame_id, const std::string & line_id, const std::string & line_name,
  const std::string & source_hash, const bool closed)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = now();
  jetpilot_msgs::msg::Trajectory trajectory;
  trajectory.header = path.header;
  trajectory.line_id = line_id;
  trajectory.display_name = line_name;
  trajectory.source_hash = source_hash;
  trajectory.closed = closed;
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

}  // namespace jetpilot_planning
