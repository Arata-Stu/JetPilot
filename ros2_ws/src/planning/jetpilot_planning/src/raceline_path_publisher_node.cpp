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

  const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  path_pub_ = create_publisher<nav_msgs::msg::Path>(output_topic, path_qos);
  publish_path(frame_id);

  RCLCPP_INFO(get_logger(), "Published %zu raceline poses from '%s' on '%s' (frame '%s')",
              raceline_.points.size(), raceline_.source_path.c_str(), output_topic.c_str(),
              frame_id.c_str());
}

void RacelinePathPublisherNode::publish_path(const std::string & frame_id)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = now();
  path.poses.reserve(raceline_.points.size());
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
    path.poses.push_back(std::move(pose));
  }
  path_pub_->publish(path);
}

}  // namespace jetpilot_planning
