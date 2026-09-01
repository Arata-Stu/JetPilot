#include "jetpilot_recovery_planner/breadcrumb_recovery_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace jetpilot_recovery_planner
{

BreadcrumbRecoveryPlannerNode::BreadcrumbRecoveryPlannerNode()
: Node("breadcrumb_recovery_planner_node")
{
  sample_spacing_m_ = declare_parameter<double>("sample_spacing_m", 0.05);
  history_length_m_ = declare_parameter<double>("history_length_m", 3.0);
  recovery_distance_m_ = declare_parameter<double>("recovery_distance_m", 0.70);
  goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.10);
  timeout_s_ = declare_parameter<double>("timeout_s", 6.0);
  reverse_speed_mps_ = declare_parameter<double>("reverse_speed_mps", 0.20);
  const auto minimum_points = declare_parameter<int>("minimum_points", 4);
  if (sample_spacing_m_ <= 0.0 || history_length_m_ <= recovery_distance_m_ ||
      recovery_distance_m_ <= goal_tolerance_m_ || goal_tolerance_m_ <= 0.0 ||
      timeout_s_ <= 0.0 || reverse_speed_mps_ <= 0.0 || minimum_points < 2)
    throw std::invalid_argument("invalid breadcrumb recovery parameters");
  minimum_points_ = static_cast<std::size_t>(minimum_points);
  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/visual_slam/tracking/odometry", 20,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { on_odometry(*message); });
  request_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/planning/recovery/request", 10,
    [this](std_msgs::msg::Bool::ConstSharedPtr message) { on_request(message->data); });
  trajectory_pub_ = create_publisher<jetpilot_msgs::msg::Trajectory>(
    "/planning/recovery/trajectory_profile", latched);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("/planning/recovery/ready", latched);
  status_pub_ = create_publisher<jetpilot_msgs::msg::RecoveryStatus>(
    "/planning/recovery/status", latched);
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { update(); });
}

double BreadcrumbRecoveryPlannerNode::distance(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

void BreadcrumbRecoveryPlannerNode::on_odometry(const nav_msgs::msg::Odometry & odometry)
{
  odometry_ = odometry;
  if (request_active_) return;
  const auto & pose = odometry.pose.pose;
  if (history_.empty() || distance(history_.back().pose, pose) >= sample_spacing_m_)
    history_.push_back({pose});
  double length = 0.0;
  for (std::size_t i = history_.size(); i > 1U; --i)
  {
    length += distance(history_[i - 1U].pose, history_[i - 2U].pose);
    if (length > history_length_m_)
    {
      history_.erase(history_.begin(), history_.begin() + static_cast<std::ptrdiff_t>(i - 2U));
      break;
    }
  }
}

void BreadcrumbRecoveryPlannerNode::on_request(bool requested)
{
  if (requested && !request_active_)
  {
    request_active_ = true;
    terminal_ = false;
    start_recovery();
  }
  else if (!requested && request_active_)
  {
    request_active_ = false;
    terminal_ = false;
    active_trajectory_.reset();
    recovery_started_at_.reset();
    publish_ready(false);
    publish_state(jetpilot_msgs::msg::RecoveryStatus::RECORDING, "recording_breadcrumbs", 0.0F);
  }
}

void BreadcrumbRecoveryPlannerNode::start_recovery()
{
  if (!odometry_ || history_.size() < minimum_points_)
  {
    terminal_ = true;
    publish_ready(false);
    publish_state(jetpilot_msgs::msg::RecoveryStatus::FAILED, "insufficient_breadcrumb_history", 0.0F);
    return;
  }
  jetpilot_msgs::msg::Trajectory trajectory;
  trajectory.header = odometry_->header;
  trajectory.header.stamp = now();
  trajectory.line_id = "breadcrumb_recovery";
  trajectory.display_name = "Breadcrumb recovery";
  trajectory.source_hash = "breadcrumb_" + std::to_string(++sequence_);
  trajectory.closed = false;
  trajectory.motion_direction = jetpilot_msgs::msg::Trajectory::MOTION_REVERSE;

  const auto add_point = [this, &trajectory](const geometry_msgs::msg::Pose & pose, double station) {
    jetpilot_msgs::msg::TrajectoryPoint point;
    point.s_m = station;
    point.pose = pose;
    point.curvature_radpm = 0.0;
    point.longitudinal_velocity_mps = reverse_speed_mps_;
    point.longitudinal_acceleration_mps2 = 0.0;
    trajectory.points.push_back(point);
  };
  double station = 0.0;
  auto previous = odometry_->pose.pose;
  add_point(previous, station);
  for (auto iterator = history_.rbegin(); iterator != history_.rend(); ++iterator)
  {
    const double segment = distance(previous, iterator->pose);
    if (segment < sample_spacing_m_ * 0.5) continue;
    station += segment;
    add_point(iterator->pose, station);
    previous = iterator->pose;
    if (station >= recovery_distance_m_) break;
  }
  if (trajectory.points.size() < minimum_points_ || station < recovery_distance_m_ * 0.5)
  {
    terminal_ = true;
    publish_ready(false);
    publish_state(jetpilot_msgs::msg::RecoveryStatus::FAILED, "breadcrumb_path_too_short",
                  static_cast<float>(station));
    return;
  }
  active_trajectory_ = trajectory;
  recovery_started_at_ = std::chrono::steady_clock::now();
  trajectory_pub_->publish(trajectory);
  publish_ready(true);
  publish_state(jetpilot_msgs::msg::RecoveryStatus::REVERSING, "reversing_breadcrumb",
                static_cast<float>(station));
}

void BreadcrumbRecoveryPlannerNode::update()
{
  if (!request_active_)
  {
    publish_state(jetpilot_msgs::msg::RecoveryStatus::RECORDING, "recording_breadcrumbs", 0.0F);
    return;
  }
  if (terminal_ || !active_trajectory_ || !odometry_ || !recovery_started_at_) return;
  const auto & goal = active_trajectory_->points.back().pose;
  const double remaining = distance(odometry_->pose.pose, goal);
  if (remaining <= goal_tolerance_m_)
  {
    terminal_ = true;
    publish_ready(false);
    publish_state(jetpilot_msgs::msg::RecoveryStatus::SUCCEEDED, "recovery_goal_reached",
                  static_cast<float>(remaining));
    return;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - *recovery_started_at_).count();
  if (elapsed > timeout_s_)
  {
    terminal_ = true;
    publish_ready(false);
    publish_state(jetpilot_msgs::msg::RecoveryStatus::FAILED, "recovery_timeout",
                  static_cast<float>(remaining));
    return;
  }
  auto trajectory = *active_trajectory_;
  trajectory.header.stamp = now();
  trajectory_pub_->publish(trajectory);
  publish_ready(true);
  publish_state(jetpilot_msgs::msg::RecoveryStatus::REVERSING, "reversing_breadcrumb",
                static_cast<float>(remaining));
}

void BreadcrumbRecoveryPlannerNode::publish_ready(bool ready)
{
  std_msgs::msg::Bool message;
  message.data = ready;
  ready_pub_->publish(message);
}

void BreadcrumbRecoveryPlannerNode::publish_state(
  std::uint8_t state, const std::string & reason, float remaining)
{
  jetpilot_msgs::msg::RecoveryStatus message;
  message.header.stamp = now();
  message.state = state;
  message.reason = reason;
  message.remaining_distance_m = remaining;
  status_pub_->publish(message);
}

}  // namespace jetpilot_recovery_planner
