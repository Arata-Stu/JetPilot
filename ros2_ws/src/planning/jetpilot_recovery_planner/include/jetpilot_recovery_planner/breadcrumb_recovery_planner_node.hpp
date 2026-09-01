#ifndef JETPILOT_RECOVERY_PLANNER__BREADCRUMB_RECOVERY_PLANNER_NODE_HPP_
#define JETPILOT_RECOVERY_PLANNER__BREADCRUMB_RECOVERY_PLANNER_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "jetpilot_msgs/msg/recovery_status.hpp"
#include "jetpilot_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace jetpilot_recovery_planner
{
class BreadcrumbRecoveryPlannerNode : public rclcpp::Node
{
public:
  BreadcrumbRecoveryPlannerNode();

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  struct Breadcrumb { geometry_msgs::msg::Pose pose; };
  void on_odometry(const nav_msgs::msg::Odometry & odometry);
  void on_request(bool requested);
  void start_recovery();
  void update();
  void publish_state(std::uint8_t state, const std::string & reason, float remaining);
  void publish_ready(bool ready);
  static double distance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b);

  double sample_spacing_m_{0.05};
  double history_length_m_{3.0};
  double recovery_distance_m_{0.70};
  double goal_tolerance_m_{0.10};
  double timeout_s_{6.0};
  double reverse_speed_mps_{0.20};
  std::size_t minimum_points_{4U};
  std::deque<Breadcrumb> history_;
  std::optional<nav_msgs::msg::Odometry> odometry_;
  std::optional<jetpilot_msgs::msg::Trajectory> active_trajectory_;
  std::optional<SteadyTime> recovery_started_at_;
  bool request_active_{false};
  bool terminal_{false};
  std::uint64_t sequence_{0U};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr request_sub_;
  rclcpp::Publisher<jetpilot_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<jetpilot_msgs::msg::RecoveryStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace jetpilot_recovery_planner
#endif
