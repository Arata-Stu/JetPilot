#ifndef JETPILOT_PLANNING__ROUTE_LANE_SELECTOR_NODE_HPP_
#define JETPILOT_PLANNING__ROUTE_LANE_SELECTOR_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "jetpilot_msgs/msg/trajectory.hpp"
#include "jetpilot_planning/lane_selection_policy.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace jetpilot_planning
{

class RouteLaneSelectorNode : public rclcpp::Node
{
public:
  RouteLaneSelectorNode();

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct CandidateCache
  {
    nav_msgs::msg::Path::ConstSharedPtr path;
    jetpilot_msgs::msg::Trajectory::ConstSharedPtr trajectory;
    std::optional<SteadyTime> received_at;
    bool expects_typed{false};
    bool valid{false};
    std::string validation_reason{"not_received"};
  };

  struct SelectionSnapshot
  {
    SelectionDecision decision;
    nav_msgs::msg::Path::ConstSharedPtr path;
    jetpilot_msgs::msg::Trajectory::ConstSharedPtr trajectory;
    std::string requested_lane_id;
    std::string current_section_id;
    std::unordered_set<std::string> available_lane_ids;
  };

  void validate_configuration(const std::vector<std::string> & lane_ids,
                              const std::vector<std::string> & lane_path_topics,
                              const std::vector<std::string> & lane_trajectory_topics,
                              const std::vector<double> & lane_target_speeds_mps,
                              const std::string & default_lane_id) const;
  static void validate_section_rules(
    const std::unordered_map<std::string, std::string> & section_rules,
    const std::vector<std::string> & lane_ids);
  void on_path(const std::string & lane_id, const nav_msgs::msg::Path::ConstSharedPtr & message);
  void on_trajectory(
    const std::string & lane_id,
    const jetpilot_msgs::msg::Trajectory::ConstSharedPtr & message);
  SelectionSnapshot snapshot_selection();
  void publish_outputs();

  std::string output_trajectory_topic_;
  std::string output_profile_topic_;
  double publish_rate_hz_{10.0};
  double path_timeout_sec_{0.0};
  double requested_lane_timeout_sec_{1.0};
  double current_section_timeout_sec_{1.0};
  bool require_requested_lane_heartbeat_{false};
  bool section_rules_configured_{false};
  std::size_t min_path_poses_{2U};
  double min_path_length_m_{0.10};
  std::unique_ptr<LaneSelectionPolicy> policy_;
  std::unordered_map<std::string, double> lane_target_speeds_mps_;

  std::mutex state_mutex_;
  std::unordered_map<std::string, CandidateCache> candidate_caches_;
  std::string requested_lane_id_;
  std::string current_section_id_;
  std::optional<SteadyTime> requested_lane_received_at_;
  std::optional<SteadyTime> current_section_received_at_;

  bool last_ready_initialized_{false};
  bool last_ready_{false};
  std::string last_selected_lane_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<jetpilot_msgs::msg::Trajectory>::SharedPtr profile_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_lane_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subscriptions_;
  std::vector<rclcpp::Subscription<jetpilot_msgs::msg::Trajectory>::SharedPtr>
    trajectory_subscriptions_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr requested_lane_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_section_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace jetpilot_planning

#endif  // JETPILOT_PLANNING__ROUTE_LANE_SELECTOR_NODE_HPP_
