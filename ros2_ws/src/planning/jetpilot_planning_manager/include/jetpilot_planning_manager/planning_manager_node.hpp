#ifndef JETPILOT_PLANNING_MANAGER__PLANNING_MANAGER_NODE_HPP_
#define JETPILOT_PLANNING_MANAGER__PLANNING_MANAGER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "jetpilot_msgs/msg/direction_signal.hpp"
#include "jetpilot_msgs/msg/junction_array.hpp"
#include "jetpilot_msgs/msg/planning_manager_status.hpp"
#include "jetpilot_msgs/msg/recovery_status.hpp"
#include "jetpilot_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace jetpilot_planning_manager
{
class PlanningManagerNode : public rclcpp::Node
{
public:
  PlanningManagerNode();

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  using RouteStamp = std::pair<std::int32_t, std::uint32_t>;
  struct JunctionRule { jetpilot_msgs::msg::Junction junction; };
  struct PendingRouteCycle
  {
    bool path_received{false};
    bool profile_received{false};
    bool diagnostics_received{false};
    bool diagnostics_valid{false};
    bool ready{false};
    std::optional<nav_msgs::msg::Path> path;
    std::optional<jetpilot_msgs::msg::Trajectory> profile;
    std::string selected_lane;
    float target_speed_mps{0.0F};
    SteadyTime updated_at{std::chrono::steady_clock::now()};
  };
  void on_section(const std::string & section, bool received = true);
  void on_signal(const jetpilot_msgs::msg::DirectionSignal & signal);
  void on_collision(bool detected);
  void on_recovery_status(const jetpilot_msgs::msg::RecoveryStatus & status);
  void on_route_path(const nav_msgs::msg::Path & path);
  void on_route_profile(const jetpilot_msgs::msg::Trajectory & profile);
  void on_route_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & diagnostics);
  void reset_route_epoch_if_clock_rewound();
  void resolve_route_cycle(const RouteStamp & stamp);
  void prune_pending_route_cycles();
  void invalidate_route_bundle(bool reset_pending_cycles = true);
  void clear_committed_lane();
  void publish_cycle();
  void publish_ready(bool ready);
  void publish_status(std::uint8_t state, const std::string & reason);
  std::string lane_for_direction(const jetpilot_msgs::msg::Junction & junction,
                                 std::uint8_t direction) const;
  nav_msgs::msg::Path path_from_profile(const jetpilot_msgs::msg::Trajectory & profile) const;
  std::string desired_lane_id() const;

  double publish_rate_hz_{10.0};
  double recovery_target_speed_mps_{0.20};
  double current_section_timeout_s_{1.0};
  double route_bundle_timeout_s_{0.5};
  bool require_junction_map_{true};
  std::string default_lane_id_{"primary"};
  std::string route_diagnostics_topic_{"/planning/route/diagnostics"};
  std::unordered_map<std::string, JunctionRule> rule_by_section_;
  std::unordered_set<std::string> release_sections_;
  std::string junction_map_fingerprint_;
  std::string current_section_;
  std::string active_junction_id_;
  std::string committed_lane_id_;
  std::optional<jetpilot_msgs::msg::Junction> active_junction_;
  bool collision_active_{false};
  bool collision_input_{false};
  bool recovery_failed_{false};
  bool route_ready_{false};
  bool junction_map_received_{false};
  std::optional<SteadyTime> current_section_received_at_;
  std::optional<SteadyTime> route_bundle_received_at_;
  bool recovery_ready_{false};
  std::optional<nav_msgs::msg::Path> route_path_;
  std::optional<jetpilot_msgs::msg::Trajectory> route_profile_;
  std::optional<jetpilot_msgs::msg::Trajectory> recovery_profile_;
  float route_target_speed_{0.0F};
  std::string selected_lane_;
  std::map<RouteStamp, PendingRouteCycle> pending_route_cycles_;
  std::optional<RouteStamp> latest_completed_route_stamp_;
  std::optional<RouteStamp> minimum_route_stamp_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<jetpilot_msgs::msg::Trajectory>::SharedPtr profile_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr requested_lane_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_request_pub_;
  rclcpp::Publisher<jetpilot_msgs::msg::PlanningManagerStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<jetpilot_msgs::msg::JunctionArray>::SharedPtr junctions_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr section_sub_;
  rclcpp::Subscription<jetpilot_msgs::msg::DirectionSignal>::SharedPtr signal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Subscription<jetpilot_msgs::msg::RecoveryStatus>::SharedPtr recovery_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_path_sub_;
  rclcpp::Subscription<jetpilot_msgs::msg::Trajectory>::SharedPtr route_profile_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr route_diagnostics_sub_;
  rclcpp::Subscription<jetpilot_msgs::msg::Trajectory>::SharedPtr recovery_profile_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_ready_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace jetpilot_planning_manager
#endif
