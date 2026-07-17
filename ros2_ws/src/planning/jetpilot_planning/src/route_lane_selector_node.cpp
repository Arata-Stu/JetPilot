#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "jetpilot_planning/lane_selection_policy.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace jetpilot_planning
{
namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;

double seconds_since(const SteadyTime & timestamp)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - timestamp).count();
}

struct PathValidation
{
  bool valid{false};
  std::string reason;
};

PathValidation validate_path(
  const nav_msgs::msg::Path & path, const std::size_t min_path_poses,
  const double min_path_length_m)
{
  if (path.header.frame_id.empty()) {
    return {false, "empty_frame_id"};
  }
  if (path.poses.size() < min_path_poses) {
    return {false, "not_enough_poses"};
  }

  double length = 0.0;
  for (std::size_t index = 0U; index < path.poses.size(); ++index) {
    const auto & position = path.poses[index].pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
      return {false, "non_finite_position"};
    }
    if (index > 0U) {
      const auto & previous = path.poses[index - 1U].pose.position;
      length += std::hypot(position.x - previous.x, position.y - previous.y);
    }
  }
  if (length < min_path_length_m) {
    return {false, "path_too_short"};
  }
  return {true, "ok"};
}

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string join(const std::unordered_set<std::string> & values)
{
  std::vector<std::string> ordered(values.begin(), values.end());
  std::sort(ordered.begin(), ordered.end());
  std::ostringstream stream;
  for (std::size_t index = 0U; index < ordered.size(); ++index) {
    if (index > 0U) {
      stream << ',';
    }
    stream << ordered[index];
  }
  return stream.str();
}

}  // namespace

class RouteLaneSelectorNode : public rclcpp::Node
{
public:
  RouteLaneSelectorNode()
  : Node("route_lane_selector")
  {
    const auto lane_ids = declare_parameter<std::vector<std::string>>(
      "lane_ids", std::vector<std::string>{"primary"});
    const auto lane_path_topics = declare_parameter<std::vector<std::string>>(
      "lane_path_topics", std::vector<std::string>{"/hd_map/primary_centerline_path"});
    const auto lane_target_speeds_mps = declare_parameter<std::vector<double>>(
      "lane_target_speeds_mps", std::vector<double>{1.0});
    const auto default_lane_id = declare_parameter<std::string>("default_lane_id", "primary");
    const auto section_rule_entries = declare_parameter<std::vector<std::string>>(
      "section_lane_rules", std::vector<std::string>{});
    const auto fallback_to_default_lane = declare_parameter<bool>(
      "fallback_to_default_lane", false);

    output_trajectory_topic_ = declare_parameter<std::string>(
      "output_trajectory_topic", "/planning/trajectory");
    const auto target_speed_topic = declare_parameter<std::string>(
      "target_speed_topic", "/planning/target_speed");
    const auto requested_lane_topic = declare_parameter<std::string>(
      "requested_lane_topic", "/planning/requested_lane");
    const auto current_section_topic = declare_parameter<std::string>(
      "current_section_topic", "/localization/current_section");
    const auto selected_lane_topic = declare_parameter<std::string>(
      "selected_lane_topic", "/planning/selected_lane");
    const auto ready_topic = declare_parameter<std::string>("ready_topic", "/planning/ready");
    const auto diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/diagnostics");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    path_timeout_sec_ = declare_parameter<double>("path_timeout_sec", 0.0);
    requested_lane_timeout_sec_ = declare_parameter<double>("requested_lane_timeout_sec", 1.0);
    current_section_timeout_sec_ = declare_parameter<double>("current_section_timeout_sec", 1.0);
    require_requested_lane_heartbeat_ = declare_parameter<bool>(
      "require_requested_lane_heartbeat", false);
    const auto min_path_poses_parameter = std::max(
      declare_parameter<int64_t>("min_path_poses", 2), int64_t{2});
    min_path_poses_ = static_cast<std::size_t>(min_path_poses_parameter);
    min_path_length_m_ = declare_parameter<double>("min_path_length_m", 0.10);
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ < 0.1 ||
      !std::isfinite(path_timeout_sec_) || path_timeout_sec_ < 0.0 ||
      !std::isfinite(requested_lane_timeout_sec_) || requested_lane_timeout_sec_ < 0.0 ||
      !std::isfinite(current_section_timeout_sec_) || current_section_timeout_sec_ < 0.0 ||
      !std::isfinite(min_path_length_m_) || min_path_length_m_ < 0.0)
    {
      throw std::invalid_argument(
              "planning rates, timeouts, and path length limits must be finite and valid");
    }

    validate_configuration(
      lane_ids, lane_path_topics, lane_target_speeds_mps, default_lane_id);
    const auto section_rules = parse_section_lane_rules(section_rule_entries);
    section_rules_configured_ = !section_rules.empty();
    validate_section_rules(section_rules, lane_ids);
    policy_ = std::make_unique<LaneSelectionPolicy>(
      default_lane_id, section_rules, fallback_to_default_lane);
    for (std::size_t index = 0U; index < lane_ids.size(); ++index) {
      lane_target_speeds_mps_.emplace(lane_ids[index], lane_target_speeds_mps[index]);
    }

    const auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
      output_trajectory_topic_, transient_qos);
    target_speed_pub_ = create_publisher<std_msgs::msg::Float32>(
      target_speed_topic, transient_qos);
    selected_lane_pub_ = create_publisher<std_msgs::msg::String>(selected_lane_topic, transient_qos);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(ready_topic, transient_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    const auto request_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    requested_lane_sub_ = create_subscription<std_msgs::msg::String>(
      requested_lane_topic, request_qos,
      [this](const std_msgs::msg::String::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        requested_lane_id_ = message->data;
        requested_lane_received_at_ = std::chrono::steady_clock::now();
      });
    current_section_sub_ = create_subscription<std_msgs::msg::String>(
      current_section_topic, request_qos,
      [this](const std_msgs::msg::String::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_section_id_ = message->data;
        current_section_received_at_ = std::chrono::steady_clock::now();
      });

    const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    for (std::size_t index = 0U; index < lane_ids.size(); ++index) {
      const auto lane_id = lane_ids[index];
      path_caches_.emplace(lane_id, PathCache{});
      path_subscriptions_.push_back(
        create_subscription<nav_msgs::msg::Path>(
          lane_path_topics[index], path_qos,
          [this, lane_id](const nav_msgs::msg::Path::ConstSharedPtr message) {
            on_path(lane_id, message);
          }));
    }

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      [this]() {publish_outputs();});

    RCLCPP_INFO(
      get_logger(), "Route lane selector configured with %zu lane(s); default='%s', output='%s'",
      lane_ids.size(), default_lane_id.c_str(), output_trajectory_topic_.c_str());
  }

private:
  struct PathCache
  {
    nav_msgs::msg::Path::ConstSharedPtr path;
    std::optional<SteadyTime> received_at;
    bool valid{false};
    std::string validation_reason{"not_received"};
  };

  struct SelectionSnapshot
  {
    SelectionDecision decision;
    nav_msgs::msg::Path::ConstSharedPtr path;
    std::string requested_lane_id;
    std::string current_section_id;
    std::unordered_set<std::string> available_lane_ids;
  };

  void validate_configuration(
    const std::vector<std::string> & lane_ids,
    const std::vector<std::string> & lane_path_topics,
    const std::vector<double> & lane_target_speeds_mps,
    const std::string & default_lane_id) const
  {
    if (
      lane_ids.empty() || lane_ids.size() != lane_path_topics.size() ||
      lane_ids.size() != lane_target_speeds_mps.size())
    {
      throw std::invalid_argument(
              "lane_ids, lane_path_topics, and lane_target_speeds_mps must be "
              "non-empty arrays of equal length");
    }
    if (output_trajectory_topic_.empty()) {
      throw std::invalid_argument("output_trajectory_topic must not be empty");
    }

    std::unordered_set<std::string> unique_lane_ids;
    std::unordered_set<std::string> unique_path_topics;
    for (std::size_t index = 0U; index < lane_ids.size(); ++index) {
      if (lane_ids[index].empty() || lane_path_topics[index].empty()) {
        throw std::invalid_argument("lane IDs and path topics must not be empty");
      }
      if (!unique_lane_ids.insert(lane_ids[index]).second) {
        throw std::invalid_argument("duplicate lane ID: " + lane_ids[index]);
      }
      if (!unique_path_topics.insert(lane_path_topics[index]).second) {
        throw std::invalid_argument("duplicate lane path topic: " + lane_path_topics[index]);
      }
      if (lane_path_topics[index] == output_trajectory_topic_) {
        throw std::invalid_argument(
                "a lane input topic must not equal output_trajectory_topic");
      }
      if (!std::isfinite(lane_target_speeds_mps[index]) || lane_target_speeds_mps[index] < 0.0) {
        throw std::invalid_argument("lane target speeds must be finite and non-negative");
      }
    }
    if (unique_lane_ids.count(default_lane_id) == 0U) {
      throw std::invalid_argument("default_lane_id is not present in lane_ids: " + default_lane_id);
    }
  }

  static void validate_section_rules(
    const std::unordered_map<std::string, std::string> & section_rules,
    const std::vector<std::string> & lane_ids)
  {
    const std::unordered_set<std::string> configured_lanes(lane_ids.begin(), lane_ids.end());
    for (const auto & rule : section_rules) {
      if (configured_lanes.count(rule.second) == 0U) {
        throw std::invalid_argument(
                "section_lane_rules references an unknown lane: " + rule.second);
      }
    }
  }

  void on_path(
    const std::string & lane_id, const nav_msgs::msg::Path::ConstSharedPtr & message)
  {
    const auto validation = validate_path(*message, min_path_poses_, min_path_length_m_);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto & cache = path_caches_.at(lane_id);
      cache.path = message;
      cache.received_at = std::chrono::steady_clock::now();
      cache.valid = validation.valid;
      cache.validation_reason = validation.reason;
    }
    if (!validation.valid) {
      RCLCPP_WARN(
        get_logger(), "Rejected path for lane '%s': %s", lane_id.c_str(),
        validation.reason.c_str());
    }
  }

  SelectionSnapshot snapshot_selection()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    SelectionSnapshot snapshot;
    snapshot.requested_lane_id = requested_lane_id_;
    snapshot.current_section_id = current_section_id_;
    for (const auto & [lane_id, cache] : path_caches_) {
      if (!cache.valid || !cache.path) {
        continue;
      }
      if (path_timeout_sec_ > 0.0) {
        if (!cache.received_at || seconds_since(*cache.received_at) > path_timeout_sec_) {
          continue;
        }
      }
      snapshot.available_lane_ids.insert(lane_id);
    }

    const bool requested_lane_watchdog_active =
      require_requested_lane_heartbeat_ || !snapshot.requested_lane_id.empty();
    if (requested_lane_watchdog_active && requested_lane_timeout_sec_ > 0.0 &&
      (!requested_lane_received_at_ ||
      seconds_since(*requested_lane_received_at_) > requested_lane_timeout_sec_))
    {
      snapshot.decision = {{}, SelectionSource::kNone, "requested_lane_state_stale"};
      return snapshot;
    }
    if (section_rules_configured_) {
      if (current_section_timeout_sec_ > 0.0 &&
        (!current_section_received_at_ ||
        seconds_since(*current_section_received_at_) > current_section_timeout_sec_))
      {
        snapshot.decision = {{}, SelectionSource::kNone, "current_section_state_stale"};
        return snapshot;
      }
      if (snapshot.current_section_id.empty() || snapshot.current_section_id == "unknown") {
        snapshot.decision = {{}, SelectionSource::kNone, "current_section_unknown"};
        return snapshot;
      }
    }

    snapshot.decision = policy_->select(
      snapshot.requested_lane_id, snapshot.current_section_id,
      snapshot.available_lane_ids);
    if (snapshot.decision.ready()) {
      snapshot.path = path_caches_.at(snapshot.decision.lane_id).path;
    }
    return snapshot;
  }

  void publish_outputs()
  {
    auto snapshot = snapshot_selection();
    const auto stamp = now().to_msg();

    nav_msgs::msg::Path output_path;
    if (snapshot.decision.ready() && snapshot.path) {
      output_path = *snapshot.path;
      output_path.header.stamp = stamp;
      for (auto & pose : output_path.poses) {
        pose.header.frame_id = output_path.header.frame_id;
        pose.header.stamp = stamp;
      }
    } else {
      // An empty transient-local path actively invalidates an older selected path.
      output_path.header.stamp = stamp;
    }
    trajectory_pub_->publish(output_path);

    std_msgs::msg::Float32 target_speed;
    target_speed.data = snapshot.decision.ready() ?
      static_cast<float>(lane_target_speeds_mps_.at(snapshot.decision.lane_id)) : 0.0F;
    target_speed_pub_->publish(target_speed);

    std_msgs::msg::String selected_lane;
    selected_lane.data = snapshot.decision.lane_id;
    selected_lane_pub_->publish(selected_lane);

    std_msgs::msg::Bool ready;
    ready.data = snapshot.decision.ready();
    ready_pub_->publish(ready);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "jetpilot_planning/route_lane_selector";
    status.hardware_id = "jetpilot";
    status.level = snapshot.decision.ready() ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = snapshot.decision.reason;
    status.values.push_back(diagnostic_value("ready", ready.data ? "true" : "false"));
    status.values.push_back(diagnostic_value("selected_lane", selected_lane.data));
    status.values.push_back(diagnostic_value(
      "target_speed_mps", std::to_string(target_speed.data)));
    status.values.push_back(diagnostic_value(
      "selection_source", selection_source_name(snapshot.decision.source)));
    status.values.push_back(diagnostic_value("requested_lane", snapshot.requested_lane_id));
    status.values.push_back(diagnostic_value("current_section", snapshot.current_section_id));
    status.values.push_back(diagnostic_value(
      "available_lanes", join(snapshot.available_lane_ids)));
    diagnostics.status.push_back(std::move(status));
    diagnostics_pub_->publish(diagnostics);

    const bool ready_changed = !last_ready_initialized_ || last_ready_ != ready.data;
    const bool lane_changed = last_selected_lane_ != selected_lane.data;
    if (ready_changed || lane_changed) {
      if (ready.data) {
        RCLCPP_INFO(
          get_logger(), "Selected lane '%s' (%s)", selected_lane.data.c_str(),
          snapshot.decision.reason.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Planning path unavailable: %s", snapshot.decision.reason.c_str());
      }
      last_ready_ = ready.data;
      last_ready_initialized_ = true;
      last_selected_lane_ = selected_lane.data;
    }
  }

  std::string output_trajectory_topic_;
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
  std::unordered_map<std::string, PathCache> path_caches_;
  std::string requested_lane_id_;
  std::string current_section_id_;
  std::optional<SteadyTime> requested_lane_received_at_;
  std::optional<SteadyTime> current_section_received_at_;

  bool last_ready_initialized_{false};
  bool last_ready_{false};
  std::string last_selected_lane_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_lane_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subscriptions_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr requested_lane_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_section_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace jetpilot_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<jetpilot_planning::RouteLaneSelectorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("route_lane_selector"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
