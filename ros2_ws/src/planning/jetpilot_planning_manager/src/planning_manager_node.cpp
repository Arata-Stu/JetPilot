#include "jetpilot_planning_manager/planning_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace jetpilot_planning_manager
{
namespace
{

using RouteStamp = std::pair<std::int32_t, std::uint32_t>;

RouteStamp route_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return {stamp.sec, stamp.nanosec};
}

std::optional<std::string> diagnostic_value(
  const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  const auto value = std::find_if(
    status.values.begin(), status.values.end(), [&key](const auto & item) {
      return item.key == key;
    });
  if (value == status.values.end()) return std::nullopt;
  return value->value;
}

std::optional<float> finite_nonnegative_float(const std::string & value)
{
  std::size_t parsed_characters = 0U;
  try
  {
    const auto parsed = std::stof(value, &parsed_characters);
    if (parsed_characters == value.size() && std::isfinite(parsed) && parsed >= 0.0F)
      return parsed;
  }
  catch (const std::exception &)
  {
  }
  return std::nullopt;
}

void append_fingerprint_field(std::ostringstream & stream, const std::string & value)
{
  stream << value.size() << ':' << value << ';';
}

void append_sorted_fields(std::ostringstream & stream, std::vector<std::string> values)
{
  std::sort(values.begin(), values.end());
  stream << values.size() << '[';
  for (const auto & value : values) append_fingerprint_field(stream, value);
  stream << ']';
}

std::string fingerprint_double(const double value)
{
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  return std::to_string(bits);
}

std::string junction_map_fingerprint(const jetpilot_msgs::msg::JunctionArray & message)
{
  std::vector<std::string> junctions;
  junctions.reserve(message.junctions.size());
  for (const auto & junction : message.junctions)
  {
    std::ostringstream stream;
    append_fingerprint_field(stream, junction.id);
    append_fingerprint_field(stream, junction.signal_id);
    append_fingerprint_field(stream, junction.left_lane_id);
    append_fingerprint_field(stream, junction.straight_lane_id);
    append_fingerprint_field(stream, junction.right_lane_id);
    append_fingerprint_field(stream, fingerprint_double(junction.position.x));
    append_fingerprint_field(stream, fingerprint_double(junction.position.y));
    append_fingerprint_field(stream, fingerprint_double(junction.position.z));
    append_sorted_fields(stream, junction.activation_section_ids);
    append_sorted_fields(stream, junction.release_section_ids);
    junctions.push_back(stream.str());
  }
  std::sort(junctions.begin(), junctions.end());
  std::ostringstream output;
  output << junctions.size() << '{';
  for (const auto & junction : junctions) append_fingerprint_field(output, junction);
  output << '}';
  return output.str();
}

}  // namespace

PlanningManagerNode::PlanningManagerNode() : Node("planning_manager_node")
{
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
  recovery_target_speed_mps_ = declare_parameter<double>("recovery_target_speed_mps", 0.20);
  current_section_timeout_s_ = declare_parameter<double>("current_section_timeout_s", 1.0);
  route_bundle_timeout_s_ = declare_parameter<double>("route_bundle_timeout_s", 0.5);
  require_junction_map_ = declare_parameter<bool>("require_junction_map", true);
  default_lane_id_ = declare_parameter<std::string>("default_lane_id", "primary");
  route_diagnostics_topic_ = declare_parameter<std::string>(
    "route_diagnostics_topic", "/planning/route/diagnostics");
  if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
      !std::isfinite(recovery_target_speed_mps_) || recovery_target_speed_mps_ <= 0.0 ||
      !std::isfinite(current_section_timeout_s_) || current_section_timeout_s_ <= 0.0 ||
      !std::isfinite(route_bundle_timeout_s_) || route_bundle_timeout_s_ <= 0.0 ||
      default_lane_id_.empty() || route_diagnostics_topic_.empty())
    throw std::invalid_argument(
            "manager rates, timeouts, lane ID, and diagnostics topic must be valid");

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/planning/trajectory", latched);
  profile_pub_ = create_publisher<jetpilot_msgs::msg::Trajectory>("/planning/trajectory_profile", latched);
  target_speed_pub_ = create_publisher<std_msgs::msg::Float32>("/planning/target_speed", latched);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("/planning/ready", latched);
  requested_lane_pub_ = create_publisher<std_msgs::msg::String>("/planning/requested_lane", 10);
  recovery_request_pub_ = create_publisher<std_msgs::msg::Bool>("/planning/recovery/request", 10);
  status_pub_ = create_publisher<jetpilot_msgs::msg::PlanningManagerStatus>("/planning/manager/status", latched);

  junctions_sub_ = create_subscription<jetpilot_msgs::msg::JunctionArray>(
    "/hd_map/junctions", latched,
    [this](jetpilot_msgs::msg::JunctionArray::ConstSharedPtr message) {
      const auto fingerprint = junction_map_fingerprint(*message);
      const bool revision_changed = junction_map_received_ &&
        fingerprint != junction_map_fingerprint_;
      if (revision_changed)
      {
        committed_lane_id_.clear();
        active_junction_.reset();
        active_junction_id_.clear();
        invalidate_route_bundle();
        RCLCPP_WARN(
          get_logger(),
          "HD map junction rules changed; cleared the committed branch and route bundle");
      }
      rule_by_section_.clear();
      release_sections_.clear();
      junction_map_received_ = true;
      junction_map_fingerprint_ = fingerprint;
      for (const auto & junction : message->junctions)
      {
        for (const auto & section : junction.activation_section_ids)
          rule_by_section_[section] = JunctionRule{junction};
        for (const auto & section : junction.release_section_ids)
          release_sections_.insert(section);
      }
      on_section(current_section_, false);
    });
  section_sub_ = create_subscription<std_msgs::msg::String>(
    "/localization/current_section", 10,
    [this](std_msgs::msg::String::ConstSharedPtr message) { on_section(message->data); });
  signal_sub_ = create_subscription<jetpilot_msgs::msg::DirectionSignal>(
    "/perception/direction_signal", 10,
    [this](jetpilot_msgs::msg::DirectionSignal::ConstSharedPtr message) { on_signal(*message); });
  collision_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/safety/collision_detected", 10,
    [this](std_msgs::msg::Bool::ConstSharedPtr message) { on_collision(message->data); });
  recovery_status_sub_ = create_subscription<jetpilot_msgs::msg::RecoveryStatus>(
    "/planning/recovery/status", latched,
    [this](jetpilot_msgs::msg::RecoveryStatus::ConstSharedPtr message) {
      on_recovery_status(*message);
    });
  route_path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/planning/route/trajectory", latched,
    [this](nav_msgs::msg::Path::ConstSharedPtr message) { on_route_path(*message); });
  route_profile_sub_ = create_subscription<jetpilot_msgs::msg::Trajectory>(
    "/planning/route/trajectory_profile", latched,
    [this](jetpilot_msgs::msg::Trajectory::ConstSharedPtr message) {
      on_route_profile(*message);
    });
  route_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    route_diagnostics_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      on_route_diagnostics(*message);
    });
  recovery_profile_sub_ = create_subscription<jetpilot_msgs::msg::Trajectory>(
    "/planning/recovery/trajectory_profile", latched,
    [this](jetpilot_msgs::msg::Trajectory::ConstSharedPtr message) {
      if (message->points.empty()) recovery_profile_.reset(); else recovery_profile_ = *message;
    });
  recovery_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/planning/recovery/ready", latched,
    [this](std_msgs::msg::Bool::ConstSharedPtr message) { recovery_ready_ = message->data; });
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_)),
    [this]() { publish_cycle(); });
}

void PlanningManagerNode::on_section(const std::string & section, bool received)
{
  current_section_ = section;
  if (received) current_section_received_at_ = std::chrono::steady_clock::now();
  if (release_sections_.find(section) != release_sections_.end())
    clear_committed_lane();
  const auto rule = rule_by_section_.find(section);
  if (rule == rule_by_section_.end())
  {
    active_junction_.reset();
    active_junction_id_.clear();
    return;
  }
  const bool junction_changed = !active_junction_ ||
    active_junction_->id != rule->second.junction.id;
  active_junction_ = rule->second.junction;
  active_junction_id_ = active_junction_->id;
  if (junction_changed)
  {
    committed_lane_id_.clear();
    // Entering a signal-controlled section must stop immediately. Do not
    // leave the previously accepted default/branch bundle ready until the
    // next manager timer tick.
    invalidate_route_bundle();
  }
}

std::string PlanningManagerNode::lane_for_direction(
  const jetpilot_msgs::msg::Junction & junction, std::uint8_t direction) const
{
  if (direction == jetpilot_msgs::msg::DirectionSignal::LEFT) return junction.left_lane_id;
  if (direction == jetpilot_msgs::msg::DirectionSignal::STRAIGHT) return junction.straight_lane_id;
  if (direction == jetpilot_msgs::msg::DirectionSignal::RIGHT) return junction.right_lane_id;
  return {};
}

void PlanningManagerNode::on_signal(const jetpilot_msgs::msg::DirectionSignal & signal)
{
  if (!active_junction_ || !committed_lane_id_.empty() || !signal.stable ||
      signal.signal_id != active_junction_->signal_id ||
      signal.activation_section_id != current_section_) return;
  committed_lane_id_ = lane_for_direction(*active_junction_, signal.direction);
  if (committed_lane_id_.empty())
    RCLCPP_ERROR(get_logger(), "Signal direction has no lane mapping at junction '%s'",
                 active_junction_->id.c_str());
  else
  {
    invalidate_route_bundle();
    RCLCPP_INFO(get_logger(), "Committed junction '%s' to lane '%s'",
                active_junction_->id.c_str(), committed_lane_id_.c_str());
  }
}

void PlanningManagerNode::on_route_path(const nav_msgs::msg::Path & path)
{
  reset_route_epoch_if_clock_rewound();
  const auto stamp = route_stamp(path.header.stamp);
  auto & cycle = pending_route_cycles_[stamp];
  cycle.path_received = true;
  if (path.poses.empty()) cycle.path.reset(); else cycle.path = path;
  cycle.updated_at = std::chrono::steady_clock::now();
  resolve_route_cycle(stamp);
  prune_pending_route_cycles();
}

void PlanningManagerNode::on_route_profile(const jetpilot_msgs::msg::Trajectory & profile)
{
  reset_route_epoch_if_clock_rewound();
  const auto stamp = route_stamp(profile.header.stamp);
  auto & cycle = pending_route_cycles_[stamp];
  cycle.profile_received = true;
  if (profile.points.empty()) cycle.profile.reset(); else cycle.profile = profile;
  cycle.updated_at = std::chrono::steady_clock::now();
  resolve_route_cycle(stamp);
  prune_pending_route_cycles();
}

void PlanningManagerNode::on_route_diagnostics(
  const diagnostic_msgs::msg::DiagnosticArray & diagnostics)
{
  reset_route_epoch_if_clock_rewound();
  const auto status = std::find_if(
    diagnostics.status.begin(), diagnostics.status.end(), [](const auto & item) {
      return item.name == "jetpilot_planning/route_lane_selector";
    });
  if (status == diagnostics.status.end()) return;

  const auto ready = diagnostic_value(*status, "ready");
  const auto selected_lane = diagnostic_value(*status, "selected_lane");
  const auto target_speed = diagnostic_value(*status, "target_speed_mps");
  const auto parsed_speed = target_speed ? finite_nonnegative_float(*target_speed) : std::nullopt;
  const auto stamp = route_stamp(diagnostics.header.stamp);
  auto & cycle = pending_route_cycles_[stamp];
  cycle.diagnostics_received = true;
  cycle.diagnostics_valid = ready.has_value() && selected_lane.has_value() &&
    parsed_speed.has_value() && (*ready == "true" || *ready == "false");
  cycle.ready = cycle.diagnostics_valid && *ready == "true" &&
    status->level == diagnostic_msgs::msg::DiagnosticStatus::OK;
  cycle.selected_lane = selected_lane.value_or("");
  cycle.target_speed_mps = parsed_speed.value_or(0.0F);
  cycle.updated_at = std::chrono::steady_clock::now();
  resolve_route_cycle(stamp);
  prune_pending_route_cycles();
}

void PlanningManagerNode::reset_route_epoch_if_clock_rewound()
{
  const auto current_stamp = route_stamp(now().to_msg());
  const bool before_transition = minimum_route_stamp_ && current_stamp < *minimum_route_stamp_;
  const bool before_completed_cycle =
    latest_completed_route_stamp_ && current_stamp < *latest_completed_route_stamp_;
  if (!before_transition && !before_completed_cycle) return;

  invalidate_route_bundle();
  // A rewind starts a new ROS-time epoch. Keeping the old transition barrier
  // would reject every valid selector cycle until /clock reached the old era.
  minimum_route_stamp_.reset();
  RCLCPP_WARN(
    get_logger(), "ROS clock moved backwards; cleared route state and started a new route epoch");
}

void PlanningManagerNode::resolve_route_cycle(const RouteStamp & stamp)
{
  auto cycle_iterator = pending_route_cycles_.find(stamp);
  if (cycle_iterator == pending_route_cycles_.end()) return;
  const auto & cycle = cycle_iterator->second;
  if (!cycle.path_received || !cycle.profile_received || !cycle.diagnostics_received) return;

  // Transient-local Path/Profile samples from the pre-reset epoch can arrive
  // after a /clock rewind. A selector cycle generated by this node's clock is
  // never legitimately newer than the manager's current ROS time.
  if (stamp > route_stamp(now().to_msg()))
  {
    pending_route_cycles_.erase(cycle_iterator);
    return;
  }
  if (minimum_route_stamp_ && stamp <= *minimum_route_stamp_)
  {
    pending_route_cycles_.erase(cycle_iterator);
    return;
  }
  if (latest_completed_route_stamp_ && stamp < *latest_completed_route_stamp_)
  {
    pending_route_cycles_.erase(cycle_iterator);
    return;
  }
  latest_completed_route_stamp_ = stamp;
  const bool lane_matches = cycle.selected_lane == desired_lane_id();
  const bool has_payload = cycle.profile.has_value() || cycle.path.has_value();
  if (!cycle.diagnostics_valid || !cycle.ready || !lane_matches || !has_payload)
  {
    route_path_.reset();
    route_profile_.reset();
    route_ready_ = false;
    route_target_speed_ = 0.0F;
    selected_lane_.clear();
    route_bundle_received_at_.reset();
    publish_ready(false);
  }
  else
  {
    route_path_ = cycle.path;
    route_profile_ = cycle.profile;
    route_ready_ = true;
    route_target_speed_ = cycle.target_speed_mps;
    selected_lane_ = cycle.selected_lane;
    route_bundle_received_at_ = std::chrono::steady_clock::now();
  }
  pending_route_cycles_.erase(cycle_iterator);
}

void PlanningManagerNode::prune_pending_route_cycles()
{
  const auto now_steady = std::chrono::steady_clock::now();
  for (auto iterator = pending_route_cycles_.begin(); iterator != pending_route_cycles_.end();)
  {
    const auto age = std::chrono::duration<double>(now_steady - iterator->second.updated_at).count();
    if (age > route_bundle_timeout_s_)
      iterator = pending_route_cycles_.erase(iterator);
    else
      ++iterator;
  }
  constexpr std::size_t kMaximumPendingRouteCycles = 16U;
  while (pending_route_cycles_.size() > kMaximumPendingRouteCycles)
  {
    const auto oldest = std::min_element(
      pending_route_cycles_.begin(), pending_route_cycles_.end(),
      [](const auto & left, const auto & right) {
        return left.second.updated_at < right.second.updated_at;
      });
    pending_route_cycles_.erase(oldest);
  }
}

void PlanningManagerNode::invalidate_route_bundle(const bool reset_pending_cycles)
{
  route_path_.reset();
  route_profile_.reset();
  route_ready_ = false;
  route_target_speed_ = 0.0F;
  selected_lane_.clear();
  route_bundle_received_at_.reset();
  minimum_route_stamp_ = route_stamp(now().to_msg());
  if (reset_pending_cycles)
  {
    pending_route_cycles_.clear();
    latest_completed_route_stamp_.reset();
  }
  std_msgs::msg::Float32 speed;
  speed.data = 0.0F;
  target_speed_pub_->publish(speed);
  publish_ready(false);
}

void PlanningManagerNode::clear_committed_lane()
{
  if (committed_lane_id_.empty()) return;
  committed_lane_id_.clear();
  invalidate_route_bundle();
}

std::string PlanningManagerNode::desired_lane_id() const
{
  return committed_lane_id_.empty() ? default_lane_id_ : committed_lane_id_;
}

void PlanningManagerNode::on_collision(bool detected)
{
  const bool rising_edge = detected && !collision_input_;
  collision_input_ = detected;
  if (!detected && recovery_failed_)
  {
    collision_active_ = false;
    recovery_failed_ = false;
    return;
  }
  if (rising_edge && !collision_active_)
  {
    collision_active_ = true;
    recovery_failed_ = false;
  }
}

void PlanningManagerNode::on_recovery_status(const jetpilot_msgs::msg::RecoveryStatus & status)
{
  if (!collision_active_) return;
  if (status.state == jetpilot_msgs::msg::RecoveryStatus::SUCCEEDED)
  {
    collision_active_ = false;
    recovery_failed_ = false;
  }
  else if (status.state == jetpilot_msgs::msg::RecoveryStatus::FAILED)
  {
    recovery_failed_ = true;
  }
}

nav_msgs::msg::Path PlanningManagerNode::path_from_profile(
  const jetpilot_msgs::msg::Trajectory & profile) const
{
  nav_msgs::msg::Path path;
  path.header = profile.header;
  for (const auto & point : profile.points)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = point.pose;
    path.poses.push_back(pose);
  }
  return path;
}

void PlanningManagerNode::publish_ready(bool ready)
{
  std_msgs::msg::Bool message;
  message.data = ready;
  ready_pub_->publish(message);
}

void PlanningManagerNode::publish_status(std::uint8_t state, const std::string & reason)
{
  jetpilot_msgs::msg::PlanningManagerStatus message;
  message.header.stamp = now();
  message.state = state;
  message.active_lane_id = committed_lane_id_.empty() ? selected_lane_ : committed_lane_id_;
  message.active_junction_id = active_junction_id_;
  message.reason = reason;
  status_pub_->publish(message);
}

void PlanningManagerNode::publish_cycle()
{
  prune_pending_route_cycles();
  std_msgs::msg::String lane_request;
  lane_request.data = committed_lane_id_;
  requested_lane_pub_->publish(lane_request);
  std_msgs::msg::Bool recovery_request;
  recovery_request.data = collision_active_;
  recovery_request_pub_->publish(recovery_request);

  if (collision_active_)
  {
    if (recovery_failed_)
    {
      publish_ready(false);
      publish_status(jetpilot_msgs::msg::PlanningManagerStatus::FAULT, "recovery_failed");
      return;
    }
    if (!recovery_ready_ || !recovery_profile_)
    {
      publish_ready(false);
      publish_status(jetpilot_msgs::msg::PlanningManagerStatus::RECOVERY, "waiting_recovery_path");
      return;
    }
    auto profile = *recovery_profile_;
    profile.header.stamp = now();
    auto path = path_from_profile(profile);
    profile_pub_->publish(profile);
    path_pub_->publish(path);
    std_msgs::msg::Float32 speed;
    speed.data = static_cast<float>(recovery_target_speed_mps_);
    target_speed_pub_->publish(speed);
    publish_ready(true);
    publish_status(jetpilot_msgs::msg::PlanningManagerStatus::RECOVERY, "reversing_breadcrumb");
    return;
  }
  const bool section_stale = !current_section_received_at_ ||
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *current_section_received_at_).count() >
      current_section_timeout_s_;
  if ((require_junction_map_ && !junction_map_received_) || section_stale ||
      current_section_.empty() || current_section_ == "unknown")
  {
    publish_ready(false);
    publish_status(jetpilot_msgs::msg::PlanningManagerStatus::FAULT,
                   !junction_map_received_ ? "waiting_junction_map" :
                   (section_stale ? "current_section_stale" : "current_section_unknown"));
    return;
  }
  if (active_junction_ && committed_lane_id_.empty())
  {
    publish_ready(false);
    publish_status(jetpilot_msgs::msg::PlanningManagerStatus::WAIT_SIGNAL, "waiting_stable_signal");
    return;
  }
  const bool route_bundle_stale = !route_bundle_received_at_ ||
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *route_bundle_received_at_).count() >
    route_bundle_timeout_s_;
  if (!route_ready_ || route_bundle_stale || selected_lane_ != desired_lane_id() ||
      (!route_path_ && !route_profile_))
  {
    if (route_bundle_stale && route_ready_) invalidate_route_bundle(false);
    publish_ready(false);
    publish_status(
      jetpilot_msgs::msg::PlanningManagerStatus::FOLLOW_ROUTE,
      route_bundle_stale ? "route_bundle_stale" :
      (selected_lane_ != desired_lane_id() ? "route_lane_mismatch" : "route_not_ready"));
    return;
  }
  if (route_profile_)
  {
    auto profile = *route_profile_;
    profile.header.stamp = now();
    profile_pub_->publish(profile);
    path_pub_->publish(path_from_profile(profile));
  }
  else
  {
    auto path = *route_path_;
    path.header.stamp = now();
    path_pub_->publish(path);
    jetpilot_msgs::msg::Trajectory empty_profile;
    empty_profile.header.stamp = now();
    profile_pub_->publish(empty_profile);
  }
  std_msgs::msg::Float32 speed;
  speed.data = route_target_speed_;
  target_speed_pub_->publish(speed);
  publish_ready(true);
  publish_status(active_junction_ ? jetpilot_msgs::msg::PlanningManagerStatus::ROUTE_COMMITTED :
                 jetpilot_msgs::msg::PlanningManagerStatus::FOLLOW_ROUTE,
                 active_junction_ ? "signal_route_committed" : "following_route");
}

}  // namespace jetpilot_planning_manager
