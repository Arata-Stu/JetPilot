#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include "jetpilot_localization_manager/localization_manager_node.hpp"

namespace jetpilot_localization_manager
{

LocalizationManagerNode::LocalizationManagerNode()
    : Node("jetpilot_localization_manager"), steady_now_(std::chrono::steady_clock::now())
{
  use_vgl_ = declare_parameter<bool>("use_vgl", false);
  autostart_ = declare_parameter<bool>("autostart", false);
  wait_for_vslam_subscriber_ = declare_parameter<bool>("wait_for_vslam_subscriber", true);
  wait_for_vslam_diagnostics_ = declare_parameter<bool>("wait_for_vslam_diagnostics", true);
  dependency_wait_timeout_sec_ = nonnegative_parameter("dependency_wait_timeout_sec", 30.0);
  vgl_response_timeout_sec_ = nonnegative_parameter("vgl_response_timeout_sec", 30.0);
  vslam_confirmation_timeout_sec_ = nonnegative_parameter("vslam_confirmation_timeout_sec", 60.0);
  retry_backoff_sec_ = nonnegative_parameter("retry_backoff_sec", 1.0);
  const auto max_attempts_parameter =
    std::max<std::int64_t>(0, declare_parameter<std::int64_t>("max_attempts", 3));
  max_attempts_ = static_cast<int>(
    std::min<std::int64_t>(max_attempts_parameter, std::numeric_limits<int>::max()));
  poll_period_sec_ = std::max(0.01, nonnegative_parameter("poll_period_sec", 0.1));
  status_publish_period_sec_ =
    std::max(0.05, nonnegative_parameter("status_publish_period_sec", 0.5));

  validation_options_.expected_frame_id =
    declare_parameter<std::string>("expected_pose_frame", "map");
  validation_options_.quaternion_norm_tolerance =
    nonnegative_parameter("quaternion_norm_tolerance", 0.1);
  validation_options_.max_pose_age_sec = nonnegative_parameter("max_pose_age_sec", 0.0);

  vslam_hint_request_topic_ =
    declare_parameter<std::string>("vslam_hint_request_topic", "/visual_slam/trigger_hint");
  vslam_pose_hint_topic_ =
    declare_parameter<std::string>("vslam_pose_hint_topic", "/localization/pose_hint");
  manual_pose_topic_ = declare_parameter<std::string>("manual_pose_topic", "/initialpose");
  vgl_trigger_service_ = declare_parameter<std::string>(
    "vgl_trigger_service", "/visual_localization/trigger_localization");
  vgl_pose_topic_ = declare_parameter<std::string>("vgl_pose_topic", "/visual_localization/pose");
  localization_trigger_topic_ =
    declare_parameter<std::string>("localization_trigger_topic", "/localization/trigger");
  localization_trigger_service_ =
    declare_parameter<std::string>("localization_trigger_service", "/localization/relocalize");
  diagnostics_topic_ = declare_parameter<std::string>(
    "diagnostics_topic", "/localization/vslam/diagnostics");
  vslam_diagnostics_hardware_id_ =
    declare_parameter<std::string>("vslam_diagnostics_hardware_id", "visual_slam");
  vslam_localized_key_ =
    declare_parameter<std::string>("vslam_localized_key", "localized_in_exist_map");
  pose_hint_required_topic_ =
    declare_parameter<std::string>("pose_hint_required_topic", "/localization/pose_hint_required");
  pose_hint_state_topic_ =
    declare_parameter<std::string>("pose_hint_state_topic", "/localization/pose_hint_state");

  const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  pose_hint_required_pub_ =
    create_publisher<std_msgs::msg::Bool>(pose_hint_required_topic_, status_qos);
  pose_hint_state_pub_ =
    create_publisher<std_msgs::msg::String>(pose_hint_state_topic_, status_qos);
  vslam_pose_hint_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    vslam_pose_hint_topic_, rclcpp::QoS(10).reliable());

  vgl_trigger_client_ = create_client<std_srvs::srv::Trigger>(vgl_trigger_service_);
  relocalize_service_ = create_service<std_srvs::srv::Trigger>(
    localization_trigger_service_, std::bind(&LocalizationManagerNode::on_relocalize_service, this,
                                             std::placeholders::_1, std::placeholders::_2));

  vslam_hint_request_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    vslam_hint_request_topic_, 10,
    std::bind(&LocalizationManagerNode::on_vslam_hint_request, this, std::placeholders::_1));
  manual_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    manual_pose_topic_, 10,
    std::bind(&LocalizationManagerNode::on_manual_pose, this, std::placeholders::_1));
  vgl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    vgl_pose_topic_, 10,
    std::bind(&LocalizationManagerNode::on_vgl_pose, this, std::placeholders::_1));
  localization_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
    localization_trigger_topic_, 10,
    std::bind(&LocalizationManagerNode::on_localization_trigger, this, std::placeholders::_1));
  diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_, 10,
    std::bind(&LocalizationManagerNode::on_diagnostics, this, std::placeholders::_1));

  tick_timer_ = create_wall_timer(std::chrono::duration<double>(poll_period_sec_),
                                  std::bind(&LocalizationManagerNode::on_tick, this));
  last_status_publish_ = SteadyClock::now();
  publish_status();

  RCLCPP_INFO(get_logger(),
              "Localization manager started (use_vgl=%s, autostart=%s, "
              "max_attempts=%d)",
              use_vgl_ ? "true" : "false", autostart_ ? "true" : "false", max_attempts_);

  if (autostart_)
  {
    begin_localization("autostart");
  }
}

double LocalizationManagerNode::nonnegative_parameter(const std::string & name,
                                                      double default_value)
{
  const double value = declare_parameter<double>(name, default_value);
  if (!std::isfinite(value) || value < 0.0)
  {
    RCLCPP_WARN(get_logger(), "%s must be finite and non-negative; using 0", name.c_str());
    return 0.0;
  }
  return value;
}

const char * LocalizationManagerNode::state_name(State state)
{
  switch (state)
  {
    case State::kIdle:
      return "idle";
    case State::kWaitingForDependencies:
      return "waiting_for_dependencies";
    case State::kWaitingForTriggerResponse:
      return "waiting_for_vgl_trigger_response";
    case State::kWaitingForVglPose:
      return "waiting_for_vgl_pose";
    case State::kRetryBackoff:
      return "retry_backoff";
    case State::kWaitingToForwardPose:
      return "waiting_for_vslam_ready";
    case State::kAwaitingVslam:
      return "awaiting_vslam";
    case State::kLocalized:
      return "localized";
    case State::kLocalizedUnconfirmed:
      return "localized_unconfirmed";
    case State::kWaitingForManual:
      return "waiting_for_manual";
  }
  return "unknown";
}

std::string LocalizationManagerNode::json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value)
  {
    switch (character)
    {
      case '\"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20)
        {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        }
        else
        {
          output << character;
        }
    }
  }
  return output.str();
}

bool LocalizationManagerNode::deadline_expired() const
{
  return deadline_.has_value() && steady_now_ >= deadline_.value();
}

void LocalizationManagerNode::set_deadline(double timeout_sec)
{
  if (timeout_sec <= 0.0)
  {
    deadline_.reset();
    return;
  }
  deadline_ = steady_now_ + std::chrono::duration_cast<SteadyClock::duration>(
                              std::chrono::duration<double>(timeout_sec));
}

bool LocalizationManagerNode::vslam_subscriber_ready() const
{
  return vslam_pose_hint_pub_->get_subscription_count() > 0;
}

bool LocalizationManagerNode::can_forward_pose() const
{
  const bool subscriber_ready = !wait_for_vslam_subscriber_ || vslam_subscriber_ready();
  const bool diagnostics_ready = !wait_for_vslam_diagnostics_ || saw_vslam_diagnostics_;
  return subscriber_ready && diagnostics_ready;
}

void LocalizationManagerNode::transition(State state, std::string reason)
{
  steady_now_ = SteadyClock::now();
  state_ = state;
  reason_ = std::move(reason);
  publish_status();
}

void LocalizationManagerNode::begin_localization(const std::string & source)
{
  clear_pending_vgl_request();
  ++request_generation_;
  attempts_ = 0;
  pending_pose_.reset();
  request_source_ = source;
  pose_hint_required_ = true;

  if (!use_vgl_)
  {
    wait_for_manual("vgl_disabled");
    return;
  }
  start_attempt();
}

void LocalizationManagerNode::start_attempt()
{
  if (max_attempts_ > 0 && attempts_ >= max_attempts_)
  {
    wait_for_manual("max_attempts_exhausted");
    return;
  }

  ++attempts_;
  ++request_generation_;
  transition(State::kWaitingForDependencies, "waiting_for_vgl_service");
  set_deadline(dependency_wait_timeout_sec_);
  if (max_attempts_ > 0)
  {
    RCLCPP_INFO(get_logger(), "Starting VGL localization attempt %d/%d (source=%s)", attempts_,
                max_attempts_, request_source_.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Starting VGL localization attempt %d (source=%s, unlimited retries)",
                attempts_, request_source_.c_str());
  }
}

void LocalizationManagerNode::request_vgl()
{
  const auto generation = request_generation_;
  transition(State::kWaitingForTriggerResponse, "waiting_for_vgl_trigger_response");
  set_deadline(vgl_response_timeout_sec_);

  try
  {
    auto request = std::make_shared<Trigger::Request>();
    const auto future_and_request_id = vgl_trigger_client_->async_send_request(
      request, [this, generation](rclcpp::Client<Trigger>::SharedFuture future)
      { on_vgl_trigger_response(future, generation); });
    pending_vgl_request_id_ = future_and_request_id.request_id;
    RCLCPP_INFO(get_logger(), "Requested VGL through %s", vgl_trigger_service_.c_str());
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(get_logger(), "Failed to call VGL trigger service: %s", error.what());
    schedule_retry("vgl_trigger_call_failed");
  }
}

void LocalizationManagerNode::on_vgl_trigger_response(
  const rclcpp::Client<Trigger>::SharedFuture & future, std::uint64_t generation)
{
  if (generation != request_generation_ || state_ != State::kWaitingForTriggerResponse)
  {
    return;
  }
  pending_vgl_request_id_.reset();

  Trigger::Response::SharedPtr response;
  try
  {
    response = future.get();
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(get_logger(), "VGL trigger service failed: %s", error.what());
    schedule_retry("vgl_trigger_service_failed");
    return;
  }
  if (!response->success)
  {
    RCLCPP_WARN(get_logger(), "VGL rejected trigger: %s", response->message.c_str());
    schedule_retry("vgl_trigger_rejected");
    return;
  }
  transition(State::kWaitingForVglPose, "waiting_for_vgl_pose");
  set_deadline(vgl_response_timeout_sec_);
}

void LocalizationManagerNode::schedule_retry(const std::string & reason)
{
  clear_pending_vgl_request();
  ++request_generation_;  // Invalidate any outstanding service response.
  pending_pose_.reset();
  pose_hint_required_ = true;
  if (max_attempts_ > 0 && attempts_ >= max_attempts_)
  {
    wait_for_manual("max_attempts_exhausted:" + reason);
    return;
  }
  transition(State::kRetryBackoff, reason);
  RCLCPP_WARN(get_logger(), "Localization attempt failed (%s); retrying after %.2f s",
              reason.c_str(), retry_backoff_sec_);
  if (retry_backoff_sec_ <= 0.0)
  {
    start_attempt();
    return;
  }
  set_deadline(retry_backoff_sec_);
}

void LocalizationManagerNode::wait_for_manual(const std::string & reason)
{
  clear_pending_vgl_request();
  ++request_generation_;
  deadline_.reset();
  pending_pose_.reset();
  pose_hint_required_ = true;
  transition(State::kWaitingForManual, reason);
  RCLCPP_WARN(get_logger(), "Waiting for a manual pose on %s (%s)", manual_pose_topic_.c_str(),
              reason.c_str());
}

void LocalizationManagerNode::queue_pose(const Pose & pose, const std::string & source)
{
  pending_pose_ = pose;
  pending_pose_source_ = source;
  pose_hint_required_ = true;
  if (can_forward_pose())
  {
    forward_pending_pose();
    return;
  }
  transition(State::kWaitingToForwardPose, "waiting_for_vslam_ready");
  set_deadline(dependency_wait_timeout_sec_);
  RCLCPP_WARN(get_logger(), "Holding %s pose until VSLAM is ready on %s", source.c_str(),
              vslam_pose_hint_topic_.c_str());
}

void LocalizationManagerNode::forward_pending_pose()
{
  if (!pending_pose_.has_value() || !can_forward_pose())
  {
    return;
  }

  vslam_pose_hint_pub_->publish(pending_pose_.value());
  last_hint_source_ = pending_pose_source_;
  pending_pose_.reset();
  pose_hint_required_ = false;
  saw_not_localized_since_hint_ =
    last_vslam_localized_.has_value() && !last_vslam_localized_.value();
  transition(State::kAwaitingVslam, "pose_hint_sent");
  set_deadline(vslam_confirmation_timeout_sec_);
  RCLCPP_INFO(get_logger(), "Forwarded %s pose hint to %s", last_hint_source_.c_str(),
              vslam_pose_hint_topic_.c_str());
}

void LocalizationManagerNode::on_vslam_hint_request(const Pose::SharedPtr /* message */)
{
  request_source_ = "vslam";
  pose_hint_required_ = true;
  RCLCPP_WARN(get_logger(), "VSLAM requested another localization hint");

  if (!use_vgl_)
  {
    wait_for_manual("vslam_requested_hint_vgl_disabled");
    return;
  }

  if (state_ == State::kIdle || state_ == State::kLocalized ||
      state_ == State::kLocalizedUnconfirmed)
  {
    begin_localization("vslam");
    return;
  }
  if (state_ == State::kWaitingForManual)
  {
    reason_ = "vslam_requested_hint_while_waiting_for_manual";
    publish_status();
    return;
  }
  schedule_retry("vslam_requested_another_hint");
}

void LocalizationManagerNode::on_manual_pose(const Pose::SharedPtr message)
{
  const auto validation = validate_pose(*message, validation_options_, now().nanoseconds());
  if (!validation.valid)
  {
    reason_ = "invalid_manual_pose:" + validation.reason;
    publish_status();
    RCLCPP_WARN(get_logger(), "Rejected manual pose: %s", validation.reason.c_str());
    return;
  }

  clear_pending_vgl_request();
  ++request_generation_;
  attempts_ = 0;
  request_source_ = "manual";
  queue_pose(validation.pose, "manual");
}

void LocalizationManagerNode::on_vgl_pose(const Pose::SharedPtr message)
{
  const bool waiting_for_vgl =
    state_ == State::kWaitingForTriggerResponse || state_ == State::kWaitingForVglPose;
  const bool accepting_late_vgl =
    state_ == State::kWaitingForManual && use_vgl_ && request_source_ != "manual";
  if (!waiting_for_vgl && !accepting_late_vgl)
  {
    RCLCPP_DEBUG(get_logger(), "Ignoring unsolicited VGL pose");
    return;
  }

  const auto validation = validate_pose(*message, validation_options_, now().nanoseconds());
  if (!validation.valid)
  {
    RCLCPP_WARN(get_logger(), "Rejected VGL pose: %s", validation.reason.c_str());
    schedule_retry("invalid_vgl_pose:" + validation.reason);
    return;
  }
  clear_pending_vgl_request();
  queue_pose(validation.pose, "vgl");
}

void LocalizationManagerNode::clear_pending_vgl_request()
{
  if (!pending_vgl_request_id_.has_value())
  {
    return;
  }
  vgl_trigger_client_->remove_pending_request(pending_vgl_request_id_.value());
  pending_vgl_request_id_.reset();
}

void LocalizationManagerNode::on_localization_trigger(const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message->data)
  {
    return;
  }
  begin_localization("topic");
}

void LocalizationManagerNode::on_relocalize_service(const Trigger::Request::SharedPtr /* request */,
                                                    Trigger::Response::SharedPtr response)
{
  begin_localization("service");
  response->success = use_vgl_;
  response->message =
    use_vgl_ ? "Localization request accepted" : "VGL is disabled; waiting for /initialpose";
}

void LocalizationManagerNode::on_diagnostics(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
{
  std::optional<bool> localized;
  for (const auto & status : message->status)
  {
    if (status.hardware_id != vslam_diagnostics_hardware_id_)
    {
      continue;
    }
    for (const auto & value : status.values)
    {
      if (value.key == vslam_localized_key_)
      {
        if (value.value == "Yes")
        {
          localized = true;
        }
        else if (value.value == "No")
        {
          localized = false;
        }
      }
    }
  }
  if (!localized.has_value())
  {
    return;
  }

  saw_vslam_diagnostics_ = true;
  last_vslam_localized_ = localized;
  if (state_ != State::kAwaitingVslam)
  {
    publish_status();
    return;
  }

  if (!localized.value())
  {
    saw_not_localized_since_hint_ = true;
    reason_ = "awaiting_vslam_localization";
    publish_status();
    return;
  }

  // cuVSLAM may keep publishing an old Yes while a new localization request
  // is in flight. Only a No -> Yes transition after/baseline-before this hint
  // is strong enough to call the new request successful.
  if (saw_not_localized_since_hint_)
  {
    pose_hint_required_ = false;
    deadline_.reset();
    transition(State::kLocalized, "vslam_confirmed_localized");
    RCLCPP_INFO(get_logger(), "VSLAM confirmed localization in the saved map");
  }
  else
  {
    reason_ = "vslam_reports_localized_without_current_request_transition";
    publish_status();
  }
}

void LocalizationManagerNode::on_tick()
{
  steady_now_ = SteadyClock::now();

  switch (state_)
  {
    case State::kWaitingForDependencies:
    {
      const bool service_ready = vgl_trigger_client_->service_is_ready();
      if (service_ready && can_forward_pose())
      {
        request_vgl();
      }
      else if (deadline_expired())
      {
        schedule_retry(service_ready ? "vslam_readiness_timeout" : "vgl_service_timeout");
      }
      else
      {
        if (!service_ready)
        {
          reason_ = "waiting_for_vgl_service";
        }
        else if (wait_for_vslam_subscriber_ && !vslam_subscriber_ready())
        {
          reason_ = "waiting_for_vslam_pose_hint_subscriber";
        }
        else
        {
          reason_ = "waiting_for_vslam_diagnostics";
        }
      }
      break;
    }
    case State::kWaitingForTriggerResponse:
      if (deadline_expired())
      {
        schedule_retry("vgl_trigger_response_timeout");
      }
      break;
    case State::kWaitingForVglPose:
      if (deadline_expired())
      {
        schedule_retry("vgl_pose_timeout");
      }
      break;
    case State::kRetryBackoff:
      if (deadline_expired())
      {
        start_attempt();
      }
      break;
    case State::kWaitingToForwardPose:
      if (can_forward_pose())
      {
        forward_pending_pose();
      }
      else if (deadline_expired())
      {
        if (use_vgl_ && pending_pose_source_ == "vgl")
        {
          schedule_retry("vslam_readiness_timeout");
        }
        else
        {
          wait_for_manual("vslam_readiness_timeout");
        }
      }
      break;
    case State::kAwaitingVslam:
      if (deadline_expired())
      {
        if (last_vslam_localized_.value_or(false))
        {
          pose_hint_required_ = false;
          transition(State::kLocalizedUnconfirmed,
                     "vslam_localized_without_current_request_transition");
        }
        else if (use_vgl_ && last_hint_source_ == "vgl")
        {
          schedule_retry("vslam_confirmation_timeout");
        }
        else
        {
          wait_for_manual("vslam_confirmation_timeout");
        }
      }
      break;
    default:
      break;
  }

  if (steady_now_ - last_status_publish_ >=
      std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(status_publish_period_sec_)))
  {
    publish_status();
  }
}

void LocalizationManagerNode::publish_status()
{
  std_msgs::msg::Bool required_message;
  required_message.data = pose_hint_required_;
  pose_hint_required_pub_->publish(required_message);

  std::ostringstream json;
  json << "{"
       << "\"state\":\"" << json_escape(state_name(state_)) << "\","
       << "\"pose_hint_required\":" << (pose_hint_required_ ? "true" : "false") << ","
       << "\"request_source\":\"" << json_escape(request_source_) << "\","
       << "\"use_vgl\":" << (use_vgl_ ? "true" : "false") << ","
       << "\"autostart\":" << (autostart_ ? "true" : "false") << ","
       << "\"vgl_available\":" << (vgl_trigger_client_->service_is_ready() ? "true" : "false")
       << ","
       << "\"vgl_service_ready\":" << (vgl_trigger_client_->service_is_ready() ? "true" : "false")
       << ","
       << "\"vslam_hint_subscriber_ready\":" << (vslam_subscriber_ready() ? "true" : "false") << ","
       << "\"vslam_diagnostics_received\":" << (saw_vslam_diagnostics_ ? "true" : "false") << ","
       << "\"vslam_ready\":" << (can_forward_pose() ? "true" : "false") << ","
       << "\"attempt\":" << attempts_ << ","
       << "\"max_attempts\":" << max_attempts_ << ","
       << "\"last_hint_source\":\"" << json_escape(last_hint_source_) << "\","
       << "\"reason\":\"" << json_escape(reason_) << "\""
       << "}";
  std_msgs::msg::String state_message;
  state_message.data = json.str();
  pose_hint_state_pub_->publish(state_message);
  last_status_publish_ = SteadyClock::now();
}

}  // namespace jetpilot_localization_manager
