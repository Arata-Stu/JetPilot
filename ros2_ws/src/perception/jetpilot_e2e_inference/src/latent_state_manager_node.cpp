#include "jetpilot_e2e_inference/latent_state_manager_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "cuda_runtime_api.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_builder.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace jetpilot_e2e_inference
{
namespace
{

double milliseconds(const std::uint64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1.0e6;
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

LatentStateManagerNode::LatentStateManagerNode(const rclcpp::NodeOptions & options)
: Node("latent_state_manager", options)
{
  rgb_latent_tensor_name_ =
    declare_parameter<std::string>("rgb_latent_tensor_name", "rgb_latent");
  state_input_tensor_name_ =
    declare_parameter<std::string>("state_input_tensor_name", "state_in");
  state_output_tensor_name_ =
    declare_parameter<std::string>("state_output_tensor_name", "state_out");
  event_source_tensor_name_ =
    declare_parameter<std::string>("event_source_tensor_name", "event_tensor");
  event_input_tensor_name_ =
    declare_parameter<std::string>("event_input_tensor_name", "event_tensor");
  delta_t_tensor_name_ = declare_parameter<std::string>("delta_t_tensor_name", "delta_t");
  include_delta_t_ = declare_parameter<bool>("include_delta_t", true);
  initial_delta_t_s_ = declare_parameter<double>("initial_delta_t_s", 0.001);
  max_delta_t_s_ = declare_parameter<double>("max_delta_t_s", 0.1);
  inference_watchdog_ms_ = declare_parameter<double>("inference_watchdog_ms", 100.0);
  statistics_interval_s_ = declare_parameter<double>("statistics_interval_s", 1.0);
  debug_ = declare_parameter<bool>("debug", false);
  const auto queue_depth = declare_parameter<std::int64_t>("queue_depth", 4);
  const auto diagnostics_topic = declare_parameter<std::string>(
    "diagnostics_topic", "/e2e/latent_state/diagnostics");

  if (
    rgb_latent_tensor_name_.empty() || state_input_tensor_name_.empty() ||
    state_output_tensor_name_.empty() || event_source_tensor_name_.empty() ||
    event_input_tensor_name_.empty() || (include_delta_t_ && delta_t_tensor_name_.empty()))
  {
    throw std::invalid_argument("latent state tensor names must not be empty");
  }
  if (
    queue_depth <= 0 || !std::isfinite(initial_delta_t_s_) || initial_delta_t_s_ <= 0.0 ||
    !std::isfinite(max_delta_t_s_) || max_delta_t_s_ < initial_delta_t_s_ ||
    !std::isfinite(inference_watchdog_ms_) || inference_watchdog_ms_ <= 0.0 ||
    !std::isfinite(statistics_interval_s_) || statistics_interval_s_ < 0.0)
  {
    throw std::invalid_argument("invalid latent state timing or queue parameter");
  }

  cuda_stream_ = nvidia::isaac_ros::common::createCudaStream("LatentStateManagerNode");
  if (include_delta_t_) {
    const auto cuda_status = scalar_memory_pool_.create(
      sizeof(float), 4U,
      nvidia::isaac_ros::nitros::CUDAMemoryPool::MemoryType::Device);
    if (cuda_status != cudaSuccess) {
      throw std::runtime_error(
              std::string("failed to create delta_t CUDA pool: ") +
              cudaGetErrorString(cuda_status));
    }
  }

  const auto depth = static_cast<std::size_t>(queue_depth);
  const auto reliable_qos =
    rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  rclcpp::PublisherOptions publisher_options;
  publisher_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  rgb_latent_subscription_ = create_subscription<TensorList>(
    "rgb_latent", reliable_qos,
    std::bind(&LatentStateManagerNode::on_rgb_latent, this, std::placeholders::_1),
    subscription_options);
  event_tensor_subscription_ = create_subscription<TensorList>(
    "event_tensor", reliable_qos,
    std::bind(&LatentStateManagerNode::on_event_tensor, this, std::placeholders::_1),
    subscription_options);
  updater_output_subscription_ = create_subscription<TensorList>(
    "updater_output", reliable_qos,
    std::bind(&LatentStateManagerNode::on_updater_output, this, std::placeholders::_1),
    subscription_options);
  updater_input_publisher_ =
    create_publisher<TensorList>("updater_input", reliable_qos, publisher_options);
  output_publisher_ = create_publisher<TensorList>("output", reliable_qos, publisher_options);
  event_feedback_publisher_ =
    create_publisher<TensorList>("event_feedback", reliable_qos, publisher_options);
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);

  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(std::max<std::int64_t>(
        1, static_cast<std::int64_t>(inference_watchdog_ms_ / 2.0))),
    std::bind(&LatentStateManagerNode::on_watchdog, this));
  if (statistics_interval_s_ > 0.0) {
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max<std::int64_t>(
          1, static_cast<std::int64_t>(statistics_interval_s_ * 1000.0))),
      std::bind(&LatentStateManagerNode::publish_diagnostics, this));
  }

  RCLCPP_INFO(
    get_logger(),
    "Latent state manager: RGB='%s', state='%s'->'%s', event='%s'->'%s', delta_t=%s",
    rgb_latent_tensor_name_.c_str(), state_output_tensor_name_.c_str(),
    state_input_tensor_name_.c_str(), event_source_tensor_name_.c_str(),
    event_input_tensor_name_.c_str(), include_delta_t_ ? "enabled" : "disabled");
}

std::int64_t LatentStateManagerNode::timestamp_ns(const TensorList & message)
{
  return static_cast<std::int64_t>(message.get_timestamp_sec()) * 1000000000LL +
         static_cast<std::int64_t>(message.get_timestamp_nsec());
}

void LatentStateManagerNode::on_rgb_latent(TensorList::ConstSharedPtr message)
{
  ++rgb_received_;
  if (!message || !message->get_tensor_by_name(rgb_latent_tensor_name_)) {
    ++invalid_message_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "RGB latent tensor '%s' was not found",
      rgb_latent_tensor_name_.c_str());
    return;
  }
  const auto stamp = timestamp_ns(*message);
  if (stamp <= last_rgb_timestamp_ns_) {
    ++rgb_stale_;
    return;
  }
  last_rgb_timestamp_ns_ = stamp;
  if (pending_rgb_message_) {
    ++rgb_replaced_;
  }
  pending_rgb_message_ = std::move(message);
  try_dispatch();
}

void LatentStateManagerNode::on_event_tensor(TensorList::ConstSharedPtr message)
{
  ++event_received_;
  if (!message || !message->get_tensor_by_name(event_source_tensor_name_)) {
    ++invalid_message_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Event tensor '%s' was not found",
      event_source_tensor_name_.c_str());
    if (message) {
      event_feedback_publisher_->publish(*message);
    }
    return;
  }
  if (pending_event_message_) {
    ++event_replaced_;
  }
  pending_event_message_ = std::move(message);
  try_dispatch();
}

void LatentStateManagerNode::try_dispatch()
{
  if (inference_in_flight_) {
    return;
  }

  if (pending_rgb_message_) {
    current_state_message_ = std::move(pending_rgb_message_);
    current_state_tensor_name_ = rgb_latent_tensor_name_;
    current_state_timestamp_ns_ = timestamp_ns(*current_state_message_);
    ++rgb_applied_;
  }
  if (!current_state_message_ || !pending_event_message_) {
    return;
  }

  const auto event_timestamp = timestamp_ns(*pending_event_message_);
  if (event_timestamp <= current_state_timestamp_ns_) {
    event_feedback_publisher_->publish(*pending_event_message_);
    pending_event_message_.reset();
    ++event_stale_;
    return;
  }

  const auto state_tensor =
    current_state_message_->get_tensor_by_name(current_state_tensor_name_);
  const auto event_tensor =
    pending_event_message_->get_tensor_by_name(event_source_tensor_name_);
  if (!state_tensor || !event_tensor) {
    event_feedback_publisher_->publish(*pending_event_message_);
    pending_event_message_.reset();
    ++invalid_message_;
    return;
  }

  const auto dispatch_start = std::chrono::steady_clock::now();
  const auto dispatched_event_message = pending_event_message_;
  try {
    nvidia::isaac_ros::nitros::NitrosTensorListBuilder builder;
    builder.WithHeader(pending_event_message_->get_header());
    builder.AddTensor(state_input_tensor_name_, *state_tensor);
    builder.AddTensor(event_input_tensor_name_, *event_tensor);

    if (include_delta_t_) {
      const auto raw_delta_t = current_state_timestamp_ns_ > 0 ?
        static_cast<double>(event_timestamp - current_state_timestamp_ns_) / 1.0e9 :
        initial_delta_t_s_;
      delta_t_staging_ = static_cast<float>(
        std::clamp(raw_delta_t, initial_delta_t_s_, max_delta_t_s_));
      nvidia::isaac_ros::nitros::NitrosTensor delta_t_tensor;
      {
        // The online updater is exported with an explicit NC scalar input.
        const nvidia::isaac_ros::nitros::NitrosTensorShape shape{1, 1};
        auto write_handle = delta_t_tensor.from_pool(
          delta_t_tensor_name_, scalar_memory_pool_, shape,
          nvidia::isaac_ros::nitros::NitrosDataType::kFloat32, *cuda_stream_);
        const auto cuda_status = cudaMemcpyAsync(
          write_handle.get_ptr(), &delta_t_staging_, sizeof(float),
          cudaMemcpyHostToDevice, *cuda_stream_);
        if (cuda_status != cudaSuccess) {
          throw std::runtime_error(
                  std::string("failed to copy delta_t to CUDA: ") +
                  cudaGetErrorString(cuda_status));
        }
      }
      builder.AddTensor(delta_t_tensor_name_, delta_t_tensor);
    }

    auto input = builder.Build();
    in_flight_timestamp_ns_ = event_timestamp;
    in_flight_started_ = std::chrono::steady_clock::now();
    inference_in_flight_ = true;
    watchdog_reported_ = false;
    pending_event_message_.reset();
    ++inference_dispatched_;
    updater_input_publisher_->publish(input);

    const auto dispatch_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - dispatch_start).count());
    dispatch_ns_sum_ += dispatch_ns;
    dispatch_ns_max_ = std::max(dispatch_ns_max_, dispatch_ns);
  } catch (const std::exception & error) {
    inference_in_flight_ = false;
    if (dispatched_event_message) {
      event_feedback_publisher_->publish(*dispatched_event_message);
    }
    pending_event_message_.reset();
    ++invalid_message_;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to assemble updater input: %s", error.what());
  }
}

void LatentStateManagerNode::on_updater_output(TensorList::ConstSharedPtr message)
{
  if (!message || !inference_in_flight_ || timestamp_ns(*message) != in_flight_timestamp_ns_) {
    ++stale_output_;
    return;
  }

  const auto round_trip_ns = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - in_flight_started_).count());
  round_trip_ns_sum_ += round_trip_ns;
  round_trip_ns_max_ = std::max(round_trip_ns_max_, round_trip_ns);
  ++inference_completed_;

  const auto next_state = message->get_tensor_by_name(state_output_tensor_name_);
  if (next_state) {
    // Retaining the complete NITROS output keeps its GPU allocation alive.  The
    // next TensorList only aliases this immutable buffer; TensorRT writes the
    // following state into a separate output buffer (GPU ping-pong).
    current_state_message_ = message;
    current_state_tensor_name_ = state_output_tensor_name_;
    current_state_timestamp_ns_ = in_flight_timestamp_ns_;
  } else {
    ++invalid_message_;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Updater output tensor '%s' was not found",
      state_output_tensor_name_.c_str());
  }

  inference_in_flight_ = false;
  watchdog_reported_ = false;
  event_feedback_publisher_->publish(*message);
  output_publisher_->publish(*message);

  if (debug_) {
    RCLCPP_INFO(
      get_logger(), "latent update: state=%lu, round_trip=%.3fms, pending_event=%s",
      static_cast<unsigned long>(inference_completed_), milliseconds(round_trip_ns),
      pending_event_message_ ? "yes" : "no");
  }
  try_dispatch();
}

void LatentStateManagerNode::on_watchdog()
{
  if (!inference_in_flight_ || watchdog_reported_) {
    return;
  }
  const auto elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - in_flight_started_).count();
  if (elapsed_ms >= inference_watchdog_ms_) {
    watchdog_reported_ = true;
    ++watchdog_count_;
    RCLCPP_ERROR(
      get_logger(),
      "Latent updater has not returned after %.3fms; keeping one-in-flight safety lock",
      elapsed_ms);
  }
}

void LatentStateManagerNode::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string("/latent_state_manager");
  status.hardware_id = "gpu";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = current_state_message_ ? "tracking" : "waiting_for_rgb_latent";
  if (watchdog_reported_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "updater_timeout";
  } else if (!current_state_message_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  const auto completed = std::max<std::uint64_t>(1U, inference_completed_);
  const auto dispatched = std::max<std::uint64_t>(1U, inference_dispatched_);
  const auto now_ns = now().nanoseconds();
  const auto state_age_ms = current_state_timestamp_ns_ > 0 && now_ns >= current_state_timestamp_ns_ ?
    static_cast<double>(now_ns - current_state_timestamp_ns_) / 1.0e6 : -1.0;
  const auto rgb_age_ms = last_rgb_timestamp_ns_ >= 0 && now_ns >= last_rgb_timestamp_ns_ ?
    static_cast<double>(now_ns - last_rgb_timestamp_ns_) / 1.0e6 : -1.0;
  const auto in_flight_elapsed_ms = inference_in_flight_ ?
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - in_flight_started_).count() : 0.0;
  status.values = {
    key_value("state_version", std::to_string(inference_completed_)),
    key_value("state_age_ms", std::to_string(state_age_ms)),
    key_value("rgb_age_ms", std::to_string(rgb_age_ms)),
    key_value("in_flight", inference_in_flight_ ? "1" : "0"),
    key_value("in_flight_elapsed_ms", std::to_string(in_flight_elapsed_ms)),
    key_value("pending_rgb", pending_rgb_message_ ? "1" : "0"),
    key_value("pending_event", pending_event_message_ ? "1" : "0"),
    key_value("rgb_received", std::to_string(rgb_received_)),
    key_value("rgb_applied", std::to_string(rgb_applied_)),
    key_value("rgb_replaced", std::to_string(rgb_replaced_)),
    key_value("rgb_stale", std::to_string(rgb_stale_)),
    key_value("event_received", std::to_string(event_received_)),
    key_value("event_replaced", std::to_string(event_replaced_)),
    key_value("event_stale", std::to_string(event_stale_)),
    key_value("inference_dispatched", std::to_string(inference_dispatched_)),
    key_value("inference_completed", std::to_string(inference_completed_)),
    key_value("stale_output", std::to_string(stale_output_)),
    key_value("invalid_message", std::to_string(invalid_message_)),
    key_value("watchdog_count", std::to_string(watchdog_count_)),
    key_value("dispatch_avg_ms", std::to_string(milliseconds(dispatch_ns_sum_ / dispatched))),
    key_value("dispatch_max_ms", std::to_string(milliseconds(dispatch_ns_max_))),
    key_value(
      "updater_round_trip_avg_ms", std::to_string(milliseconds(round_trip_ns_sum_ / completed))),
    key_value("updater_round_trip_max_ms", std::to_string(milliseconds(round_trip_ns_max_)))
  };
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

}  // namespace jetpilot_e2e_inference

RCLCPP_COMPONENTS_REGISTER_NODE(jetpilot_e2e_inference::LatentStateManagerNode)
