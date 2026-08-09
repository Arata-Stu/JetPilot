#include "jetpilot_e2e_inference/e2e_control_decoder_node.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"

namespace jetpilot_e2e_inference
{
namespace
{

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std_msgs::msg::Header make_header(
  const nvidia::isaac_ros::nitros::NitrosTensorListView & message)
{
  std_msgs::msg::Header header;
  header.stamp.sec = message.GetTimestampSeconds();
  header.stamp.nanosec = message.GetTimestampNanoseconds();
  header.frame_id = message.GetFrameId();
  return header;
}

}  // namespace

E2EControlDecoderNode::E2EControlDecoderNode(const rclcpp::NodeOptions & options)
: Node("e2e_control_decoder", options)
{
  output_tensor_name_ = declare_parameter<std::string>("output_tensor_name", "output_tensor");
  output_fields_ = declare_parameter<std::vector<std::string>>(
    "output_fields", std::vector<std::string>{"steering", "throttle"});
  steering_min_ = declare_parameter<double>("steering_min", -1.0);
  steering_max_ = declare_parameter<double>("steering_max", 1.0);
  throttle_min_ = declare_parameter<double>("throttle_min", 0.0);
  throttle_max_ = declare_parameter<double>("throttle_max", 1.0);
  stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.2);
  deadline_ms_ = declare_parameter<double>("deadline_ms", 33.3);
  const auto diagnostics_topic =
    declare_parameter<std::string>("diagnostics_topic", "/e2e/diagnostics");
  const auto nitros_tensor_format = declare_parameter<std::string>(
    "nitros_tensor_format",
    nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name);

  if (output_fields_.empty()) {
    throw std::invalid_argument("output_fields must not be empty");
  }
  if (steering_min_ > steering_max_) {
    throw std::invalid_argument("steering_min must be <= steering_max");
  }
  if (throttle_min_ > throttle_max_) {
    throw std::invalid_argument("throttle_min must be <= throttle_max");
  }
  if (stale_timeout_sec_ <= 0.0 || deadline_ms_ <= 0.0) {
    throw std::invalid_argument("stale_timeout_sec and deadline_ms must be positive");
  }

  command_pub_ = create_publisher<jetpilot_msgs::msg::ControlCommand>("control_cmd", 10);
  diagnostics_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);
  tensor_sub_ = std::make_shared<NitrosSubscriber>(
    this, "tensor_sub", nitros_tensor_format,
    std::bind(&E2EControlDecoderNode::on_tensor, this, std::placeholders::_1),
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig{}, rclcpp::QoS(10));
}

void E2EControlDecoderNode::on_tensor(const TensorListView & message)
{
  const auto callback_started = std::chrono::steady_clock::now();
  const auto publish_time = callback_started;
  const bool has_output_interval = has_last_publish_time_;
  const double output_interval_ms = has_output_interval ?
    std::chrono::duration<double, std::milli>(publish_time - last_publish_time_).count() : 0.0;

  try {
    const auto tensor = message.GetNamedTensor(output_tensor_name_);
    if (tensor.GetElementType() != nvidia::gxf::PrimitiveType::kFloat32 ||
      tensor.GetBytesPerElement() != sizeof(float))
    {
      RCLCPP_WARN(get_logger(), "Tensor '%s' is not float32", output_tensor_name_.c_str());
      return;
    }
    if (tensor.GetElementCount() < output_fields_.size()) {
      RCLCPP_WARN(
        get_logger(), "Tensor '%s' has %lu values; expected at least %lu",
        output_tensor_name_.c_str(), static_cast<unsigned long>(tensor.GetElementCount()),
        static_cast<unsigned long>(output_fields_.size()));
      return;
    }

    std::vector<float> values(output_fields_.size());
    const auto cuda_status = cudaMemcpy(
      values.data(), tensor.GetBuffer(), values.size() * sizeof(float), cudaMemcpyDefault);
    if (cuda_status != cudaSuccess) {
      RCLCPP_ERROR(
        get_logger(), "Failed to copy control tensor from CUDA: %s",
        cudaGetErrorString(cuda_status));
      return;
    }

    float steering = 0.0F;
    float throttle = 0.0F;
    for (std::size_t index = 0U; index < output_fields_.size(); ++index) {
      if (!std::isfinite(values[index])) {
        RCLCPP_WARN(get_logger(), "Tensor contains a non-finite control value");
        return;
      }
      if (output_fields_[index] == "steering") {
        steering = values[index];
      } else if (output_fields_[index] == "throttle") {
        throttle = values[index];
      }
    }

    jetpilot_msgs::msg::ControlCommand command;
    command.header = make_header(message);
    if (command.header.frame_id.empty()) {
      command.header.frame_id = "base_link";
    }
    command.steering = std::clamp(
      static_cast<double>(steering), steering_min_, steering_max_);
    command.throttle = std::clamp(
      static_cast<double>(throttle), throttle_min_, throttle_max_);
    command.brake = 0.0;
    command.reverse = 0.0;
    command_pub_->publish(command);

    ++sequence_;
    const auto callback_finished = std::chrono::steady_clock::now();
    const double callback_ms =
      std::chrono::duration<double, std::milli>(callback_finished - callback_started).count();
    publish_diagnostics(message, callback_ms, output_interval_ms, has_output_interval);
    last_publish_time_ = publish_time;
    has_last_publish_time_ = true;
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "Failed to decode control tensor: %s", error.what());
  }
}

void E2EControlDecoderNode::publish_diagnostics(
  const TensorListView & message, const double callback_ms, const double output_interval_ms,
  const bool has_output_interval)
{
  const auto current_time = now();
  const std::int64_t stamp_ns =
    static_cast<std::int64_t>(message.GetTimestampSeconds()) * 1000000000LL +
    static_cast<std::int64_t>(message.GetTimestampNanoseconds());
  const bool has_capture_stamp = stamp_ns > 0;
  const double capture_to_command_ms = has_capture_stamp ?
    std::max(0.0, static_cast<double>(current_time.nanoseconds() - stamp_ns) / 1.0e6) : 0.0;
  const double deadline_value_ms = has_capture_stamp ? capture_to_command_ms : callback_ms;
  const bool missed_deadline = deadline_value_ms > deadline_ms_;
  const bool stale_output =
    has_output_interval && output_interval_ms > stale_timeout_sec_ * 1000.0;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "jetpilot_e2e_inference/pipeline";
  status.hardware_id = "jetpilot-e2e";
  status.level = (missed_deadline || stale_output) ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (missed_deadline) {
    status.message = "deadline missed";
  } else if (stale_output) {
    status.message = "stale output interval";
  } else {
    status.message = "ok";
  }
  status.values = {
    diagnostic_value(
      "capture_to_command_ms", has_capture_stamp ? std::to_string(capture_to_command_ms) : ""),
    diagnostic_value("decoder_callback_ms", std::to_string(callback_ms)),
    diagnostic_value(
      "output_interval_ms", has_output_interval ? std::to_string(output_interval_ms) : ""),
    diagnostic_value("deadline_ms", std::to_string(deadline_ms_)),
    diagnostic_value("stale_timeout_sec", std::to_string(stale_timeout_sec_)),
    diagnostic_value("missed_deadline", missed_deadline ? "1" : "0"),
    diagnostic_value("stale_output", stale_output ? "1" : "0"),
    diagnostic_value("sequence", std::to_string(sequence_)),
  };

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = current_time;
  diagnostics.status.push_back(std::move(status));
  diagnostics_pub_->publish(diagnostics);
}

}  // namespace jetpilot_e2e_inference

RCLCPP_COMPONENTS_REGISTER_NODE(jetpilot_e2e_inference::E2EControlDecoderNode)
