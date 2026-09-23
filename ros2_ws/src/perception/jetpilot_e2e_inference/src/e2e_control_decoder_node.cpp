#include "jetpilot_e2e_inference/e2e_control_decoder_node.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
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
  const nvidia::isaac_ros::nitros::NitrosTensorList & message)
{
  return message.get_header();
}

}  // namespace

E2EControlDecoderNode::E2EControlDecoderNode(const rclcpp::NodeOptions & options)
: Node("e2e_control_decoder", options)
{
  output_tensor_name_ = declare_parameter<std::string>("output_tensor_name", "output_tensor");
  output_fields_ = declare_parameter<std::vector<std::string>>(
    "output_fields", std::vector<std::string>{"steering", "throttle"});
  section_head_names_ = declare_parameter<std::vector<std::string>>("section_head_names", std::vector<std::string>{});
  section_ids_ = declare_parameter<std::vector<std::string>>("section_ids", std::vector<std::string>{});
  section_head_indices_ = declare_parameter<std::vector<std::int64_t>>("section_head_indices", std::vector<std::int64_t>{});
  section_throttles_ = declare_parameter<std::vector<double>>("section_throttles", std::vector<double>{});
  const auto generic_index = declare_parameter<int>("generic_head_index", 0);
  generic_throttle_ = declare_parameter<double>("generic_throttle", 0.2);
  section_timeout_sec_ = declare_parameter<double>("section_timeout_sec", 0.3);
  throttle_rise_per_sec_ = declare_parameter<double>("throttle_rise_per_sec", 0.2);
  steering_rate_per_sec_ = declare_parameter<double>("steering_rate_per_sec", 3.0);
  if (!section_head_names_.empty()) {
    if (generic_index < 0 || static_cast<std::size_t>(generic_index) >= section_head_names_.size() ||
      section_ids_.size() != section_head_indices_.size() || section_ids_.size() != section_throttles_.size() ||
      !std::isfinite(generic_throttle_) || generic_throttle_ < 0.0 || generic_throttle_ > 1.0 ||
      !std::isfinite(section_timeout_sec_) || section_timeout_sec_ <= 0.0 ||
      !std::isfinite(throttle_rise_per_sec_) || throttle_rise_per_sec_ <= 0.0 ||
      !std::isfinite(steering_rate_per_sec_) || steering_rate_per_sec_ <= 0.0)
    {
      throw std::invalid_argument("Invalid section multihead contract");
    }
    generic_head_index_ = static_cast<std::size_t>(generic_index);
    for (std::size_t i = 0; i < section_ids_.size(); ++i) {
      if (section_head_indices_[i] < 0 ||
        static_cast<std::size_t>(section_head_indices_[i]) >= section_head_names_.size() ||
        !std::isfinite(section_throttles_[i]) || section_throttles_[i] < 0.0 || section_throttles_[i] > 1.0)
      {
        throw std::invalid_argument("Invalid section head index or throttle");
      }
    }
    section_sub_ = create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("section_topic", "/e2e/validated_section"), rclcpp::QoS(1),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(section_mutex_);
        current_section_ = message->data;
        section_received_ = std::chrono::steady_clock::now();
      });
    active_head_pub_ = create_publisher<std_msgs::msg::String>("/e2e/active_head", 1);
  }
  steering_min_ = declare_parameter<double>("steering_min", -1.0);
  steering_max_ = declare_parameter<double>("steering_max", 1.0);
  throttle_min_ = declare_parameter<double>("throttle_min", 0.0);
  throttle_max_ = declare_parameter<double>("throttle_max", 1.0);
  fixed_throttle_mode_ = declare_parameter<bool>("fixed_throttle_mode", false);
  fixed_throttle_ = declare_parameter<double>("fixed_throttle", 0.2);
  if (!std::isfinite(fixed_throttle_.load()) || fixed_throttle_ < 0.0 || fixed_throttle_ > 1.0) {
    throw std::invalid_argument("fixed_throttle must be finite and within [0, 1]");
  }
  steering_scale_ = declare_parameter<double>("steering_scale", 1.0);
  steering_offset_ = declare_parameter<double>("steering_offset", 0.0);
  if (!std::isfinite(steering_scale_.load()) || steering_scale_ < 0.0 || steering_scale_ > 3.0 ||
      !std::isfinite(steering_offset_.load()) || steering_offset_ < -1.0 || steering_offset_ > 1.0) {
    throw std::invalid_argument("steering scale/offset is outside its allowed range");
  }
  rcl_interfaces::msg::ParameterDescriptor tuning_descriptor;
  tuning_descriptor.read_only = true;
  declare_parameter<std::vector<std::string>>("dynamic_tuning_parameters",
    std::vector<std::string>{"fixed_throttle", "steering_scale", "steering_offset"}, tuning_descriptor);

  parameter_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name != "fixed_throttle" && name != "steering_scale" && name != "steering_offset") continue;
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason = name + " must be a double";
          return result;
        }
        const auto value = parameter.as_double();
        const auto minimum = name == "steering_offset" ? -1.0 : 0.0;
        const auto maximum = name == "steering_scale" ? 3.0 : 1.0;
        if (!std::isfinite(value) || value < minimum || value > maximum) {
          result.successful = false;
          result.reason = name + " is outside its allowed range";
          return result;
        }
      }
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "fixed_throttle") fixed_throttle_.store(parameter.as_double());
        if (name == "steering_scale") steering_scale_.store(parameter.as_double());
        if (name == "steering_offset") steering_offset_.store(parameter.as_double());
      }
      return result;
    });
  stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.2);
  deadline_ms_ = declare_parameter<double>("deadline_ms", 33.3);
  enable_pipeline_latency_breakdown_ =
    declare_parameter<bool>("enable_pipeline_latency_breakdown", false);
  const auto pipeline_latency_max_pending =
    declare_parameter<std::int64_t>("pipeline_latency_max_pending", 4096);
  const auto diagnostics_topic =
    declare_parameter<std::string>("diagnostics_topic", "/e2e/diagnostics");

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
  if (pipeline_latency_max_pending <= 0) {
    throw std::invalid_argument("pipeline_latency_max_pending must be positive");
  }
  pipeline_latency_max_pending_ = static_cast<std::size_t>(pipeline_latency_max_pending);

  command_pub_ = create_publisher<jetpilot_msgs::msg::ControlCommand>("control_cmd", 10);
  diagnostics_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  if (enable_pipeline_latency_breakdown_) {
    tensor_input_probe_sub_ = create_subscription<TensorList>(
      "tensor_input_probe", rclcpp::QoS(10),
      std::bind(&E2EControlDecoderNode::on_input_tensor, this, std::placeholders::_1),
      subscription_options);
  }
  tensor_sub_ = create_subscription<TensorList>(
    "tensor_sub", rclcpp::QoS(10),
    std::bind(&E2EControlDecoderNode::on_tensor, this, std::placeholders::_1),
    subscription_options);
}

void E2EControlDecoderNode::on_input_tensor(TensorList::ConstSharedPtr message)
{
  if (!message) {
    return;
  }
  const auto received_at = std::chrono::steady_clock::now();
  const auto current_ros_ns = now().nanoseconds();
  const auto stamp_ns =
    static_cast<std::int64_t>(message->get_timestamp_sec()) * 1000000000LL +
    static_cast<std::int64_t>(message->get_timestamp_nsec());
  if (stamp_ns <= 0) {
    return;
  }
  const double sensor_to_input_ms = std::max(
    0.0, static_cast<double>(current_ros_ns - stamp_ns) / 1.0e6);

  std::lock_guard<std::mutex> lock(pipeline_timing_mutex_);
  while (!pipeline_input_order_.empty() &&
    pipeline_input_timings_.find(pipeline_input_order_.front()) == pipeline_input_timings_.end())
  {
    pipeline_input_order_.pop_front();
  }
  while (pipeline_input_timings_.size() >= pipeline_latency_max_pending_ &&
    !pipeline_input_order_.empty())
  {
    pipeline_input_timings_.erase(pipeline_input_order_.front());
    pipeline_input_order_.pop_front();
    pipeline_evicted_inputs_.fetch_add(1U, std::memory_order_relaxed);
  }
  const auto [iterator, inserted] = pipeline_input_timings_.insert_or_assign(
    stamp_ns, TensorInputTiming{received_at, sensor_to_input_ms});
  (void)iterator;
  if (inserted) {
    pipeline_input_order_.push_back(stamp_ns);
  }
}

E2EControlDecoderNode::PipelineTiming E2EControlDecoderNode::match_input_tensor(
  const TensorList & message, const SteadyTime output_received_at)
{
  PipelineTiming result;
  if (!enable_pipeline_latency_breakdown_) {
    return result;
  }
  const auto stamp_ns =
    static_cast<std::int64_t>(message.get_timestamp_sec()) * 1000000000LL +
    static_cast<std::int64_t>(message.get_timestamp_nsec());
  const auto current_ros_ns = now().nanoseconds();
  if (stamp_ns > 0) {
    result.sensor_to_output_ms = std::max(
      0.0, static_cast<double>(current_ros_ns - stamp_ns) / 1.0e6);
  }

  std::lock_guard<std::mutex> lock(pipeline_timing_mutex_);
  const auto iterator = pipeline_input_timings_.find(stamp_ns);
  if (iterator != pipeline_input_timings_.end()) {
    result.matched = true;
    result.sensor_to_input_ms = iterator->second.sensor_to_input_ms;
    result.input_to_output_ms = std::chrono::duration<double, std::milli>(
      output_received_at - iterator->second.received_at).count();
    pipeline_input_timings_.erase(iterator);
  } else {
    pipeline_unmatched_outputs_.fetch_add(1U, std::memory_order_relaxed);
  }
  while (!pipeline_input_order_.empty() &&
    pipeline_input_timings_.find(pipeline_input_order_.front()) == pipeline_input_timings_.end())
  {
    pipeline_input_order_.pop_front();
  }
  result.pending_inputs = pipeline_input_timings_.size();
  return result;
}

void E2EControlDecoderNode::on_tensor(TensorList::ConstSharedPtr message)
{
  const auto callback_started = std::chrono::steady_clock::now();
  const auto pipeline_timing = match_input_tensor(*message, callback_started);
  const auto publish_time = callback_started;
  const bool has_output_interval = has_last_publish_time_;
  const double output_interval_ms = has_output_interval ?
    std::chrono::duration<double, std::milli>(publish_time - last_publish_time_).count() : 0.0;

  try {
    const auto tensor = message->get_tensor_by_name(output_tensor_name_);
    if (!tensor) {
      RCLCPP_WARN(get_logger(), "Tensor '%s' was not found", output_tensor_name_.c_str());
      return;
    }
    if (tensor->data_type() != nvidia::isaac_ros::nitros::NitrosDataType::kFloat32 ||
      tensor->bytes_per_element() != sizeof(float))
    {
      RCLCPP_WARN(get_logger(), "Tensor '%s' is not float32", output_tensor_name_.c_str());
      return;
    }
    const auto value_count = section_head_names_.empty() ? output_fields_.size() : section_head_names_.size();
    if (!section_head_names_.empty() && tensor->element_count() != value_count) {
      RCLCPP_ERROR(get_logger(), "Multihead output count does not match metadata");
      return;
    }
    if (tensor->element_count() < value_count) {
      RCLCPP_WARN(
        get_logger(), "Tensor '%s' has %lu values; expected at least %lu",
        output_tensor_name_.c_str(), static_cast<unsigned long>(tensor->element_count()),
        static_cast<unsigned long>(output_fields_.size()));
      return;
    }

    std::vector<float> values(value_count);
    auto stream_handle =
      nvidia::isaac_ros::nitros::CudaStreamPool::instance().get_stream_handle();
    auto read_handle = tensor->get_read_handle(stream_handle.get());
    if (read_handle.get_ptr() == nullptr) {
      RCLCPP_ERROR(get_logger(), "Control tensor buffer is null");
      return;
    }

    auto cuda_status = cudaSuccess;
    const auto storage_type = message->get_storage_type();
    switch (storage_type) {
      case cudaMemoryTypeDevice:
        cuda_status = cudaMemcpyAsync(
          values.data(), read_handle.get_ptr(), values.size() * sizeof(float),
          cudaMemcpyDeviceToHost, stream_handle.get());
        break;
      case cudaMemoryTypeHost:
        cuda_status = cudaStreamSynchronize(stream_handle.get());
        if (cuda_status == cudaSuccess) {
          std::memcpy(
            values.data(), read_handle.get_ptr(), values.size() * sizeof(float));
        }
        break;
      default:
        RCLCPP_ERROR(
          get_logger(), "Unsupported control tensor storage type: %d",
          static_cast<int>(storage_type));
        return;
    }
    if (cuda_status == cudaSuccess && storage_type == cudaMemoryTypeDevice) {
      cuda_status = cudaStreamSynchronize(stream_handle.get());
    }
    if (cuda_status != cudaSuccess) {
      RCLCPP_ERROR(
        get_logger(), "Failed to copy control tensor from CUDA: %s",
        cudaGetErrorString(cuda_status));
      return;
    }

    float steering = 0.0F;
    float throttle = 0.0F;
    std::size_t selected_head = generic_head_index_;
    double selected_throttle = generic_throttle_;
    if (!section_head_names_.empty()) {
      const auto sensor_age = (now() - rclcpp::Time(message->get_header().stamp)).seconds();
      if (sensor_age < -0.05 || sensor_age > stale_timeout_sec_) {
        RCLCPP_WARN(get_logger(), "Dropping stale multihead inference");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(section_mutex_);
        if (std::chrono::duration<double>(callback_started - section_received_).count() <= section_timeout_sec_) {
          for (std::size_t i = 0; i < section_ids_.size(); ++i) {
            if (section_ids_[i] == current_section_) {
              selected_head = static_cast<std::size_t>(section_head_indices_[i]);
              selected_throttle = section_throttles_[i];
              break;
            }
          }
        }
      }
      steering = values[selected_head];
      throttle = static_cast<float>(selected_throttle);
    } else {
      for (std::size_t index = 0U; index < output_fields_.size(); ++index) {
        if (!std::isfinite(values[index])) {
          RCLCPP_WARN(get_logger(), "Tensor contains a non-finite control value");
          return;
        }
        if (output_fields_[index] == "steering") steering = values[index];
        else if (output_fields_[index] == "throttle") throttle = values[index];
      }
    }
    if (!std::isfinite(steering) || !std::isfinite(throttle)) {
      RCLCPP_WARN(get_logger(), "Tensor contains a non-finite control value");
      return;
    }

    jetpilot_msgs::msg::ControlCommand command;
    command.header = make_header(*message);
    if (command.header.frame_id.empty()) {
      command.header.frame_id = "base_link";
    }
    command.steering = std::clamp(
      static_cast<double>(steering) * steering_scale_.load() + steering_offset_.load(), steering_min_, steering_max_);
    command.throttle = std::clamp(
      (section_head_names_.empty() && fixed_throttle_mode_) ? fixed_throttle_.load() : static_cast<double>(throttle),
      throttle_min_, throttle_max_);
    if (!section_head_names_.empty()) {
      const double dt = has_output_interval ? std::clamp(output_interval_ms / 1000.0, 0.0, 0.1) : 0.0;
      command.throttle = std::min(
        static_cast<double>(command.throttle), previous_throttle_ + throttle_rise_per_sec_ * dt);
      command.steering = std::clamp(static_cast<double>(command.steering),
        previous_steering_ - steering_rate_per_sec_ * dt,
        previous_steering_ + steering_rate_per_sec_ * dt);
      previous_throttle_ = command.throttle;
      previous_steering_ = command.steering;
      std_msgs::msg::String selected;
      selected.data = section_head_names_[selected_head];
      active_head_pub_->publish(selected);
    }
    command.brake = 0.0;
    command.reverse = 0.0;
    command_pub_->publish(command);

    ++sequence_;
    const auto callback_finished = std::chrono::steady_clock::now();
    const double callback_ms =
      std::chrono::duration<double, std::milli>(callback_finished - callback_started).count();
    publish_diagnostics(
      *message, callback_ms, output_interval_ms, has_output_interval, pipeline_timing);
    last_publish_time_ = publish_time;
    has_last_publish_time_ = true;
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "Failed to decode control tensor: %s", error.what());
  }
}

void E2EControlDecoderNode::publish_diagnostics(
  const TensorList & message, const double callback_ms, const double output_interval_ms,
  const bool has_output_interval, const PipelineTiming & pipeline_timing)
{
  const auto current_time = now();
  const std::int64_t stamp_ns =
    static_cast<std::int64_t>(message.get_timestamp_sec()) * 1000000000LL +
    static_cast<std::int64_t>(message.get_timestamp_nsec());
  const bool has_capture_stamp = stamp_ns > 0;
  const double capture_to_command_signed_ms = has_capture_stamp ?
    static_cast<double>(current_time.nanoseconds() - stamp_ns) / 1.0e6 : 0.0;
  const bool source_timestamp_future = has_capture_stamp && capture_to_command_signed_ms < 0.0;
  const double capture_to_command_ms = std::max(0.0, capture_to_command_signed_ms);
  const double deadline_value_ms = has_capture_stamp ? capture_to_command_ms : callback_ms;
  const bool missed_deadline = deadline_value_ms > deadline_ms_;
  const bool stale_output =
    has_output_interval && output_interval_ms > stale_timeout_sec_ * 1000.0;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "jetpilot_e2e_inference/pipeline";
  status.hardware_id = "jetpilot-e2e";
  status.level = (missed_deadline || stale_output || source_timestamp_future) ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (missed_deadline) {
    status.message = "deadline missed";
  } else if (stale_output) {
    status.message = "stale output interval";
  } else if (source_timestamp_future) {
    status.message = "source timestamp is in the future";
  } else {
    status.message = "ok";
  }
  status.values = {
    diagnostic_value(
      "capture_to_command_ms", has_capture_stamp ? std::to_string(capture_to_command_ms) : ""),
    diagnostic_value(
      "capture_to_command_signed_ms",
      has_capture_stamp ? std::to_string(capture_to_command_signed_ms) : ""),
    diagnostic_value("source_timestamp_future", source_timestamp_future ? "1" : "0"),
    diagnostic_value(
      "source_timestamp_future_by_ms",
      source_timestamp_future ? std::to_string(-capture_to_command_signed_ms) : "0.0"),
    diagnostic_value("decoder_callback_ms", std::to_string(callback_ms)),
    diagnostic_value(
      "output_interval_ms", has_output_interval ? std::to_string(output_interval_ms) : ""),
    diagnostic_value("deadline_ms", std::to_string(deadline_ms_)),
    diagnostic_value("stale_timeout_sec", std::to_string(stale_timeout_sec_)),
    diagnostic_value("missed_deadline", missed_deadline ? "1" : "0"),
    diagnostic_value("stale_output", stale_output ? "1" : "0"),
    diagnostic_value("sequence", std::to_string(sequence_)),
    diagnostic_value("source_timestamp_sec", std::to_string(message.get_timestamp_sec())),
    diagnostic_value("source_timestamp_nanosec", std::to_string(message.get_timestamp_nsec())),
    diagnostic_value(
      "pipeline_latency_breakdown_enabled",
      enable_pipeline_latency_breakdown_ ? "true" : "false"),
    diagnostic_value("tensor_input_matched", pipeline_timing.matched ? "1" : "0"),
    diagnostic_value(
      "sensor_to_tensor_input_ms",
      pipeline_timing.matched ? std::to_string(pipeline_timing.sensor_to_input_ms) : ""),
    diagnostic_value(
      "tensor_input_to_output_ms",
      pipeline_timing.matched ? std::to_string(pipeline_timing.input_to_output_ms) : ""),
    diagnostic_value(
      "sensor_to_tensor_output_ms",
      pipeline_timing.matched ? std::to_string(pipeline_timing.sensor_to_output_ms) : ""),
    diagnostic_value("tensor_output_to_command_ms", std::to_string(callback_ms)),
    diagnostic_value(
      "pipeline_latency_pending_inputs", std::to_string(pipeline_timing.pending_inputs)),
    diagnostic_value(
      "pipeline_latency_unmatched_outputs",
      std::to_string(pipeline_unmatched_outputs_.load(std::memory_order_relaxed))),
    diagnostic_value(
      "pipeline_latency_evicted_inputs",
      std::to_string(pipeline_evicted_inputs_.load(std::memory_order_relaxed))),
  };

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = current_time;
  diagnostics.status.push_back(std::move(status));
  diagnostics_pub_->publish(diagnostics);
}

}  // namespace jetpilot_e2e_inference

RCLCPP_COMPONENTS_REGISTER_NODE(jetpilot_e2e_inference::E2EControlDecoderNode)
