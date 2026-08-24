#include "jetpilot_object_detection/yolov8_decoder_node.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace jetpilot_object_detection
{
namespace
{

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

YoloV8DecoderNode::YoloV8DecoderNode(const rclcpp::NodeOptions & options)
: Node("yolov8_decoder", options)
{
  output_tensor_name_ = declare_parameter<std::string>("output_tensor_name", "output_tensor");
  class_names_ = declare_parameter<std::vector<std::string>>(
    "class_names", std::vector<std::string>{"vehicle", "barrier"});
  config_.num_classes = class_names_.size();
  config_.network_width = declare_parameter<int>("network_width", 224);
  config_.network_height = declare_parameter<int>("network_height", 224);
  config_.source_width = declare_parameter<int>("source_width", 424);
  config_.source_height = declare_parameter<int>("source_height", 240);
  config_.confidence_threshold =
    static_cast<float>(declare_parameter<double>("confidence_threshold", 0.35));
  config_.nms_threshold =
    static_cast<float>(declare_parameter<double>("nms_threshold", 0.45));
  const auto max_detections = declare_parameter<int64_t>("max_detections", 50);
  config_.tensor_layout = parse_tensor_layout(
    declare_parameter<std::string>("tensor_layout", "channel_major"));
  config_.resize_mode = parse_resize_mode(
    declare_parameter<std::string>("resize_mode", "letterbox"));

  if (class_names_.empty()) {
    throw std::invalid_argument("class_names must not be empty");
  }
  for (const auto & class_name : class_names_) {
    if (class_name.empty()) {
      throw std::invalid_argument("class_names must not contain an empty label");
    }
  }
  if (max_detections <= 0) {
    throw std::invalid_argument("max_detections must be positive");
  }
  config_.max_detections = static_cast<std::size_t>(max_detections);
  // Validates dimensions and thresholds before the first callback.
  static_cast<void>(decode_yolov8(
    std::vector<float>((4U + config_.num_classes), 0.0F).data(),
    4U + config_.num_classes, config_));

  detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "detections_output", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  tensor_sub_ = create_subscription<TensorList>(
    "tensor_sub", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&YoloV8DecoderNode::on_tensor, this, std::placeholders::_1),
    subscription_options);
}

void YoloV8DecoderNode::on_tensor(TensorList::ConstSharedPtr message)
{
  const auto callback_start = std::chrono::steady_clock::now();
  const auto header = message->get_header();
  std::size_t candidate_count = 0U;
  try {
    const auto tensor = message->get_tensor_by_name(output_tensor_name_);
    if (!tensor) {
      throw std::runtime_error("output tensor was not found: " + output_tensor_name_);
    }
    if (tensor->data_type() != nvidia::isaac_ros::nitros::NitrosDataType::kFloat32 ||
      tensor->bytes_per_element() != sizeof(float))
    {
      throw std::runtime_error("output tensor must be float32");
    }

    candidate_count = infer_candidate_count(tensor->element_count(), config_.num_classes);
    std::vector<float> values(tensor->element_count());
    auto stream_handle =
      nvidia::isaac_ros::nitros::CudaStreamPool::instance().get_stream_handle();
    auto read_handle = tensor->get_read_handle(stream_handle.get());
    if (read_handle.get_ptr() == nullptr) {
      throw std::runtime_error("output tensor buffer is null");
    }

    auto cuda_status = cudaSuccess;
    const auto storage_type = message->get_storage_type();
    switch (storage_type) {
      case cudaMemoryTypeDevice:
        cuda_status = cudaMemcpyAsync(
          values.data(), read_handle.get_ptr(), values.size() * sizeof(float),
          cudaMemcpyDeviceToHost, stream_handle.get());
        if (cuda_status == cudaSuccess) {
          cuda_status = cudaStreamSynchronize(stream_handle.get());
        }
        break;
      case cudaMemoryTypeHost:
        cuda_status = cudaStreamSynchronize(stream_handle.get());
        if (cuda_status == cudaSuccess) {
          std::memcpy(values.data(), read_handle.get_ptr(), values.size() * sizeof(float));
        }
        break;
      default:
        throw std::runtime_error("unsupported tensor storage type");
    }
    if (cuda_status != cudaSuccess) {
      throw std::runtime_error(
              std::string("failed to copy tensor from CUDA: ") + cudaGetErrorString(cuda_status));
    }

    const auto decoded = decode_yolov8(values.data(), values.size(), config_);
    vision_msgs::msg::Detection2DArray output;
    output.header = header;
    output.detections.reserve(decoded.size());
    for (const auto & item : decoded) {
      vision_msgs::msg::Detection2D detection;
      detection.header = header;
      detection.bbox.center.position.x = (item.x_min + item.x_max) * 0.5F;
      detection.bbox.center.position.y = (item.y_min + item.y_max) * 0.5F;
      detection.bbox.size_x = item.x_max - item.x_min;
      detection.bbox.size_y = item.y_max - item.y_min;
      vision_msgs::msg::ObjectHypothesisWithPose result;
      result.hypothesis.class_id = class_names_.at(static_cast<std::size_t>(item.class_id));
      result.hypothesis.score = item.score;
      detection.results.push_back(std::move(result));
      output.detections.push_back(std::move(detection));
    }
    detections_pub_->publish(output);

    const auto callback_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count();
    const auto now = get_clock()->now();
    const auto capture = rclcpp::Time(header.stamp, now.get_clock_type());
    const double capture_latency_ms = header.stamp.sec == 0 && header.stamp.nanosec == 0 ?
      0.0 : std::max(0.0, (now - capture).seconds() * 1000.0);
    publish_diagnostics(
      header, candidate_count, decoded.size(), callback_ms, capture_latency_ms);
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "YOLOv8 decode failed: %s", error.what());
    const auto callback_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count();
    publish_diagnostics(header, candidate_count, 0U, callback_ms, 0.0, error.what());
  }
}

void YoloV8DecoderNode::publish_diagnostics(
  const std_msgs::msg::Header & header,
  const std::size_t candidate_count,
  const std::size_t detection_count,
  const double callback_ms,
  const double capture_latency_ms,
  const std::string & error)
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header = header;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "JetPilot object detection";
  status.hardware_id = "jetpilot_object_detection";
  status.level = error.empty() ?
    diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = error.empty() ? "YOLOv8 decoder is running" : error;
  status.values.push_back(key_value("candidate_count", std::to_string(candidate_count)));
  status.values.push_back(key_value("detection_count", std::to_string(detection_count)));
  status.values.push_back(key_value("callback_ms", std::to_string(callback_ms)));
  status.values.push_back(key_value("capture_latency_ms", std::to_string(capture_latency_ms)));
  status.values.push_back(key_value("network_size", std::to_string(config_.network_width) + "x" +
    std::to_string(config_.network_height)));
  message.status.push_back(std::move(status));
  diagnostics_pub_->publish(message);
}

}  // namespace jetpilot_object_detection

RCLCPP_COMPONENTS_REGISTER_NODE(jetpilot_object_detection::YoloV8DecoderNode)
