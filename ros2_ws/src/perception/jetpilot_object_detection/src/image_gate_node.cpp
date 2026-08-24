#include "jetpilot_object_detection/image_gate_node.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace jetpilot_object_detection
{

ImageGateNode::ImageGateNode(const rclcpp::NodeOptions & options)
: Node("object_detection_image_gate", options)
{
  const double max_fps = declare_parameter<double>("max_fps", 15.0);
  if (!std::isfinite(max_fps) || max_fps <= 0.0) {
    throw std::invalid_argument("max_fps must be positive and finite");
  }
  min_interval_ns_ = static_cast<std::int64_t>(1.0e9 / max_fps);

  const auto qos = rclcpp::SensorDataQoS().keep_last(1);
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_output", qos);
  camera_info_pub_ =
    create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_output", qos);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "image_input", qos,
    std::bind(&ImageGateNode::on_image, this, std::placeholders::_1),
    subscription_options);
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info_input", qos,
    std::bind(&ImageGateNode::on_camera_info, this, std::placeholders::_1),
    subscription_options);
}

void ImageGateNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  latest_camera_info_ = std::move(message);
}

void ImageGateNode::on_image(std::unique_ptr<sensor_msgs::msg::Image> message)
{
  const auto & stamp = message->header.stamp;
  std::int64_t timestamp_ns =
    static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
  if (timestamp_ns <= 0) {
    timestamp_ns = get_clock()->now().nanoseconds();
  }
  if (last_published_ns_ > 0 && timestamp_ns >= last_published_ns_ &&
    timestamp_ns - last_published_ns_ < min_interval_ns_)
  {
    return;
  }

  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    latest_camera_info = latest_camera_info_;
  }
  if (!latest_camera_info) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for object-detection camera_info");
    return;
  }

  sensor_msgs::msg::CameraInfo camera_info(*latest_camera_info);
  camera_info.header = message->header;
  camera_info_pub_->publish(camera_info);
  last_published_ns_ = timestamp_ns;
  image_pub_->publish(std::move(message));
}

}  // namespace jetpilot_object_detection

RCLCPP_COMPONENTS_REGISTER_NODE(jetpilot_object_detection::ImageGateNode)
