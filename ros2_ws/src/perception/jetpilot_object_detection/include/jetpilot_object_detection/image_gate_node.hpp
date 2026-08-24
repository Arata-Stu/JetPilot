#ifndef JETPILOT_OBJECT_DETECTION__IMAGE_GATE_NODE_HPP_
#define JETPILOT_OBJECT_DETECTION__IMAGE_GATE_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace jetpilot_object_detection
{

class ImageGateNode : public rclcpp::Node
{
public:
  explicit ImageGateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr message);
  void on_image(std::unique_ptr<sensor_msgs::msg::Image> message);

  std::int64_t min_interval_ns_{0};
  std::int64_t last_published_ns_{0};
  std::mutex camera_info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

}  // namespace jetpilot_object_detection

#endif  // JETPILOT_OBJECT_DETECTION__IMAGE_GATE_NODE_HPP_
