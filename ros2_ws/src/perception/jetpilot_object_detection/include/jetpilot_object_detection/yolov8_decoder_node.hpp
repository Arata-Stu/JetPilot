#ifndef JETPILOT_OBJECT_DETECTION__YOLOV8_DECODER_NODE_HPP_
#define JETPILOT_OBJECT_DETECTION__YOLOV8_DECODER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "jetpilot_object_detection/yolov8_decoder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace jetpilot_object_detection
{

class YoloV8DecoderNode : public rclcpp::Node
{
public:
  explicit YoloV8DecoderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using TensorList = nvidia::isaac_ros::nitros::NitrosTensorList;

  void on_tensor(TensorList::ConstSharedPtr message);
  void publish_diagnostics(
    const std_msgs::msg::Header & header,
    std::size_t candidate_count,
    std::size_t detection_count,
    double callback_ms,
    double capture_latency_ms,
    const std::string & error = "");

  std::string output_tensor_name_;
  std::vector<std::string> class_names_;
  DecoderConfig config_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<TensorList>::SharedPtr tensor_sub_;
};

}  // namespace jetpilot_object_detection

#endif  // JETPILOT_OBJECT_DETECTION__YOLOV8_DECODER_NODE_HPP_
