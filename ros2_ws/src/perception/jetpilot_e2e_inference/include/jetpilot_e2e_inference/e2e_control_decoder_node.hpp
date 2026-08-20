#ifndef JETPILOT_E2E_INFERENCE__E2E_CONTROL_DECODER_NODE_HPP_
#define JETPILOT_E2E_INFERENCE__E2E_CONTROL_DECODER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "jetpilot_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jetpilot_e2e_inference
{

class E2EControlDecoderNode : public rclcpp::Node
{
public:
  explicit E2EControlDecoderNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using TensorList = nvidia::isaac_ros::nitros::NitrosTensorList;

  void on_tensor(TensorList::ConstSharedPtr message);
  void publish_diagnostics(
    const TensorList & message, double callback_ms, double output_interval_ms,
    bool has_output_interval);

  std::string output_tensor_name_;
  std::vector<std::string> output_fields_;
  double steering_min_{-1.0};
  double steering_max_{1.0};
  double throttle_min_{0.0};
  double throttle_max_{1.0};
  double stale_timeout_sec_{0.2};
  double deadline_ms_{33.3};
  std::uint64_t sequence_{0U};
  bool has_last_publish_time_{false};
  std::chrono::steady_clock::time_point last_publish_time_;

  rclcpp::Publisher<jetpilot_msgs::msg::ControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<TensorList>::SharedPtr tensor_sub_;
};

}  // namespace jetpilot_e2e_inference

#endif  // JETPILOT_E2E_INFERENCE__E2E_CONTROL_DECODER_NODE_HPP_
