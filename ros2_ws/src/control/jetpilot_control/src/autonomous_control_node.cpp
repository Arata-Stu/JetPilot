#include <algorithm>
#include <chrono>

#include "jetpilot_control/autonomous_control_node.hpp"

namespace jetpilot_control
{

AutonomousControlNode::AutonomousControlNode() : Node("autonomous_control_node")
{
  publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 20.0));
  steering_ = std::clamp(declare_parameter<double>("steering", 0.0), -1.0, 1.0);
  throttle_ = std::clamp(declare_parameter<double>("throttle", 0.0), 0.0, 1.0);
  brake_ = std::clamp(declare_parameter<double>("brake", 1.0), 0.0, 1.0);
  reverse_ = std::clamp(declare_parameter<double>("reverse", 0.0), 0.0, 1.0);

  const auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  cmd_pub_ = create_publisher<jetpilot_msgs::msg::ControlCommand>("/auto/control_cmd", qos_cmd);
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                             [this]() { publish_command(); });
}

void AutonomousControlNode::publish_command()
{
  jetpilot_msgs::msg::ControlCommand cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.steering = static_cast<float>(steering_);
  cmd.throttle = static_cast<float>(throttle_);
  cmd.brake = static_cast<float>(brake_);
  cmd.reverse = static_cast<float>(reverse_);
  cmd_pub_->publish(cmd);
}

}  // namespace jetpilot_control
