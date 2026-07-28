#ifndef JETPILOT_CONTROL__AUTONOMOUS_CONTROL_NODE_HPP_
#define JETPILOT_CONTROL__AUTONOMOUS_CONTROL_NODE_HPP_

#include "jetpilot_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jetpilot_control
{

class AutonomousControlNode : public rclcpp::Node
{
public:
  AutonomousControlNode();

private:
  void publish_command();

  double publish_rate_hz_;
  double steering_;
  double throttle_;
  double brake_;
  double reverse_;
  rclcpp::Publisher<jetpilot_msgs::msg::ControlCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace jetpilot_control

#endif  // JETPILOT_CONTROL__AUTONOMOUS_CONTROL_NODE_HPP_
