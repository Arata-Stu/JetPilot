#ifndef JETPILOT_TELEOP_TOOLS__TELEOP_CMD_NODE_HPP_
#define JETPILOT_TELEOP_TOOLS__TELEOP_CMD_NODE_HPP_

#include <string>

#include "jetpilot_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace jetpilot_teleop_tools
{

class TeleopCmdNode : public rclcpp::Node
{
public:
  TeleopCmdNode();

private:
  double declare_numeric_parameter(const std::string & name, double default_value);
  static bool has_axis(const sensor_msgs::msg::Joy & joy, int index);
  static bool has_button(const sensor_msgs::msg::Joy & joy, int index);
  double apply_deadzone(double value) const;
  double normalized_trigger(const sensor_msgs::msg::Joy & joy, int axis, double scale,
                            double trigger_min, double trigger_max, bool inverted) const;
  void handle_joy(const sensor_msgs::msg::Joy & joy);

  int steering_axis_;
  int throttle_axis_;
  int reverse_axis_;
  int brake_button_;
  int deadman_button_;
  double steering_scale_;
  double throttle_scale_;
  double reverse_scale_;
  double brake_value_;
  double deadzone_;
  double trigger_min_;
  double trigger_max_;
  double throttle_trigger_min_;
  double throttle_trigger_max_;
  bool throttle_trigger_inverted_;
  double reverse_trigger_min_;
  double reverse_trigger_max_;
  bool reverse_trigger_inverted_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<jetpilot_msgs::msg::ControlCommand>::SharedPtr cmd_pub_;
};

}  // namespace jetpilot_teleop_tools

#endif  // JETPILOT_TELEOP_TOOLS__TELEOP_CMD_NODE_HPP_
