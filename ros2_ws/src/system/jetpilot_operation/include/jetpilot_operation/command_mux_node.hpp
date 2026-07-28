#ifndef JETPILOT_OPERATION__COMMAND_MUX_NODE_HPP_
#define JETPILOT_OPERATION__COMMAND_MUX_NODE_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "jetpilot_msgs/msg/control_command.hpp"
#include "jetpilot_msgs/msg/operation_mode_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jetpilot_operation
{

class CommandMuxNode : public rclcpp::Node
{
public:
  CommandMuxNode();

private:
  bool fresh(const std::optional<rclcpp::Time> & stamp) const;
  jetpilot_msgs::msg::ControlCommand stop_command();
  void publish_selected_command();

  double publish_rate_hz_;
  double command_timeout_s_;
  std::string control_authority_;
  std::uint8_t mode_;
  std::optional<jetpilot_msgs::msg::ControlCommand> latest_auto_;
  std::optional<jetpilot_msgs::msg::ControlCommand> latest_teleop_;
  std::optional<jetpilot_msgs::msg::ControlCommand> latest_propo_;
  std::optional<rclcpp::Time> latest_auto_stamp_;
  std::optional<rclcpp::Time> latest_teleop_stamp_;
  std::optional<rclcpp::Time> latest_propo_stamp_;
  rclcpp::Subscription<jetpilot_msgs::msg::ControlCommand>::SharedPtr auto_sub_;
  rclcpp::Subscription<jetpilot_msgs::msg::ControlCommand>::SharedPtr teleop_sub_;
  rclcpp::Subscription<jetpilot_msgs::msg::ControlCommand>::SharedPtr propo_sub_;
  rclcpp::Subscription<jetpilot_msgs::msg::OperationModeState>::SharedPtr mode_sub_;
  rclcpp::Publisher<jetpilot_msgs::msg::ControlCommand>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace jetpilot_operation

#endif  // JETPILOT_OPERATION__COMMAND_MUX_NODE_HPP_
