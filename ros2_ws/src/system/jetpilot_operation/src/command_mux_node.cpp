#include <algorithm>
#include <chrono>

#include "jetpilot_operation/command_mux_node.hpp"

namespace jetpilot_operation
{

CommandMuxNode::CommandMuxNode() : Node("command_mux_node")
{
  publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 100.0));
  command_timeout_s_ = std::max(0.0, declare_parameter<double>("command_timeout_s", 0.3));
  control_authority_ = declare_parameter<std::string>("control_authority", "hardware_mux");
  mode_ = jetpilot_msgs::msg::OperationModeState::STOP;

  const auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  auto_sub_ = create_subscription<jetpilot_msgs::msg::ControlCommand>(
    "/auto/control_cmd", qos_cmd,
    [this](const jetpilot_msgs::msg::ControlCommand::SharedPtr msg)
    {
      latest_auto_ = *msg;
      latest_auto_stamp_ = now();
    });
  teleop_sub_ = create_subscription<jetpilot_msgs::msg::ControlCommand>(
    "/teleop/control_cmd", qos_cmd,
    [this](const jetpilot_msgs::msg::ControlCommand::SharedPtr msg)
    {
      latest_teleop_ = *msg;
      latest_teleop_stamp_ = now();
    });
  propo_sub_ = create_subscription<jetpilot_msgs::msg::ControlCommand>(
    "/propo/control_cmd", qos_cmd,
    [this](const jetpilot_msgs::msg::ControlCommand::SharedPtr msg)
    {
      latest_propo_ = *msg;
      latest_propo_stamp_ = now();
    });
  mode_sub_ = create_subscription<jetpilot_msgs::msg::OperationModeState>(
    "/operation_mode/state", rclcpp::QoS(1).transient_local().reliable(),
    [this](const jetpilot_msgs::msg::OperationModeState::SharedPtr msg) { mode_ = msg->mode; });
  output_pub_ =
    create_publisher<jetpilot_msgs::msg::ControlCommand>("/vehicle/control_cmd", qos_cmd);

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                             [this]() { publish_selected_command(); });
}

bool CommandMuxNode::fresh(const std::optional<rclcpp::Time> & stamp) const
{
  if (!stamp)
  {
    return false;
  }
  return (now() - *stamp).seconds() <= command_timeout_s_;
}

jetpilot_msgs::msg::ControlCommand CommandMuxNode::stop_command()
{
  jetpilot_msgs::msg::ControlCommand cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.steering = 0.0F;
  cmd.throttle = 0.0F;
  cmd.brake = 0.0F;
  cmd.reverse = 0.0F;
  return cmd;
}

void CommandMuxNode::publish_selected_command()
{
  auto output = stop_command();
  if (mode_ == jetpilot_msgs::msg::OperationModeState::AUTO && latest_auto_ &&
      fresh(latest_auto_stamp_))
  {
    output = *latest_auto_;
  }
  else if (mode_ == jetpilot_msgs::msg::OperationModeState::MANUAL && latest_teleop_ &&
           fresh(latest_teleop_stamp_))
  {
    output = *latest_teleop_;
  }
  else if (control_authority_ == "jetson_mux" &&
           mode_ == jetpilot_msgs::msg::OperationModeState::PROPO && latest_propo_ &&
           fresh(latest_propo_stamp_))
  {
    output = *latest_propo_;
  }

  output.header.stamp = now();
  output_pub_->publish(output);
}

}  // namespace jetpilot_operation
