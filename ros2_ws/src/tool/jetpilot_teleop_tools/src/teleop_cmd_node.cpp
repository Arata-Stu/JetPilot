#include <algorithm>
#include <cstddef>
#include <cmath>

#include "jetpilot_teleop_tools/teleop_cmd_node.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace jetpilot_teleop_tools
{

TeleopCmdNode::TeleopCmdNode() : Node("teleop_cmd_node")
{
  steering_axis_ = declare_parameter<int>("steering_axis", 0);
  throttle_axis_ = declare_parameter<int>("throttle_axis", 5);
  reverse_axis_ = declare_parameter<int>("reverse_axis", 2);
  brake_button_ = declare_parameter<int>("brake_button", -1);
  deadman_button_ = declare_parameter<int>("deadman_button", 3);
  steering_scale_ = declare_numeric_parameter("steering_scale", 1.0);
  throttle_scale_step_ = std::max(0.0, declare_numeric_parameter("throttle_scale_step", 0.05));
  throttle_scale_min_ = std::clamp(
    declare_numeric_parameter("throttle_scale_min", 0.0), 0.0, 1.0);
  throttle_scale_max_ = std::clamp(
    declare_numeric_parameter("throttle_scale_max", 1.0), throttle_scale_min_, 1.0);
  throttle_scale_.store(std::clamp(
    declare_numeric_parameter("throttle_scale", 1.0),
    throttle_scale_min_, throttle_scale_max_));
  reverse_scale_ = declare_numeric_parameter("reverse_scale", 1.0);
  brake_value_ = std::clamp(declare_numeric_parameter("brake_value", 1.0), 0.0, 1.0);
  deadzone_ = std::clamp(declare_numeric_parameter("deadzone", 0.05), 0.0, 1.0);
  trigger_min_ = declare_numeric_parameter("trigger_min", -1.0);
  trigger_max_ = declare_numeric_parameter("trigger_max", 1.0);
  throttle_trigger_min_ = declare_numeric_parameter("throttle_trigger_min", trigger_min_);
  throttle_trigger_max_ = declare_numeric_parameter("throttle_trigger_max", trigger_max_);
  throttle_trigger_inverted_ = declare_parameter<bool>("throttle_trigger_inverted", false);
  reverse_trigger_min_ = declare_numeric_parameter("reverse_trigger_min", trigger_min_);
  reverse_trigger_max_ = declare_numeric_parameter("reverse_trigger_max", trigger_max_);
  reverse_trigger_inverted_ = declare_parameter<bool>("reverse_trigger_inverted", false);

  parameter_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return handle_parameters(parameters);
    });

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { handle_joy(*msg); });
  speed_offset_inc_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/speed_offset_inc", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data)
      {
        adjust_throttle_scale(1.0);
      }
    });
  speed_offset_dec_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/speed_offset_dec", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data)
      {
        adjust_throttle_scale(-1.0);
      }
    });
  const auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  cmd_pub_ = create_publisher<jetpilot_msgs::msg::ControlCommand>("/teleop/control_cmd", qos_cmd);

  // Keep introspection (`ros2 param get`) consistent when an out-of-range startup value was
  // clamped above.
  set_parameter(rclcpp::Parameter("throttle_scale", throttle_scale_.load()));
}

double TeleopCmdNode::declare_numeric_parameter(const std::string & name,
                                                const double default_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor, false);

  rclcpp::Parameter parameter;
  if (!get_parameter(name, parameter) ||
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    return default_value;
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
  {
    return parameter.as_double();
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    return static_cast<double>(parameter.as_int());
  }

  RCLCPP_WARN(get_logger(), "Parameter '%s' must be numeric; using default %.3f", name.c_str(),
              default_value);
  return default_value;
}

bool TeleopCmdNode::has_axis(const sensor_msgs::msg::Joy & joy, const int index)
{
  return index >= 0 && static_cast<std::size_t>(index) < joy.axes.size();
}

bool TeleopCmdNode::has_button(const sensor_msgs::msg::Joy & joy, const int index)
{
  return index >= 0 && static_cast<std::size_t>(index) < joy.buttons.size();
}

double TeleopCmdNode::apply_deadzone(const double value) const
{
  return std::abs(value) < deadzone_ ? 0.0 : value;
}

double TeleopCmdNode::normalized_trigger(const sensor_msgs::msg::Joy & joy, const int axis,
                                         const double scale, const double trigger_min,
                                         const double trigger_max, const bool inverted) const
{
  if (!has_axis(joy, axis) || trigger_max == trigger_min)
  {
    return 0.0;
  }
  const double raw = std::clamp(static_cast<double>(joy.axes[axis]), trigger_min, trigger_max);
  const double normalized = inverted ? (trigger_max - raw) / (trigger_max - trigger_min)
                                     : (raw - trigger_min) / (trigger_max - trigger_min);
  return std::clamp(normalized * scale, 0.0, 1.0);
}

void TeleopCmdNode::adjust_throttle_scale(const double direction)
{
  const double previous = throttle_scale_.load();
  const double requested = std::clamp(
    previous + direction * throttle_scale_step_, throttle_scale_min_, throttle_scale_max_);
  if (std::abs(requested - previous) < 1.0e-9)
  {
    RCLCPP_INFO(get_logger(), "Throttle scale remains at limit %.3f", previous);
    return;
  }

  const auto result = set_parameter(rclcpp::Parameter("throttle_scale", requested));
  if (!result.successful)
  {
    RCLCPP_WARN(get_logger(), "Failed to change throttle scale: %s", result.reason.c_str());
  }
}

rcl_interfaces::msg::SetParametersResult TeleopCmdNode::handle_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters)
  {
    if (parameter.get_name() != "throttle_scale")
    {
      continue;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      result.successful = false;
      result.reason = "throttle_scale must be a double";
      return result;
    }
    const double value = parameter.as_double();
    if (!std::isfinite(value) || value < throttle_scale_min_ || value > throttle_scale_max_)
    {
      result.successful = false;
      result.reason = "throttle_scale must be within configured min/max";
      return result;
    }
    throttle_scale_.store(value);
    RCLCPP_INFO(get_logger(), "Throttle scale changed to %.3f", value);
  }
  return result;
}

void TeleopCmdNode::handle_joy(const sensor_msgs::msg::Joy & joy)
{
  jetpilot_msgs::msg::ControlCommand cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";

  const bool deadman_pressed =
    deadman_button_ < 0 || (has_button(joy, deadman_button_) && joy.buttons[deadman_button_] != 0);
  if (deadman_pressed)
  {
    const double steering = has_axis(joy, steering_axis_)
                              ? apply_deadzone(joy.axes[steering_axis_]) * steering_scale_
                              : 0.0;
    cmd.steering = static_cast<float>(std::clamp(steering, -1.0, 1.0));
    cmd.throttle = static_cast<float>(
      normalized_trigger(joy, throttle_axis_, throttle_scale_.load(), throttle_trigger_min_,
                         throttle_trigger_max_, throttle_trigger_inverted_));
    cmd.reverse = static_cast<float>(normalized_trigger(joy, reverse_axis_, reverse_scale_,
                                                        reverse_trigger_min_, reverse_trigger_max_,
                                                        reverse_trigger_inverted_));
    cmd.brake = has_button(joy, brake_button_) && joy.buttons[brake_button_] != 0
                  ? static_cast<float>(brake_value_)
                  : 0.0F;
  }
  else
  {
    cmd.steering = 0.0F;
    cmd.throttle = 0.0F;
    cmd.brake = 0.0F;
    cmd.reverse = 0.0F;
  }

  cmd_pub_->publish(cmd);
}

}  // namespace jetpilot_teleop_tools
