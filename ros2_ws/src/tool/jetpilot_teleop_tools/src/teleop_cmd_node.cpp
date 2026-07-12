#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "jetpilot_msgs/msg/control_command.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class TeleopCmdNode : public rclcpp::Node
{
public:
  TeleopCmdNode()
  : Node("teleop_cmd_node")
  {
    steering_axis_ = declare_parameter<int>("steering_axis", 0);
    throttle_axis_ = declare_parameter<int>("throttle_axis", 5);
    reverse_axis_ = declare_parameter<int>("reverse_axis", 2);
    brake_button_ = declare_parameter<int>("brake_button", -1);
    deadman_button_ = declare_parameter<int>("deadman_button", 4);
    steering_scale_ = declare_numeric_parameter("steering_scale", 1.0);
    throttle_scale_ = declare_numeric_parameter("throttle_scale", 1.0);
    reverse_scale_ = declare_numeric_parameter("reverse_scale", 1.0);
    brake_value_ = std::clamp(declare_numeric_parameter("brake_value", 1.0), 0.0, 1.0);
    deadzone_ = std::clamp(declare_numeric_parameter("deadzone", 0.05), 0.0, 1.0);
    trigger_min_ = declare_numeric_parameter("trigger_min", -1.0);
    trigger_max_ = declare_numeric_parameter("trigger_max", 1.0);
    throttle_trigger_min_ = declare_numeric_parameter("throttle_trigger_min", trigger_min_);
    throttle_trigger_max_ = declare_numeric_parameter("throttle_trigger_max", trigger_max_);
    throttle_trigger_inverted_ = declare_parameter<bool>("throttle_trigger_inverted", true);
    reverse_trigger_min_ = declare_numeric_parameter("reverse_trigger_min", trigger_min_);
    reverse_trigger_max_ = declare_numeric_parameter("reverse_trigger_max", trigger_max_);
    reverse_trigger_inverted_ = declare_parameter<bool>("reverse_trigger_inverted", true);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { handle_joy(*msg); });
    const auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    cmd_pub_ = create_publisher<jetpilot_msgs::msg::ControlCommand>("/teleop/control_cmd", qos_cmd);
  }

private:
  double declare_numeric_parameter(const std::string & name, const double default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    declare_parameter(name, rclcpp::ParameterType::PARAMETER_NOT_SET, descriptor, false);

    rclcpp::Parameter parameter;
    if (!get_parameter(name, parameter) ||
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      return default_value;
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return parameter.as_double();
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(parameter.as_int());
    }

    RCLCPP_WARN(
      get_logger(), "Parameter '%s' must be numeric; using default %.3f", name.c_str(),
      default_value);
    return default_value;
  }

  static bool has_axis(const sensor_msgs::msg::Joy & joy, const int index)
  {
    return index >= 0 && static_cast<size_t>(index) < joy.axes.size();
  }

  static bool has_button(const sensor_msgs::msg::Joy & joy, const int index)
  {
    return index >= 0 && static_cast<size_t>(index) < joy.buttons.size();
  }

  double apply_deadzone(const double value) const
  {
    return std::abs(value) < deadzone_ ? 0.0 : value;
  }

  double normalized_trigger(
    const sensor_msgs::msg::Joy & joy, const int axis, const double scale,
    const double trigger_min, const double trigger_max, const bool inverted) const
  {
    if (!has_axis(joy, axis) || trigger_max == trigger_min) {
      return 0.0;
    }
    const double raw = std::clamp(static_cast<double>(joy.axes[axis]), trigger_min, trigger_max);
    const double normalized = inverted ?
      (trigger_max - raw) / (trigger_max - trigger_min) :
      (raw - trigger_min) / (trigger_max - trigger_min);
    return std::clamp(normalized * scale, 0.0, 1.0);
  }

  void handle_joy(const sensor_msgs::msg::Joy & joy)
  {
    jetpilot_msgs::msg::ControlCommand cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";

    const bool deadman_pressed =
      deadman_button_ < 0 || (has_button(joy, deadman_button_) && joy.buttons[deadman_button_] != 0);
    if (deadman_pressed) {
      const double steering =
        has_axis(joy, steering_axis_) ? apply_deadzone(joy.axes[steering_axis_]) * steering_scale_ : 0.0;
      cmd.steering = static_cast<float>(std::clamp(steering, -1.0, 1.0));
      cmd.throttle = static_cast<float>(
        normalized_trigger(
          joy, throttle_axis_, throttle_scale_, throttle_trigger_min_, throttle_trigger_max_,
          throttle_trigger_inverted_));
      cmd.reverse = static_cast<float>(
        normalized_trigger(
          joy, reverse_axis_, reverse_scale_, reverse_trigger_min_, reverse_trigger_max_,
          reverse_trigger_inverted_));
      cmd.brake = has_button(joy, brake_button_) && joy.buttons[brake_button_] != 0 ?
        static_cast<float>(brake_value_) : 0.0F;
    } else {
      cmd.steering = 0.0F;
      cmd.throttle = 0.0F;
      cmd.brake = 1.0F;
      cmd.reverse = 0.0F;
    }

    cmd_pub_->publish(cmd);
  }

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopCmdNode>());
  rclcpp::shutdown();
  return 0;
}
