#include <algorithm>
#include <cmath>
#include <memory>

#include "jetpilot_msgs/msg/control_command.hpp"
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
    brake_axis_ = declare_parameter<int>("brake_axis", 2);
    deadman_button_ = declare_parameter<int>("deadman_button", 4);
    steering_scale_ = declare_parameter<double>("steering_scale", 1.0);
    throttle_scale_ = declare_parameter<double>("throttle_scale", 1.0);
    brake_scale_ = declare_parameter<double>("brake_scale", 1.0);
    deadzone_ = std::clamp(declare_parameter<double>("deadzone", 0.05), 0.0, 1.0);
    trigger_min_ = declare_parameter<double>("trigger_min", -1.0);
    trigger_max_ = declare_parameter<double>("trigger_max", 1.0);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { handle_joy(*msg); });
    cmd_pub_ = create_publisher<jetpilot_msgs::msg::ControlCommand>("/teleop/control_cmd", 10);
  }

private:
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

  double normalized_trigger(const sensor_msgs::msg::Joy & joy, const int axis, const double scale) const
  {
    if (!has_axis(joy, axis) || trigger_max_ == trigger_min_) {
      return 0.0;
    }
    const double raw = std::clamp(static_cast<double>(joy.axes[axis]), trigger_min_, trigger_max_);
    const double normalized = (trigger_max_ - raw) / (trigger_max_ - trigger_min_);
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
      cmd.throttle = static_cast<float>(normalized_trigger(joy, throttle_axis_, throttle_scale_));
      cmd.brake = static_cast<float>(normalized_trigger(joy, brake_axis_, brake_scale_));
    } else {
      cmd.steering = 0.0F;
      cmd.throttle = 0.0F;
      cmd.brake = 1.0F;
    }

    cmd_pub_->publish(cmd);
  }

  int steering_axis_;
  int throttle_axis_;
  int brake_axis_;
  int deadman_button_;
  double steering_scale_;
  double throttle_scale_;
  double brake_scale_;
  double deadzone_;
  double trigger_min_;
  double trigger_max_;
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
