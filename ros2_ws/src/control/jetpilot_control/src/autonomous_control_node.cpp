#include <algorithm>
#include <chrono>
#include <memory>

#include "jetpilot_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"

class AutonomousControlNode : public rclcpp::Node
{
public:
  AutonomousControlNode()
  : Node("autonomous_control_node")
  {
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 20.0));
    steering_ = std::clamp(declare_parameter<double>("steering", 0.0), -1.0, 1.0);
    throttle_ = std::clamp(declare_parameter<double>("throttle", 0.0), 0.0, 1.0);
    brake_ = std::clamp(declare_parameter<double>("brake", 1.0), 0.0, 1.0);
    reverse_ = std::clamp(declare_parameter<double>("reverse", 0.0), 0.0, 1.0);

    const auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    cmd_pub_ = create_publisher<jetpilot_msgs::msg::ControlCommand>("/auto/control_cmd", qos_cmd);
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_command(); });
  }

private:
  void publish_command()
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

  double publish_rate_hz_;
  double steering_;
  double throttle_;
  double brake_;
  double reverse_;
  rclcpp::Publisher<jetpilot_msgs::msg::ControlCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousControlNode>());
  rclcpp::shutdown();
  return 0;
}
