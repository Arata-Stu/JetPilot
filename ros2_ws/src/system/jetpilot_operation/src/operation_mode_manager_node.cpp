#include <chrono>
#include <memory>
#include <string>

#include "jetpilot_msgs/msg/operation_mode_request.hpp"
#include "jetpilot_msgs/msg/operation_mode_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class OperationModeManagerNode : public rclcpp::Node
{
public:
  OperationModeManagerNode()
  : Node("operation_mode_manager_node")
  {
    const auto initial_mode = declare_parameter<std::string>("initial_mode", "STOP");
    state_.mode = mode_from_string(initial_mode);
    state_.source = "initial";

    state_pub_ = create_publisher<jetpilot_msgs::msg::OperationModeState>(
      "/operation_mode/state", rclcpp::QoS(1).transient_local().reliable());
    request_sub_ = create_subscription<jetpilot_msgs::msg::OperationModeRequest>(
      "/operation_mode/request", 10,
      [this](const jetpilot_msgs::msg::OperationModeRequest::SharedPtr msg) {
        handle_request(*msg);
      });
    timer_ = create_wall_timer(100ms, [this]() { publish_state(); });
    publish_state();
  }

private:
  static uint8_t mode_from_string(const std::string & mode)
  {
    if (mode == "AUTO") {
      return jetpilot_msgs::msg::OperationModeState::AUTO;
    }
    if (mode == "MANUAL") {
      return jetpilot_msgs::msg::OperationModeState::MANUAL;
    }
    if (mode == "PROPO") {
      return jetpilot_msgs::msg::OperationModeState::PROPO;
    }
    return jetpilot_msgs::msg::OperationModeState::STOP;
  }

  static const char * mode_name(const uint8_t mode)
  {
    switch (mode) {
      case jetpilot_msgs::msg::OperationModeState::AUTO:
        return "AUTO";
      case jetpilot_msgs::msg::OperationModeState::MANUAL:
        return "MANUAL";
      case jetpilot_msgs::msg::OperationModeState::STOP:
        return "STOP";
      case jetpilot_msgs::msg::OperationModeState::PROPO:
        return "PROPO";
      default:
        return "UNKNOWN";
    }
  }

  static bool is_valid_mode(const uint8_t mode)
  {
    return mode == jetpilot_msgs::msg::OperationModeState::AUTO ||
           mode == jetpilot_msgs::msg::OperationModeState::MANUAL ||
           mode == jetpilot_msgs::msg::OperationModeState::STOP ||
           mode == jetpilot_msgs::msg::OperationModeState::PROPO;
  }

  void handle_request(const jetpilot_msgs::msg::OperationModeRequest & request)
  {
    if (!is_valid_mode(request.mode)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid operation mode request: %u", request.mode);
      return;
    }

    if (state_.mode != request.mode) {
      RCLCPP_INFO(
        get_logger(), "Operation mode changed: %s -> %s",
        mode_name(state_.mode), mode_name(request.mode));
    }
    state_.mode = request.mode;
    state_.source = request.source.empty() ? "request" : request.source;
    publish_state();
  }

  void publish_state()
  {
    state_.header.stamp = now();
    state_.header.frame_id = "operation";
    state_pub_->publish(state_);
  }

  jetpilot_msgs::msg::OperationModeState state_;
  rclcpp::Publisher<jetpilot_msgs::msg::OperationModeState>::SharedPtr state_pub_;
  rclcpp::Subscription<jetpilot_msgs::msg::OperationModeRequest>::SharedPtr request_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OperationModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
