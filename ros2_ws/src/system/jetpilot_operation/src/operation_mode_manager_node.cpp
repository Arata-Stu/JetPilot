#include <chrono>

#include "jetpilot_operation/operation_mode_manager_node.hpp"

using namespace std::chrono_literals;

namespace jetpilot_operation
{

OperationModeManagerNode::OperationModeManagerNode() : Node("operation_mode_manager_node")
{
  const auto initial_mode = declare_parameter<std::string>("initial_mode", "STOP");
  state_.mode = mode_from_string(initial_mode);
  state_.source = "initial";

  state_pub_ = create_publisher<jetpilot_msgs::msg::OperationModeState>(
    "/operation_mode/state", rclcpp::QoS(1).transient_local().reliable());
  request_sub_ = create_subscription<jetpilot_msgs::msg::OperationModeRequest>(
    "/operation_mode/request", 10,
    [this](const jetpilot_msgs::msg::OperationModeRequest::SharedPtr msg)
    { handle_request(*msg); });
  timer_ = create_wall_timer(100ms, [this]() { publish_state(); });
  publish_state();
}

std::uint8_t OperationModeManagerNode::mode_from_string(const std::string & mode)
{
  if (mode == "AUTO")
  {
    return jetpilot_msgs::msg::OperationModeState::AUTO;
  }
  if (mode == "MANUAL")
  {
    return jetpilot_msgs::msg::OperationModeState::MANUAL;
  }
  if (mode == "PROPO")
  {
    return jetpilot_msgs::msg::OperationModeState::PROPO;
  }
  return jetpilot_msgs::msg::OperationModeState::STOP;
}

const char * OperationModeManagerNode::mode_name(const std::uint8_t mode)
{
  switch (mode)
  {
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

bool OperationModeManagerNode::is_valid_mode(const std::uint8_t mode)
{
  return mode == jetpilot_msgs::msg::OperationModeState::AUTO ||
         mode == jetpilot_msgs::msg::OperationModeState::MANUAL ||
         mode == jetpilot_msgs::msg::OperationModeState::STOP ||
         mode == jetpilot_msgs::msg::OperationModeState::PROPO;
}

void OperationModeManagerNode::handle_request(
  const jetpilot_msgs::msg::OperationModeRequest & request)
{
  if (!is_valid_mode(request.mode))
  {
    RCLCPP_WARN(get_logger(), "Ignoring invalid operation mode request: %u", request.mode);
    return;
  }

  if (state_.mode != request.mode)
  {
    RCLCPP_INFO(get_logger(), "Operation mode changed: %s -> %s", mode_name(state_.mode),
                mode_name(request.mode));
  }
  state_.mode = request.mode;
  state_.source = request.source.empty() ? "request" : request.source;
  publish_state();
}

void OperationModeManagerNode::publish_state()
{
  state_.header.stamp = now();
  state_.header.frame_id = "operation";
  state_pub_->publish(state_);
}

}  // namespace jetpilot_operation
