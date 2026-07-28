#ifndef JETPILOT_OPERATION__OPERATION_MODE_MANAGER_NODE_HPP_
#define JETPILOT_OPERATION__OPERATION_MODE_MANAGER_NODE_HPP_

#include <cstdint>
#include <string>

#include "jetpilot_msgs/msg/operation_mode_request.hpp"
#include "jetpilot_msgs/msg/operation_mode_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jetpilot_operation
{

class OperationModeManagerNode : public rclcpp::Node
{
public:
  OperationModeManagerNode();

private:
  static std::uint8_t mode_from_string(const std::string & mode);
  static const char * mode_name(std::uint8_t mode);
  static bool is_valid_mode(std::uint8_t mode);
  void handle_request(const jetpilot_msgs::msg::OperationModeRequest & request);
  void publish_state();

  jetpilot_msgs::msg::OperationModeState state_;
  rclcpp::Publisher<jetpilot_msgs::msg::OperationModeState>::SharedPtr state_pub_;
  rclcpp::Subscription<jetpilot_msgs::msg::OperationModeRequest>::SharedPtr request_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace jetpilot_operation

#endif  // JETPILOT_OPERATION__OPERATION_MODE_MANAGER_NODE_HPP_
