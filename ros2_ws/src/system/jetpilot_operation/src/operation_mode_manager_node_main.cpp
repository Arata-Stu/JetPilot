#include <memory>

#include "jetpilot_operation/operation_mode_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_operation::OperationModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
