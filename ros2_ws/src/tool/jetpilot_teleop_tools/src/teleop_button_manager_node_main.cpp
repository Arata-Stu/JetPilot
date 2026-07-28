#include <memory>

#include "jetpilot_teleop_tools/teleop_button_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_teleop_tools::TeleopButtonManagerNode>());
  rclcpp::shutdown();
  return 0;
}
