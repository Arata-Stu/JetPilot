#include <memory>

#include "jetpilot_teleop_tools/teleop_cmd_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_teleop_tools::TeleopCmdNode>());
  rclcpp::shutdown();
  return 0;
}
