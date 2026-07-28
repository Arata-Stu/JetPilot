#include <memory>

#include "jetpilot_control/autonomous_control_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_control::AutonomousControlNode>());
  rclcpp::shutdown();
  return 0;
}
