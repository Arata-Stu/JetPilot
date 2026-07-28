#include <memory>

#include "jetpilot_operation/command_mux_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_operation::CommandMuxNode>());
  rclcpp::shutdown();
  return 0;
}
