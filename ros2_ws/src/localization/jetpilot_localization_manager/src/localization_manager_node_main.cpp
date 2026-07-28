#include <memory>

#include "jetpilot_localization_manager/localization_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_localization_manager::LocalizationManagerNode>());
  rclcpp::shutdown();
  return 0;
}
