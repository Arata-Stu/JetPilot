#include "jetpilot_planning_manager/planning_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_planning_manager::PlanningManagerNode>());
  rclcpp::shutdown();
  return 0;
}
