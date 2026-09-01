#include "jetpilot_recovery_planner/breadcrumb_recovery_planner_node.hpp"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_recovery_planner::BreadcrumbRecoveryPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
