#include <exception>
#include <memory>

#include "jetpilot_planning/route_lane_selector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<jetpilot_planning::RouteLaneSelectorNode>());
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("route_lane_selector"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
