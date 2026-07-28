#include <exception>
#include <memory>

#include "jetpilot_controller/path_tracking_controller_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<jetpilot_controller::PathTrackingControllerNode>());
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("path_tracking_controller_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
