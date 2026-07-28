#include <exception>
#include <memory>

#include "jetpilot_planning/raceline_path_publisher_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<jetpilot_planning::RacelinePathPublisherNode>());
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("raceline_path_publisher"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
