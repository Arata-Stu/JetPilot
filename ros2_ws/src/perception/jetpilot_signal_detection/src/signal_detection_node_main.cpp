#include "jetpilot_signal_detection/signal_detection_node.hpp"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_signal_detection::SignalDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
