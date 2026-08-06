#ifndef JETPILOT_TELEOP_TOOLS__TELEOP_BUTTON_MANAGER_NODE_HPP_
#define JETPILOT_TELEOP_TOOLS__TELEOP_BUTTON_MANAGER_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "jetpilot_msgs/msg/bag_request.hpp"
#include "jetpilot_msgs/msg/operation_mode_request.hpp"
#include "jetpilot_teleop_tools/button_rising_edge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

namespace jetpilot_teleop_tools
{

class TeleopButtonManagerNode : public rclcpp::Node
{
public:
  TeleopButtonManagerNode();

private:
  struct HoldState
  {
    bool pressed{false};
    bool emitted{false};
    rclcpp::Time since;
  };

  double declare_numeric_parameter(const std::string & name, double default_value);
  static bool button_pressed(const sensor_msgs::msg::Joy & joy, int index);
  static HoldState & state_for(std::vector<HoldState> & states, std::size_t index);
  bool held_once(std::vector<HoldState> & states, std::size_t state_index, bool pressed,
                 const rclcpp::Time & current_time);
  static bool pressed_once(std::vector<HoldState> & states, std::size_t state_index, bool pressed);
  void publish_mode_request(std::uint8_t mode, const std::string & source);
  void publish_bag_request(std::uint8_t command, const std::string & label);
  static void publish_bool(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher);
  void handle_joy(const sensor_msgs::msg::Joy & joy);

  int auto_button_;
  int manual_button_;
  int stop_button_;
  int back_button_;
  int bag_start_button_;
  int bag_stop_button_;
  int steer_offset_inc_button_;
  int steer_offset_dec_button_;
  int steer_offset_inc_axis_;
  int steer_offset_dec_axis_;
  double steer_offset_inc_axis_value_;
  double steer_offset_dec_axis_value_;
  double steer_offset_axis_threshold_;
  std::string localization_trigger_topic_;
  double hold_time_s_;
  std::vector<HoldState> states_;
  ButtonRisingEdge localization_trigger_button_;
  rclcpp::Publisher<jetpilot_msgs::msg::OperationModeRequest>::SharedPtr mode_pub_;
  rclcpp::Publisher<jetpilot_msgs::msg::BagRequest>::SharedPtr bag_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr steer_offset_inc_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr steer_offset_dec_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localization_trigger_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

}  // namespace jetpilot_teleop_tools

#endif  // JETPILOT_TELEOP_TOOLS__TELEOP_BUTTON_MANAGER_NODE_HPP_
