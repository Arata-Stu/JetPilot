#include <memory>
#include <string>
#include <vector>

#include "jetpilot_msgs/msg/bag_request.hpp"
#include "jetpilot_msgs/msg/operation_mode_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class TeleopButtonManagerNode : public rclcpp::Node
{
public:
  TeleopButtonManagerNode()
  : Node("teleop_button_manager_node")
  {
    auto_button_ = declare_parameter<int>("auto_button", 0);
    manual_button_ = declare_parameter<int>("manual_button", 4);
    stop_button_ = declare_parameter<int>("stop_button", 1);
    back_button_ = declare_parameter<int>("back_button", 6);
    bag_start_button_ = declare_parameter<int>("bag_start_button", 0);
    bag_stop_button_ = declare_parameter<int>("bag_stop_button", 1);
    hold_time_s_ = declare_parameter<double>("hold_time_s", 1.0);

    mode_pub_ = create_publisher<jetpilot_msgs::msg::OperationModeRequest>(
      "/operation_mode/request", 10);
    bag_pub_ = create_publisher<jetpilot_msgs::msg::BagRequest>("/bag/request", 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { handle_joy(*msg); });
  }

private:
  struct HoldState
  {
    bool pressed{false};
    bool emitted{false};
    rclcpp::Time since;
  };

  static bool button_pressed(const sensor_msgs::msg::Joy & joy, const int index)
  {
    return index >= 0 && static_cast<size_t>(index) < joy.buttons.size() && joy.buttons[index] != 0;
  }

  HoldState & state_for(std::vector<HoldState> & states, const size_t index)
  {
    if (states.size() <= index) {
      states.resize(index + 1);
    }
    return states[index];
  }

  bool held_once(
    std::vector<HoldState> & states, const size_t state_index, const bool pressed,
    const rclcpp::Time & current_time)
  {
    auto & state = state_for(states, state_index);
    if (!pressed) {
      state.pressed = false;
      state.emitted = false;
      return false;
    }
    if (!state.pressed) {
      state.pressed = true;
      state.emitted = false;
      state.since = current_time;
      return false;
    }
    if (!state.emitted && (current_time - state.since).seconds() >= hold_time_s_) {
      state.emitted = true;
      return true;
    }
    return false;
  }

  void publish_mode_request(const uint8_t mode, const std::string & source)
  {
    jetpilot_msgs::msg::OperationModeRequest request;
    request.header.stamp = now();
    request.header.frame_id = "joy";
    request.mode = mode;
    request.source = source;
    mode_pub_->publish(request);
  }

  void publish_bag_request(const uint8_t command, const std::string & label)
  {
    jetpilot_msgs::msg::BagRequest request;
    request.header.stamp = now();
    request.header.frame_id = "joy";
    request.command = command;
    request.label = label;
    bag_pub_->publish(request);
  }

  void handle_joy(const sensor_msgs::msg::Joy & joy)
  {
    const auto current_time = now();
    const bool back = button_pressed(joy, back_button_);

    if (held_once(states_, 0, !back && button_pressed(joy, auto_button_), current_time)) {
      publish_mode_request(jetpilot_msgs::msg::OperationModeRequest::AUTO, "joy_auto_hold");
    }
    if (held_once(states_, 1, !back && button_pressed(joy, manual_button_), current_time)) {
      publish_mode_request(jetpilot_msgs::msg::OperationModeRequest::MANUAL, "joy_manual_hold");
    }
    if (held_once(states_, 2, !back && button_pressed(joy, stop_button_), current_time)) {
      publish_mode_request(jetpilot_msgs::msg::OperationModeRequest::STOP, "joy_stop_hold");
    }
    if (held_once(states_, 3, back && button_pressed(joy, bag_start_button_), current_time)) {
      publish_bag_request(jetpilot_msgs::msg::BagRequest::START, "joy_start");
    }
    if (held_once(states_, 4, back && button_pressed(joy, bag_stop_button_), current_time)) {
      publish_bag_request(jetpilot_msgs::msg::BagRequest::STOP, "joy_stop");
    }
  }

  int auto_button_;
  int manual_button_;
  int stop_button_;
  int back_button_;
  int bag_start_button_;
  int bag_stop_button_;
  double hold_time_s_;
  std::vector<HoldState> states_;
  rclcpp::Publisher<jetpilot_msgs::msg::OperationModeRequest>::SharedPtr mode_pub_;
  rclcpp::Publisher<jetpilot_msgs::msg::BagRequest>::SharedPtr bag_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopButtonManagerNode>());
  rclcpp::shutdown();
  return 0;
}
