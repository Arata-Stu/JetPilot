#include <algorithm>
#include <string>

#include "jetpilot_teleop_tools/teleop_button_manager_node.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace jetpilot_teleop_tools
{

TeleopButtonManagerNode::TeleopButtonManagerNode() : Node("teleop_button_manager_node")
{
  auto_button_ = declare_parameter<int>("auto_button", 0);
  manual_button_ = declare_parameter<int>("manual_button", 3);
  stop_button_ = declare_parameter<int>("stop_button", 1);
  back_button_ = declare_parameter<int>("back_button", 6);
  bag_start_button_ = declare_parameter<int>("bag_start_button", 5);
  bag_stop_button_ = declare_parameter<int>("bag_stop_button", 4);
  steer_offset_inc_button_ = declare_parameter<int>("steer_offset_inc_button", 15);
  steer_offset_dec_button_ = declare_parameter<int>("steer_offset_dec_button", 14);
  steer_offset_inc_axis_ = declare_parameter<int>("steer_offset_inc_axis", -1);
  steer_offset_dec_axis_ = declare_parameter<int>("steer_offset_dec_axis", -1);
  steer_offset_inc_axis_value_ = declare_numeric_parameter("steer_offset_inc_axis_value", 1.0);
  steer_offset_dec_axis_value_ = declare_numeric_parameter("steer_offset_dec_axis_value", -1.0);
  steer_offset_axis_threshold_ = std::clamp(
    declare_numeric_parameter("steer_offset_axis_threshold", 0.5), 0.01, 1.0);
  const int localization_trigger_button = declare_parameter<int>("localization_trigger_button", -1);
  const bool speed_offset_inc_uses_localization_button =
    declare_parameter<bool>("speed_offset_inc_uses_localization_button", false);
  localization_trigger_topic_ =
    declare_parameter<std::string>("localization_trigger_topic", "/localization/trigger");
  if (localization_trigger_topic_.empty())
  {
    RCLCPP_WARN(get_logger(), "localization_trigger_topic is empty; using /localization/trigger");
    localization_trigger_topic_ = "/localization/trigger";
  }
  held_mode_selector_.set_hold_time(declare_numeric_parameter("hold_time_s", 0.1));

  ButtonManagerAssignments button_assignments;
  button_assignments.auto_button = auto_button_;
  button_assignments.manual_button = manual_button_;
  button_assignments.stop_button = stop_button_;
  button_assignments.back_button = back_button_;
  button_assignments.bag_start_button = bag_start_button_;
  button_assignments.bag_stop_button = bag_stop_button_;
  button_assignments.steer_offset_inc_button = steer_offset_inc_button_;
  button_assignments.steer_offset_dec_button = steer_offset_dec_button_;
  const auto localization_conflict =
    find_localization_button_conflict(localization_trigger_button, button_assignments);
  speed_offset_inc_button_.configure(
    localization_trigger_button, speed_offset_inc_uses_localization_button);
  localization_trigger_button_.configure(
    localization_trigger_button,
    !speed_offset_inc_uses_localization_button && !localization_conflict.has_value());
  if (speed_offset_inc_uses_localization_button && localization_trigger_button >= 0)
  {
    RCLCPP_INFO(
      get_logger(),
      "Button %d is assigned to throttle scale increase; localization trigger is disabled",
      localization_trigger_button);
  }
  else if (localization_trigger_button < 0)
  {
    RCLCPP_INFO(get_logger(), "Localization trigger button is disabled");
  }
  else if (localization_conflict)
  {
    const std::string conflicting_parameter(*localization_conflict);
    RCLCPP_ERROR(get_logger(),
                 "localization_trigger_button=%d conflicts with %s; "
                 "localization trigger is disabled",
                 localization_trigger_button, conflicting_parameter.c_str());
  }

  mode_pub_ =
    create_publisher<jetpilot_msgs::msg::OperationModeRequest>("/operation_mode/request", 10);
  bag_pub_ = create_publisher<jetpilot_msgs::msg::BagRequest>("/bag/request", 10);
  steer_offset_inc_pub_ = create_publisher<std_msgs::msg::Bool>("/steer_offset_inc", 10);
  steer_offset_dec_pub_ = create_publisher<std_msgs::msg::Bool>("/steer_offset_dec", 10);
  speed_offset_inc_pub_ = create_publisher<std_msgs::msg::Bool>("/speed_offset_inc", 10);
  localization_trigger_pub_ =
    create_publisher<std_msgs::msg::Bool>(localization_trigger_topic_, 10);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { handle_joy(*msg); });
}

double TeleopButtonManagerNode::declare_numeric_parameter(const std::string & name,
                                                          const double default_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor, false);

  rclcpp::Parameter parameter;
  if (!get_parameter(name, parameter) ||
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    return default_value;
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
  {
    return parameter.as_double();
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    return static_cast<double>(parameter.as_int());
  }

  RCLCPP_WARN(get_logger(), "Parameter '%s' must be numeric; using default %.3f", name.c_str(),
              default_value);
  return default_value;
}

bool TeleopButtonManagerNode::button_pressed(const sensor_msgs::msg::Joy & joy, const int index)
{
  return index >= 0 && static_cast<std::size_t>(index) < joy.buttons.size() &&
         joy.buttons[index] != 0;
}

TeleopButtonManagerNode::HoldState & TeleopButtonManagerNode::state_for(
  std::vector<HoldState> & states, const std::size_t index)
{
  if (states.size() <= index)
  {
    states.resize(index + 1);
  }
  return states[index];
}

bool TeleopButtonManagerNode::pressed_once(std::vector<HoldState> & states,
                                           const std::size_t state_index, const bool pressed)
{
  auto & state = state_for(states, state_index);
  if (!pressed)
  {
    state.pressed = false;
    return false;
  }
  if (!state.pressed)
  {
    state.pressed = true;
    return true;
  }
  return false;
}

void TeleopButtonManagerNode::publish_mode_request(const std::uint8_t mode,
                                                   const std::string & source)
{
  jetpilot_msgs::msg::OperationModeRequest request;
  request.header.stamp = now();
  request.header.frame_id = "joy";
  request.mode = mode;
  request.source = source;
  mode_pub_->publish(request);
}

void TeleopButtonManagerNode::publish_bag_request(const std::uint8_t command,
                                                  const std::string & label)
{
  jetpilot_msgs::msg::BagRequest request;
  request.header.stamp = now();
  request.header.frame_id = "joy";
  request.command = command;
  request.label = label;
  bag_pub_->publish(request);
}

void TeleopButtonManagerNode::publish_bool(
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  publisher->publish(msg);
}

void TeleopButtonManagerNode::handle_joy(const sensor_msgs::msg::Joy & joy)
{
  const auto current_time = now();
  const bool back = button_pressed(joy, back_button_);
  const bool auto_pressed = !back && button_pressed(joy, auto_button_);
  const bool manual_pressed = !back && button_pressed(joy, manual_button_);
  const bool stop_pressed = !back && button_pressed(joy, stop_button_);
  const auto held_mode = held_mode_selector_.update(
    auto_pressed, manual_pressed, stop_pressed, current_time.seconds());

  std::uint8_t requested_mode = jetpilot_msgs::msg::OperationModeRequest::STOP;
  std::string mode_source = "joy_mode_released";
  if (held_mode == HeldMode::AUTO)
  {
    requested_mode = jetpilot_msgs::msg::OperationModeRequest::AUTO;
    mode_source = "joy_auto_held";
  }
  else if (held_mode == HeldMode::MANUAL)
  {
    requested_mode = jetpilot_msgs::msg::OperationModeRequest::MANUAL;
    mode_source = "joy_manual_held";
  }
  else if (stop_pressed)
  {
    mode_source = "joy_stop_hold";
  }
  else if (auto_pressed && manual_pressed)
  {
    mode_source = "joy_mode_conflict";
  }

  if (last_held_mode_request_ != requested_mode)
  {
    publish_mode_request(requested_mode, mode_source);
    last_held_mode_request_ = requested_mode;
  }
  if (pressed_once(states_, 0, button_pressed(joy, bag_start_button_)))
  {
    publish_bag_request(jetpilot_msgs::msg::BagRequest::START, "joy_start");
  }
  if (pressed_once(states_, 1, button_pressed(joy, bag_stop_button_)))
  {
    publish_bag_request(jetpilot_msgs::msg::BagRequest::STOP, "joy_stop");
  }
  const bool steer_offset_inc = button_pressed(joy, steer_offset_inc_button_) ||
    axis_direction_pressed(
    joy.axes, steer_offset_inc_axis_, steer_offset_inc_axis_value_,
    steer_offset_axis_threshold_);
  const bool steer_offset_dec = button_pressed(joy, steer_offset_dec_button_) ||
    axis_direction_pressed(
    joy.axes, steer_offset_dec_axis_, steer_offset_dec_axis_value_,
    steer_offset_axis_threshold_);
  if (pressed_once(states_, 2, steer_offset_inc))
  {
    publish_bool(steer_offset_inc_pub_);
  }
  if (pressed_once(states_, 3, steer_offset_dec))
  {
    publish_bool(steer_offset_dec_pub_);
  }
  if (localization_trigger_button_.update(joy.buttons))
  {
    publish_bool(localization_trigger_pub_);
  }
  if (speed_offset_inc_button_.update(joy.buttons))
  {
    publish_bool(speed_offset_inc_pub_);
  }
}

}  // namespace jetpilot_teleop_tools
