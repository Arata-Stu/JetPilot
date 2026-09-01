#ifndef JETPILOT_SIGNAL_DETECTION__SIGNAL_DETECTION_NODE_HPP_
#define JETPILOT_SIGNAL_DETECTION__SIGNAL_DETECTION_NODE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "jetpilot_msgs/msg/direction_signal.hpp"
#include "jetpilot_msgs/msg/junction_array.hpp"
#include "jetpilot_signal_detection/signal_vote_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace jetpilot_signal_detection
{
class SignalDetectionNode : public rclcpp::Node
{
public:
  SignalDetectionNode();

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  struct Activation { std::string junction_id; std::string signal_id; };
  void on_junctions(const jetpilot_msgs::msg::JunctionArray::SharedPtr message);
  void on_section(const std_msgs::msg::String::SharedPtr message);
  void on_detections(const vision_msgs::msg::Detection2DArray::SharedPtr message);
  void publish_unknown(const std::string & source_class = "");
  void publish_decision(const SignalDecision & decision, const std::string & source_class);
  std::uint8_t direction_for_class(const std::string & class_id) const;
  void watchdog();

  std::string left_class_;
  std::string straight_class_;
  std::string right_class_;
  double minimum_confidence_{0.60};
  double detection_timeout_s_{0.50};
  std::unique_ptr<SignalVoteFilter> filter_;
  std::unordered_map<std::string, Activation> activation_by_section_;
  std::string current_section_;
  std::optional<SteadyTime> last_detection_at_;
  rclcpp::Subscription<jetpilot_msgs::msg::JunctionArray>::SharedPtr junctions_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr section_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<jetpilot_msgs::msg::DirectionSignal>::SharedPtr signal_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};
}  // namespace jetpilot_signal_detection
#endif
