#include "jetpilot_signal_detection/signal_detection_node.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace jetpilot_signal_detection
{

SignalDetectionNode::SignalDetectionNode() : Node("signal_detection_node")
{
  left_class_ = declare_parameter<std::string>("left_class", "arrow_left");
  straight_class_ = declare_parameter<std::string>("straight_class", "arrow_straight");
  right_class_ = declare_parameter<std::string>("right_class", "arrow_right");
  minimum_confidence_ = declare_parameter<double>("minimum_confidence", 0.60);
  detection_timeout_s_ = declare_parameter<double>("detection_timeout_s", 0.50);
  const auto window = declare_parameter<int>("vote_window_size", 5);
  const auto minimum_votes = declare_parameter<int>("minimum_votes", 3);
  if (left_class_.empty() || straight_class_.empty() || right_class_.empty() ||
      minimum_confidence_ < 0.0 || minimum_confidence_ > 1.0 || detection_timeout_s_ <= 0.0 ||
      window <= 0 || minimum_votes <= 0)
  {
    throw std::invalid_argument("invalid signal detection parameters");
  }
  filter_ = std::make_unique<SignalVoteFilter>(static_cast<std::size_t>(window),
                                               static_cast<std::size_t>(minimum_votes));
  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  junctions_sub_ = create_subscription<jetpilot_msgs::msg::JunctionArray>(
    "/hd_map/junctions", latched,
    [this](const auto message) { on_junctions(message); });
  section_sub_ = create_subscription<std_msgs::msg::String>(
    "/localization/current_section", 10, [this](const auto message) { on_section(message); });
  detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "/perception/signal/detections", 10,
    [this](const auto message) { on_detections(message); });
  signal_pub_ = create_publisher<jetpilot_msgs::msg::DirectionSignal>(
    "/perception/direction_signal", latched);
  watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { watchdog(); });
}

void SignalDetectionNode::on_junctions(const jetpilot_msgs::msg::JunctionArray::SharedPtr message)
{
  const auto previous = activation_by_section_.find(current_section_);
  const std::string previous_junction = previous == activation_by_section_.end() ? "" :
    previous->second.junction_id;
  const std::string previous_signal = previous == activation_by_section_.end() ? "" :
    previous->second.signal_id;
  activation_by_section_.clear();
  for (const auto & junction : message->junctions)
  {
    for (const auto & section : junction.activation_section_ids)
      activation_by_section_[section] = {junction.id, junction.signal_id};
  }
  const auto current = activation_by_section_.find(current_section_);
  const std::string current_junction = current == activation_by_section_.end() ? "" :
    current->second.junction_id;
  const std::string current_signal = current == activation_by_section_.end() ? "" :
    current->second.signal_id;
  if (previous_junction != current_junction || previous_signal != current_signal)
  {
    filter_->reset();
    last_detection_at_.reset();
    publish_unknown("map_update");
  }
}

void SignalDetectionNode::on_section(const std_msgs::msg::String::SharedPtr message)
{
  if (current_section_ == message->data) return;
  current_section_ = message->data;
  filter_->reset();
  last_detection_at_.reset();
  publish_unknown();
}

std::uint8_t SignalDetectionNode::direction_for_class(const std::string & class_id) const
{
  if (class_id == left_class_) return jetpilot_msgs::msg::DirectionSignal::LEFT;
  if (class_id == straight_class_) return jetpilot_msgs::msg::DirectionSignal::STRAIGHT;
  if (class_id == right_class_) return jetpilot_msgs::msg::DirectionSignal::RIGHT;
  return jetpilot_msgs::msg::DirectionSignal::UNKNOWN;
}

void SignalDetectionNode::on_detections(
  const vision_msgs::msg::Detection2DArray::SharedPtr message)
{
  if (activation_by_section_.find(current_section_) == activation_by_section_.end()) return;
  std::uint8_t best_direction = jetpilot_msgs::msg::DirectionSignal::UNKNOWN;
  double best_confidence = 0.0;
  std::string best_class;
  for (const auto & detection : message->detections)
  {
    for (const auto & result : detection.results)
    {
      const auto direction = direction_for_class(result.hypothesis.class_id);
      if (direction != jetpilot_msgs::msg::DirectionSignal::UNKNOWN &&
          result.hypothesis.score >= minimum_confidence_ && result.hypothesis.score > best_confidence)
      {
        best_direction = direction;
        best_confidence = result.hypothesis.score;
        best_class = result.hypothesis.class_id;
      }
    }
  }
  last_detection_at_ = std::chrono::steady_clock::now();
  publish_decision(filter_->add({best_direction, best_confidence}), best_class);
}

void SignalDetectionNode::publish_unknown(const std::string & source_class)
{
  publish_decision({}, source_class);
}

void SignalDetectionNode::publish_decision(
  const SignalDecision & decision, const std::string & source_class)
{
  jetpilot_msgs::msg::DirectionSignal output;
  output.header.stamp = now();
  output.direction = decision.direction;
  output.confidence = static_cast<float>(decision.confidence);
  output.stable = decision.stable;
  output.source_class = source_class;
  output.activation_section_id = current_section_;
  const auto activation = activation_by_section_.find(current_section_);
  if (activation != activation_by_section_.end()) output.signal_id = activation->second.signal_id;
  signal_pub_->publish(output);
}

void SignalDetectionNode::watchdog()
{
  if (!last_detection_at_) return;
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - *last_detection_at_).count();
  if (age > detection_timeout_s_)
  {
    filter_->reset();
    last_detection_at_.reset();
    publish_unknown("timeout");
  }
}

}  // namespace jetpilot_signal_detection
