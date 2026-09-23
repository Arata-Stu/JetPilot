#ifndef JETPILOT_E2E_INFERENCE__E2E_CONTROL_DECODER_NODE_HPP_
#define JETPILOT_E2E_INFERENCE__E2E_CONTROL_DECODER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "jetpilot_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace jetpilot_e2e_inference
{

class E2EControlDecoderNode : public rclcpp::Node
{
public:
  explicit E2EControlDecoderNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using TensorList = nvidia::isaac_ros::nitros::NitrosTensorList;
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct TensorInputTiming
  {
    SteadyTime received_at;
    double sensor_to_input_ms{0.0};
  };

  struct PipelineTiming
  {
    bool matched{false};
    double sensor_to_input_ms{0.0};
    double input_to_output_ms{0.0};
    double sensor_to_output_ms{0.0};
    std::size_t pending_inputs{0U};
  };

  void on_input_tensor(TensorList::ConstSharedPtr message);
  void on_tensor(TensorList::ConstSharedPtr message);
  PipelineTiming match_input_tensor(
    const TensorList & message, SteadyTime output_received_at);
  void publish_diagnostics(
    const TensorList & message, double callback_ms, double output_interval_ms,
    bool has_output_interval, const PipelineTiming & pipeline_timing);

  std::vector<std::string> section_head_names_;
  std::vector<std::string> section_ids_;
  std::vector<std::int64_t> section_head_indices_;
  std::vector<double> section_throttles_;
  std::size_t generic_head_index_{0U};
  double generic_throttle_{0.2};
  double section_timeout_sec_{0.3};
  double throttle_rise_per_sec_{0.2};
  double steering_rate_per_sec_{3.0};
  double previous_throttle_{0.0};
  double previous_steering_{0.0};
  std::mutex section_mutex_;
  std::string current_section_;
  SteadyTime section_received_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr section_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_head_pub_;
  std::string output_tensor_name_;
  std::vector<std::string> output_fields_;
  double steering_min_{-1.0};
  double steering_max_{1.0};
  double throttle_min_{0.0};
  double throttle_max_{1.0};
  bool fixed_throttle_mode_{false};
  std::atomic<double> fixed_throttle_{0.2};
  std::atomic<double> steering_scale_{1.0};
  std::atomic<double> steering_offset_{0.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  double stale_timeout_sec_{0.2};
  double deadline_ms_{33.3};
  bool enable_pipeline_latency_breakdown_{false};
  std::size_t pipeline_latency_max_pending_{4096U};
  std::uint64_t sequence_{0U};
  std::atomic<std::uint64_t> pipeline_unmatched_outputs_{0U};
  std::atomic<std::uint64_t> pipeline_evicted_inputs_{0U};
  bool has_last_publish_time_{false};
  SteadyTime last_publish_time_;
  std::mutex pipeline_timing_mutex_;
  std::unordered_map<std::int64_t, TensorInputTiming> pipeline_input_timings_;
  std::deque<std::int64_t> pipeline_input_order_;

  rclcpp::Publisher<jetpilot_msgs::msg::ControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<TensorList>::SharedPtr tensor_input_probe_sub_;
  rclcpp::Subscription<TensorList>::SharedPtr tensor_sub_;
};

}  // namespace jetpilot_e2e_inference

#endif  // JETPILOT_E2E_INFERENCE__E2E_CONTROL_DECODER_NODE_HPP_
