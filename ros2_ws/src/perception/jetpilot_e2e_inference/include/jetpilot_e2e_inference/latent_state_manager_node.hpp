#ifndef JETPILOT_E2E_INFERENCE__LATENT_STATE_MANAGER_NODE_HPP_
#define JETPILOT_E2E_INFERENCE__LATENT_STATE_MANAGER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "isaac_ros_common/cuda_stream.hpp"
#include "isaac_ros_nitros/types/cuda_memory_pool.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jetpilot_e2e_inference
{

class LatentStateManagerNode : public rclcpp::Node
{
public:
  explicit LatentStateManagerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using TensorList = nvidia::isaac_ros::nitros::NitrosTensorList;

  static std::int64_t timestamp_ns(const TensorList & message);
  void on_rgb_latent(TensorList::ConstSharedPtr message);
  void on_event_tensor(TensorList::ConstSharedPtr message);
  void on_updater_output(TensorList::ConstSharedPtr message);
  void try_dispatch();
  void on_watchdog();
  void publish_diagnostics();

  std::string rgb_latent_tensor_name_{"rgb_latent"};
  std::string state_input_tensor_name_{"state_in"};
  std::string state_output_tensor_name_{"state_out"};
  std::string event_source_tensor_name_{"event_tensor"};
  std::string event_input_tensor_name_{"event_tensor"};
  std::string delta_t_tensor_name_{"delta_t"};
  bool include_delta_t_{true};
  double initial_delta_t_s_{0.001};
  double max_delta_t_s_{0.1};
  double inference_watchdog_ms_{100.0};
  double statistics_interval_s_{1.0};
  bool debug_{false};

  TensorList::ConstSharedPtr current_state_message_;
  TensorList::ConstSharedPtr pending_rgb_message_;
  TensorList::ConstSharedPtr pending_event_message_;
  std::string current_state_tensor_name_;
  std::int64_t current_state_timestamp_ns_{0};
  std::int64_t last_rgb_timestamp_ns_{-1};
  std::int64_t in_flight_timestamp_ns_{0};
  bool inference_in_flight_{false};
  bool watchdog_reported_{false};
  float delta_t_staging_{0.0F};
  std::chrono::steady_clock::time_point in_flight_started_;

  nvidia::isaac_ros::common::CudaStreamPtr cuda_stream_;
  nvidia::isaac_ros::nitros::CUDAMemoryPool scalar_memory_pool_;
  rclcpp::Subscription<TensorList>::SharedPtr rgb_latent_subscription_;
  rclcpp::Subscription<TensorList>::SharedPtr event_tensor_subscription_;
  rclcpp::Subscription<TensorList>::SharedPtr updater_output_subscription_;
  rclcpp::Publisher<TensorList>::SharedPtr updater_input_publisher_;
  rclcpp::Publisher<TensorList>::SharedPtr output_publisher_;
  rclcpp::Publisher<TensorList>::SharedPtr event_feedback_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::uint64_t rgb_received_{0};
  std::uint64_t rgb_applied_{0};
  std::uint64_t rgb_replaced_{0};
  std::uint64_t rgb_stale_{0};
  std::uint64_t event_received_{0};
  std::uint64_t event_replaced_{0};
  std::uint64_t event_stale_{0};
  std::uint64_t inference_dispatched_{0};
  std::uint64_t inference_completed_{0};
  std::uint64_t stale_output_{0};
  std::uint64_t invalid_message_{0};
  std::uint64_t watchdog_count_{0};
  std::uint64_t round_trip_ns_sum_{0};
  std::uint64_t round_trip_ns_max_{0};
  std::uint64_t dispatch_ns_sum_{0};
  std::uint64_t dispatch_ns_max_{0};
};

}  // namespace jetpilot_e2e_inference

#endif  // JETPILOT_E2E_INFERENCE__LATENT_STATE_MANAGER_NODE_HPP_
