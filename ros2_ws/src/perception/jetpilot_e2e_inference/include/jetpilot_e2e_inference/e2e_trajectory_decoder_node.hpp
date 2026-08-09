#ifndef JETPILOT_E2E_INFERENCE__E2E_TRAJECTORY_DECODER_NODE_HPP_
#define JETPILOT_E2E_INFERENCE__E2E_TRAJECTORY_DECODER_NODE_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_view.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace jetpilot_e2e_inference
{

class E2ETrajectoryDecoderNode : public rclcpp::Node
{
public:
  explicit E2ETrajectoryDecoderNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using TensorListView = nvidia::isaac_ros::nitros::NitrosTensorListView;
  using NitrosSubscriber =
    nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<TensorListView>;

  void on_tensor(const TensorListView & message);
  void publish_ready(bool ready);

  std::string output_tensor_name_;
  std::size_t trajectory_points_{10U};
  double trajectory_scale_m_{5.0};
  std::string frame_id_{"base_link"};
  double target_speed_mps_{0.8};
  double max_point_distance_m_{8.0};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  std::shared_ptr<NitrosSubscriber> tensor_sub_;
};

}  // namespace jetpilot_e2e_inference

#endif  // JETPILOT_E2E_INFERENCE__E2E_TRAJECTORY_DECODER_NODE_HPP_
