#include "jetpilot_e2e_inference/e2e_trajectory_decoder_node.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"

namespace jetpilot_e2e_inference
{
namespace
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

std_msgs::msg::Header make_header(
  const nvidia::isaac_ros::nitros::NitrosTensorListView & message,
  const std::string & frame_id)
{
  std_msgs::msg::Header header;
  header.stamp.sec = message.GetTimestampSeconds();
  header.stamp.nanosec = message.GetTimestampNanoseconds();
  header.frame_id = frame_id;
  return header;
}

geometry_msgs::msg::Quaternion orientation(const Point2D & first, const Point2D & second)
{
  const double yaw = std::atan2(second.y - first.y, second.x - first.x);
  geometry_msgs::msg::Quaternion result;
  result.z = std::sin(yaw * 0.5);
  result.w = std::cos(yaw * 0.5);
  return result;
}

}  // namespace

E2ETrajectoryDecoderNode::E2ETrajectoryDecoderNode(const rclcpp::NodeOptions & options)
: Node("e2e_trajectory_decoder", options)
{
  output_tensor_name_ = declare_parameter<std::string>("output_tensor_name", "output_tensor");
  const auto trajectory_points = declare_parameter<int64_t>("trajectory_points", 10);
  trajectory_scale_m_ = declare_parameter<double>("trajectory_scale_m", 5.0);
  frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
  target_speed_mps_ = declare_parameter<double>("target_speed_mps", 0.8);
  max_point_distance_m_ = declare_parameter<double>("max_point_distance_m", 8.0);
  const auto nitros_tensor_format = declare_parameter<std::string>(
    "nitros_tensor_format",
    nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name);

  if (trajectory_points <= 0) {
    throw std::invalid_argument("trajectory_points must be positive");
  }
  if (trajectory_scale_m_ <= 0.0 || max_point_distance_m_ <= 0.0) {
    throw std::invalid_argument("trajectory_scale_m and max_point_distance_m must be positive");
  }
  if (frame_id_.empty()) {
    throw std::invalid_argument("frame_id must not be empty");
  }
  trajectory_points_ = static_cast<std::size_t>(trajectory_points);

  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory", output_qos);
  target_speed_pub_ = create_publisher<std_msgs::msg::Float32>("target_speed", output_qos);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("planning_ready", output_qos);
  tensor_sub_ = std::make_shared<NitrosSubscriber>(
    this, "tensor_sub", nitros_tensor_format,
    std::bind(&E2ETrajectoryDecoderNode::on_tensor, this, std::placeholders::_1),
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig{}, rclcpp::QoS(10));
}

void E2ETrajectoryDecoderNode::on_tensor(const TensorListView & message)
{
  try {
    const auto tensor = message.GetNamedTensor(output_tensor_name_);
    if (tensor.GetElementType() != nvidia::gxf::PrimitiveType::kFloat32 ||
      tensor.GetBytesPerElement() != sizeof(float))
    {
      RCLCPP_WARN(
        get_logger(), "Trajectory tensor '%s' is not float32", output_tensor_name_.c_str());
      publish_ready(false);
      return;
    }

    const std::size_t required_values = trajectory_points_ * 2U;
    if (tensor.GetElementCount() < required_values) {
      RCLCPP_WARN(
        get_logger(), "Trajectory tensor '%s' has %lu values; expected at least %lu",
        output_tensor_name_.c_str(), static_cast<unsigned long>(tensor.GetElementCount()),
        static_cast<unsigned long>(required_values));
      publish_ready(false);
      return;
    }

    std::vector<float> values(required_values);
    const auto cuda_status = cudaMemcpy(
      values.data(), tensor.GetBuffer(), values.size() * sizeof(float), cudaMemcpyDefault);
    if (cuda_status != cudaSuccess) {
      RCLCPP_ERROR(
        get_logger(), "Failed to copy trajectory tensor from CUDA: %s",
        cudaGetErrorString(cuda_status));
      publish_ready(false);
      return;
    }

    std::vector<Point2D> points;
    points.reserve(trajectory_points_ + 1U);
    points.push_back(Point2D{});
    for (std::size_t index = 0U; index < trajectory_points_; ++index) {
      const Point2D point{
        static_cast<double>(values[index * 2U]) * trajectory_scale_m_,
        static_cast<double>(values[index * 2U + 1U]) * trajectory_scale_m_};
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        std::hypot(point.x, point.y) > max_point_distance_m_)
      {
        RCLCPP_WARN(get_logger(), "Rejected non-finite or out-of-bounds trajectory");
        publish_ready(false);
        return;
      }
      points.push_back(point);
    }

    nav_msgs::msg::Path path;
    path.header = make_header(message, frame_id_);
    path.poses.reserve(points.size());
    for (std::size_t index = 0U; index < points.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[index].x;
      pose.pose.position.y = points[index].y;
      const auto & previous = points[index == 0U ? 0U : index - 1U];
      const auto & neighbor = points[std::min(index + 1U, points.size() - 1U)];
      pose.pose.orientation = orientation(previous, neighbor);
      path.poses.push_back(std::move(pose));
    }

    std_msgs::msg::Float32 target_speed;
    target_speed.data = static_cast<float>(std::max(0.0, target_speed_mps_));
    trajectory_pub_->publish(path);
    target_speed_pub_->publish(target_speed);
    publish_ready(true);
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "Failed to decode trajectory tensor: %s", error.what());
    publish_ready(false);
  }
}

void E2ETrajectoryDecoderNode::publish_ready(const bool ready)
{
  std_msgs::msg::Bool message;
  message.data = ready;
  ready_pub_->publish(message);
}

}  // namespace jetpilot_e2e_inference

RCLCPP_COMPONENTS_REGISTER_NODE(jetpilot_e2e_inference::E2ETrajectoryDecoderNode)
