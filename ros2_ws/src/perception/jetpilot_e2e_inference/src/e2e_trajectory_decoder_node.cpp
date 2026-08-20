#include "jetpilot_e2e_inference/e2e_trajectory_decoder_node.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "isaac_ros_nitros/types/cuda_stream_pool.hpp"
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
  const nvidia::isaac_ros::nitros::NitrosTensorList & message,
  const std::string & frame_id)
{
  auto header = message.get_header();
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
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  tensor_sub_ = create_subscription<TensorList>(
    "tensor_sub", rclcpp::QoS(10),
    std::bind(&E2ETrajectoryDecoderNode::on_tensor, this, std::placeholders::_1),
    subscription_options);
}

void E2ETrajectoryDecoderNode::on_tensor(TensorList::ConstSharedPtr message)
{
  try {
    const auto tensor = message->get_tensor_by_name(output_tensor_name_);
    if (!tensor) {
      RCLCPP_WARN(
        get_logger(), "Trajectory tensor '%s' was not found", output_tensor_name_.c_str());
      publish_ready(false);
      return;
    }
    if (tensor->data_type() != nvidia::isaac_ros::nitros::NitrosDataType::kFloat32 ||
      tensor->bytes_per_element() != sizeof(float))
    {
      RCLCPP_WARN(
        get_logger(), "Trajectory tensor '%s' is not float32", output_tensor_name_.c_str());
      publish_ready(false);
      return;
    }

    const std::size_t required_values = trajectory_points_ * 2U;
    if (tensor->element_count() < required_values) {
      RCLCPP_WARN(
        get_logger(), "Trajectory tensor '%s' has %lu values; expected at least %lu",
        output_tensor_name_.c_str(), static_cast<unsigned long>(tensor->element_count()),
        static_cast<unsigned long>(required_values));
      publish_ready(false);
      return;
    }

    std::vector<float> values(required_values);
    auto stream_handle =
      nvidia::isaac_ros::nitros::CudaStreamPool::instance().get_stream_handle();
    auto read_handle = tensor->get_read_handle(stream_handle.get());
    if (read_handle.get_ptr() == nullptr) {
      RCLCPP_ERROR(get_logger(), "Trajectory tensor buffer is null");
      publish_ready(false);
      return;
    }

    auto cuda_status = cudaSuccess;
    const auto storage_type = message->get_storage_type();
    switch (storage_type) {
      case cudaMemoryTypeDevice:
        cuda_status = cudaMemcpyAsync(
          values.data(), read_handle.get_ptr(), values.size() * sizeof(float),
          cudaMemcpyDeviceToHost, stream_handle.get());
        break;
      case cudaMemoryTypeHost:
        cuda_status = cudaStreamSynchronize(stream_handle.get());
        if (cuda_status == cudaSuccess) {
          std::memcpy(
            values.data(), read_handle.get_ptr(), values.size() * sizeof(float));
        }
        break;
      default:
        RCLCPP_ERROR(
          get_logger(), "Unsupported trajectory tensor storage type: %d",
          static_cast<int>(storage_type));
        publish_ready(false);
        return;
    }
    if (cuda_status == cudaSuccess && storage_type == cudaMemoryTypeDevice) {
      cuda_status = cudaStreamSynchronize(stream_handle.get());
    }
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
    path.header = make_header(*message, frame_id_);
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
