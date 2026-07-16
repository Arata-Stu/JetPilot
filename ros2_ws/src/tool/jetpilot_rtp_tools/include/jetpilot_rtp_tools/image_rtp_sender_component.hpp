#pragma once

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <cstdint>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace jetpilot_rtp_tools
{

class ImageRtpSenderComponent : public rclcpp::Node
{
public:
  explicit ImageRtpSenderComponent(const rclcpp::NodeOptions & options);
  ~ImageRtpSenderComponent() override;

private:
  struct ImageFormat
  {
    std::string gst_format;
    std::size_t bytes_per_pixel;
  };

  static ImageFormat image_format_from_encoding(const std::string & encoding);

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void status_callback();
  bool start_pipeline(const sensor_msgs::msg::Image & msg, const ImageFormat & format);
  void poll_bus();
  std::string build_pipeline_description(
    const sensor_msgs::msg::Image & msg, const ImageFormat & format) const;
  std::string select_encoder(const std::string & codec) const;
  bool has_gst_element(const std::string & name) const;
  bool copy_image_to_buffer(
    const sensor_msgs::msg::Image & msg,
    const ImageFormat & format,
    GstBuffer * buffer) const;
  void stop_pipeline();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  mutable std::mutex pipeline_mutex_;
  GstElement * pipeline_{nullptr};
  GstElement * appsrc_{nullptr};
  std::uint64_t frame_index_{0};
  std::uint64_t pushed_frames_{0};
  std::uint64_t dropped_frames_{0};
  std::uint64_t last_reported_pushed_frames_{0};
  bool pipeline_started_{false};
  GstFlowReturn last_flow_return_{GST_FLOW_OK};

  std::string image_topic_;
  std::string host_;
  std::string codec_;
  std::string encoder_;
  std::string h264_encoder_extra_;
  std::string h265_encoder_extra_;
  int port_;
  int fps_;
  int bitrate_;
  int gop_;
  int mtu_;
  int payload_;
};

}  // namespace jetpilot_rtp_tools
