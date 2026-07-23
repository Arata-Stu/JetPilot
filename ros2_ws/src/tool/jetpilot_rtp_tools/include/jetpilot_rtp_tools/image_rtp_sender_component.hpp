#pragma once

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace jetpilot_rtp_tools
{

class ImageRtpSenderComponentTestAccess;

class ImageRtpSenderComponent : public rclcpp::Node
{
public:
  explicit ImageRtpSenderComponent(const rclcpp::NodeOptions & options);
  ~ImageRtpSenderComponent() override;

private:
  friend class ImageRtpSenderComponentTestAccess;

  struct ImageFormat
  {
    std::string gst_format;
    std::size_t source_bytes_per_pixel;
    std::size_t gst_bytes_per_pixel;
    bool normalize_to_gray8{false};
  };

  struct UdpSinkStats
  {
    bool available{false};
    std::uint64_t packets_sent{0};
    std::uint64_t bytes_sent{0};
  };

  static ImageFormat image_format_from_encoding(const std::string & encoding);
  static GstPadProbeReturn udp_sink_probe_callback(
    GstPad * pad, GstPadProbeInfo * info, gpointer user_data);

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void status_callback();
  bool start_pipeline(const sensor_msgs::msg::Image & msg, const ImageFormat & format);
  void poll_bus();
  std::string build_pipeline_description() const;
  std::string select_encoder(const std::string & codec) const;
  bool has_gst_element(const std::string & name) const;
  UdpSinkStats read_udp_sink_stats() const;
  bool status_log_enabled() const;
  bool copy_image_to_buffer(
    const sensor_msgs::msg::Image & msg,
    const ImageFormat & format,
    GstBuffer * buffer);
  void stop_pipeline();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  mutable std::mutex pipeline_mutex_;
  GstElement * pipeline_{nullptr};
  GstElement * appsrc_{nullptr};
  GstElement * udp_sink_{nullptr};
  GstPad * udp_sink_pad_{nullptr};
  gulong udp_sink_probe_id_{0};
  std::uint64_t frame_index_{0};
  std::uint64_t pushed_frames_{0};
  std::uint64_t dropped_frames_{0};
  std::uint64_t last_reported_pushed_frames_{0};
  std::uint64_t last_reported_udp_packets_{0};
  std::uint64_t bus_error_count_{0};
  std::atomic<std::uint64_t> sink_packets_{0};
  std::atomic<std::uint64_t> sink_bytes_{0};
  bool pipeline_started_{false};
  bool thermal_range_initialized_{false};
  double thermal_low_{0.0};
  double thermal_high_{65535.0};
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
