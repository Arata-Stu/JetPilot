#include "jetpilot_rtp_tools/image_rtp_sender_component.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cinttypes>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace jetpilot_rtp_tools
{
namespace
{
std::once_flag gst_init_flag;

std::string trim_copy(const std::string & value)
{
  const auto first = std::find_if_not(
    value.begin(), value.end(), [](const unsigned char c) {
      return std::isspace(c) != 0;
    });
  const auto last = std::find_if_not(
    value.rbegin(), value.rend(), [](const unsigned char c) {
      return std::isspace(c) != 0;
    }).base();
  return first < last ? std::string(first, last) : std::string{};
}

bool contains_whitespace_or_control(const std::string & value)
{
  return std::any_of(value.begin(), value.end(), [](const unsigned char c) {
    return std::isspace(c) != 0 || std::iscntrl(c) != 0;
  });
}

bool is_loopback_host(const std::string & value)
{
  std::string lower = value;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return lower == "localhost" || lower == "::1" || lower == "[::1]" ||
         lower.rfind("127.", 0) == 0;
}

int positive_or_default(const int value, const int default_value)
{
  return value > 0 ? value : default_value;
}
}  // namespace

ImageRtpSenderComponent::ImageRtpSenderComponent(const rclcpp::NodeOptions & options)
: Node("image_rtp_sender", options)
{
  std::call_once(gst_init_flag, []() {
    gst_init(nullptr, nullptr);
  });

  image_topic_ = declare_parameter<std::string>("image_topic", "/realsense/color/image_raw");
  host_ = trim_copy(declare_parameter<std::string>("host", ""));
  port_ = declare_parameter<int>("port", 5004);
  codec_ = declare_parameter<std::string>("codec", "h264");
  encoder_ = declare_parameter<std::string>("encoder", "auto");
  fps_ = positive_or_default(declare_parameter<int>("fps", 60), 60);
  bitrate_ = positive_or_default(declare_parameter<int>("bitrate", 4000000), 4000000);
  gop_ = positive_or_default(declare_parameter<int>("gop", fps_), fps_);
  mtu_ = positive_or_default(declare_parameter<int>("mtu", 1200), 1200);
  payload_ = positive_or_default(declare_parameter<int>("payload", 96), 96);
  h264_encoder_extra_ =
    declare_parameter<std::string>("h264_encoder_extra", "num-B-Frames=0 preset-level=1");
  h265_encoder_extra_ =
    declare_parameter<std::string>("h265_encoder_extra", "num-B-Frames=0 preset-level=1");

  if (host_.empty()) {
    throw std::invalid_argument(
            "RTP parameter 'host' is required; set it to the notebook receiver IP address");
  }
  if (contains_whitespace_or_control(host_)) {
    throw std::invalid_argument("RTP parameter 'host' must not contain whitespace");
  }
  if (port_ < 1 || port_ > 65535) {
    throw std::invalid_argument("RTP parameter 'port' must be between 1 and 65535");
  }
  if (is_loopback_host(host_)) {
    RCLCPP_WARN(
      get_logger(),
      "RTP destination %s is loopback-only; a remote notebook will not receive this stream",
      host_.c_str());
  }

  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, qos,
    std::bind(&ImageRtpSenderComponent::image_callback, this, std::placeholders::_1));
  status_timer_ = create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&ImageRtpSenderComponent::status_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "RTP sender waiting for %s, destination=%s:%d, codec=%s, encoder=%s",
    image_topic_.c_str(), host_.c_str(), port_, codec_.c_str(), encoder_.c_str());
}

ImageRtpSenderComponent::~ImageRtpSenderComponent()
{
  stop_pipeline();
}

ImageRtpSenderComponent::ImageFormat ImageRtpSenderComponent::image_format_from_encoding(
  const std::string & encoding)
{
  namespace enc = sensor_msgs::image_encodings;

  if (encoding == enc::RGB8) {
    return {"RGB", 3};
  }
  if (encoding == enc::BGR8) {
    return {"BGR", 3};
  }
  if (encoding == enc::RGBA8) {
    return {"RGBA", 4};
  }
  if (encoding == enc::BGRA8) {
    return {"BGRA", 4};
  }
  if (encoding == enc::MONO8) {
    return {"GRAY8", 1};
  }

  throw std::invalid_argument("unsupported image encoding: " + encoding);
}

bool ImageRtpSenderComponent::has_gst_element(const std::string & name) const
{
  GstElementFactory * factory = gst_element_factory_find(name.c_str());
  if (factory == nullptr) {
    return false;
  }
  gst_object_unref(factory);
  return true;
}

std::string ImageRtpSenderComponent::select_encoder(const std::string & codec) const
{
  if (encoder_ != "auto") {
    return encoder_;
  }

  if (codec == "h264") {
    if (has_gst_element("nvv4l2h264enc")) {
      return "nvv4l2h264enc";
    }
    if (has_gst_element("x264enc")) {
      return "x264enc";
    }
  }

  if (codec == "h265") {
    if (has_gst_element("nvv4l2h265enc")) {
      return "nvv4l2h265enc";
    }
    if (has_gst_element("x265enc")) {
      return "x265enc";
    }
  }

  return "";
}

std::string ImageRtpSenderComponent::build_pipeline_description() const
{
  std::ostringstream pipeline;
  pipeline
    << "appsrc name=src "
    << "! videoconvert "
    << "! video/x-raw,format=RGB "
    << "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ";

  if (codec_ == "raw") {
    pipeline
      << "! rtpvrawpay pt=" << payload_ << " mtu=" << mtu_ << " ";
  } else if (codec_ == "mjpeg") {
    pipeline
      << "! videoconvert "
      << "! video/x-raw,format=I420 "
      << "! jpegenc quality=80 "
      << "! rtpjpegpay pt=26 mtu=" << mtu_ << " ";
  } else if (codec_ == "h264") {
    const auto encoder = select_encoder(codec_);
    if (encoder == "nvv4l2h264enc") {
      pipeline
        << "! videoconvert "
        << "! video/x-raw,format=NV12 "
        << "! nvvidconv "
        << "! video/x-raw(memory:NVMM),format=NV12 "
        << "! nvv4l2h264enc bitrate=" << bitrate_
        << " control-rate=1 iframeinterval=" << gop_ << " "
        << h264_encoder_extra_ << " "
        << "! h264parse config-interval=-1 "
        << "! video/x-h264,alignment=au,stream-format=byte-stream "
        << "! rtph264pay pt=" << payload_ << " config-interval=1 mtu=" << mtu_ << " ";
    } else if (encoder == "x264enc") {
      pipeline
        << "! videoconvert "
        << "! video/x-raw,format=I420 "
        << "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << (bitrate_ / 1000)
        << " key-int-max=" << gop_ << " bframes=0 byte-stream=true "
        << "! h264parse config-interval=-1 "
        << "! video/x-h264,alignment=au,stream-format=byte-stream "
        << "! rtph264pay pt=" << payload_ << " config-interval=1 mtu=" << mtu_ << " ";
    } else {
      throw std::runtime_error("No H.264 encoder found");
    }
  } else if (codec_ == "h265") {
    const auto encoder = select_encoder(codec_);
    if (encoder == "nvv4l2h265enc") {
      pipeline
        << "! videoconvert "
        << "! video/x-raw,format=NV12 "
        << "! nvvidconv "
        << "! video/x-raw(memory:NVMM),format=NV12 "
        << "! nvv4l2h265enc bitrate=" << bitrate_
        << " control-rate=1 iframeinterval=" << gop_ << " "
        << h265_encoder_extra_ << " "
        << "! h265parse config-interval=-1 "
        << "! video/x-h265,alignment=au,stream-format=byte-stream "
        << "! rtph265pay pt=" << payload_ << " config-interval=1 mtu=" << mtu_ << " ";
    } else if (encoder == "x265enc") {
      pipeline
        << "! videoconvert "
        << "! video/x-raw,format=I420 "
        << "! x265enc tune=zerolatency speed-preset=ultrafast bitrate=" << (bitrate_ / 1000)
        << " key-int-max=" << gop_ << " option-string=bframes=0 "
        << "! h265parse config-interval=-1 "
        << "! video/x-h265,alignment=au,stream-format=byte-stream "
        << "! rtph265pay pt=" << payload_ << " config-interval=1 mtu=" << mtu_ << " ";
    } else {
      throw std::runtime_error("No H.265 encoder found");
    }
  } else {
    throw std::runtime_error("unsupported codec: " + codec_);
  }

  // gst_parse_launch() is not a shell. Applying a shell-quoted value here makes the quote
  // characters part of the hostname, so host/port are set on the element after parsing.
  pipeline << "! udpsink name=rtp_sink sync=false async=false";

  return pipeline.str();
}

bool ImageRtpSenderComponent::start_pipeline(
  const sensor_msgs::msg::Image & msg, const ImageFormat & format)
{
  std::string pipeline_description;
  try {
    pipeline_description = build_pipeline_description();
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to build RTP pipeline: %s", error.what());
    return false;
  }
  GError * error = nullptr;
  GstElement * pipeline = gst_parse_launch(pipeline_description.c_str(), &error);
  GstElement * appsrc = nullptr;
  GstElement * udp_sink = nullptr;
  GstPad * udp_sink_pad = nullptr;
  gulong udp_sink_probe_id = 0;
  const auto cleanup = [&]() {
    if (pipeline != nullptr) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
    }
    if (udp_sink_pad != nullptr && udp_sink_probe_id != 0) {
      gst_pad_remove_probe(udp_sink_pad, udp_sink_probe_id);
    }
    if (udp_sink_pad != nullptr) {
      gst_object_unref(udp_sink_pad);
    }
    if (udp_sink != nullptr) {
      gst_object_unref(udp_sink);
    }
    if (appsrc != nullptr) {
      gst_object_unref(appsrc);
    }
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
  };

  if (pipeline == nullptr) {
    RCLCPP_ERROR(
      get_logger(), "Failed to create RTP pipeline: %s",
      error != nullptr ? error->message : "unknown error");
    if (error != nullptr) {
      g_error_free(error);
    }
    return false;
  }
  if (error != nullptr) {
    RCLCPP_ERROR(get_logger(), "GStreamer pipeline parse failed: %s", error->message);
    g_error_free(error);
    cleanup();
    return false;
  }

  appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
  if (appsrc == nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to find appsrc in RTP pipeline");
    cleanup();
    return false;
  }

  udp_sink = gst_bin_get_by_name(GST_BIN(pipeline), "rtp_sink");
  if (udp_sink == nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to find udpsink in RTP pipeline");
    cleanup();
    return false;
  }

  g_object_set(
    G_OBJECT(udp_sink),
    "host", host_.c_str(),
    "port", port_,
    nullptr);
  gchar * configured_host = nullptr;
  gchar * configured_clients = nullptr;
  gint configured_port = 0;
  g_object_get(
    G_OBJECT(udp_sink),
    "host", &configured_host,
    "port", &configured_port,
    "clients", &configured_clients,
    nullptr);
  const bool destination_matches =
    configured_host != nullptr && host_ == configured_host && configured_port == port_ &&
    configured_clients != nullptr && configured_clients[0] != '\0';
  if (!destination_matches) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to configure or resolve RTP destination: requested=%s:%d actual=%s:%d clients=%s",
      host_.c_str(), port_, configured_host != nullptr ? configured_host : "<null>",
      configured_port, configured_clients != nullptr ? configured_clients : "<null>");
    g_free(configured_host);
    g_free(configured_clients);
    cleanup();
    return false;
  }
  g_free(configured_host);
  g_free(configured_clients);

  udp_sink_pad = gst_element_get_static_pad(udp_sink, "sink");
  if (udp_sink_pad == nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to find udpsink sink pad in RTP pipeline");
    cleanup();
    return false;
  }
  udp_sink_probe_id = gst_pad_add_probe(
    udp_sink_pad,
    static_cast<GstPadProbeType>(GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST),
    &ImageRtpSenderComponent::udp_sink_probe_callback, this, nullptr);
  if (udp_sink_probe_id == 0) {
    RCLCPP_ERROR(get_logger(), "Failed to install RTP packet diagnostics probe");
    cleanup();
    return false;
  }

  GstCaps * caps = gst_caps_new_simple(
    "video/x-raw",
    "format", G_TYPE_STRING, format.gst_format.c_str(),
    "width", G_TYPE_INT, static_cast<int>(msg.width),
    "height", G_TYPE_INT, static_cast<int>(msg.height),
    "framerate", GST_TYPE_FRACTION, fps_, 1,
    nullptr);
  gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
  gst_caps_unref(caps);

  gst_app_src_set_stream_type(GST_APP_SRC(appsrc), GST_APP_STREAM_TYPE_STREAM);
  g_object_set(
    G_OBJECT(appsrc),
    "is-live", TRUE,
    "format", GST_FORMAT_TIME,
    "block", FALSE,
    "do-timestamp", TRUE,
    "max-buffers", 1,
    "max-bytes", 0,
    nullptr);
  gst_app_src_set_latency(GST_APP_SRC(appsrc), 0, 0);

  const auto state_change = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  if (state_change == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Failed to start RTP pipeline");
    cleanup();
    return false;
  }

  pipeline_ = pipeline;
  appsrc_ = appsrc;
  udp_sink_ = udp_sink;
  udp_sink_pad_ = udp_sink_pad;
  udp_sink_probe_id_ = udp_sink_probe_id;
  pipeline_started_ = true;
  frame_index_ = 0;
  pushed_frames_ = 0;
  dropped_frames_ = 0;
  last_reported_pushed_frames_ = 0;
  last_reported_udp_packets_ = 0;
  bus_error_count_ = 0;
  sink_packets_.store(0, std::memory_order_relaxed);
  sink_bytes_.store(0, std::memory_order_relaxed);
  last_flow_return_ = GST_FLOW_OK;

  RCLCPP_INFO(
    get_logger(), "Started RTP pipeline for destination %s:%d: %s",
    host_.c_str(), port_, pipeline_description.c_str());
  RCLCPP_INFO(
    get_logger(), "RTP input caps: video/x-raw,format=%s,width=%u,height=%u,framerate=%d/1",
    format.gst_format.c_str(), msg.width, msg.height, fps_);
  return true;
}

GstPadProbeReturn ImageRtpSenderComponent::udp_sink_probe_callback(
  GstPad * pad, GstPadProbeInfo * info, gpointer user_data)
{
  (void)pad;
  auto * sender = static_cast<ImageRtpSenderComponent *>(user_data);
  if (sender == nullptr || info == nullptr) {
    return GST_PAD_PROBE_OK;
  }

  std::uint64_t packets = 0;
  std::uint64_t bytes = 0;
  if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) != 0) {
    GstBuffer * buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (buffer != nullptr) {
      packets = 1;
      bytes = gst_buffer_get_size(buffer);
    }
  } else if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER_LIST) != 0) {
    GstBufferList * list = GST_PAD_PROBE_INFO_BUFFER_LIST(info);
    if (list != nullptr) {
      const guint length = gst_buffer_list_length(list);
      packets = length;
      for (guint index = 0; index < length; ++index) {
        GstBuffer * buffer = gst_buffer_list_get(list, index);
        if (buffer != nullptr) {
          bytes += gst_buffer_get_size(buffer);
        }
      }
    }
  }
  sender->sink_packets_.fetch_add(packets, std::memory_order_relaxed);
  sender->sink_bytes_.fetch_add(bytes, std::memory_order_relaxed);
  return GST_PAD_PROBE_OK;
}

ImageRtpSenderComponent::UdpSinkStats ImageRtpSenderComponent::read_udp_sink_stats() const
{
  UdpSinkStats result;
  if (udp_sink_ == nullptr || g_signal_lookup("get-stats", G_OBJECT_TYPE(udp_sink_)) == 0) {
    return result;
  }

  GstStructure * stats = nullptr;
  g_signal_emit_by_name(udp_sink_, "get-stats", host_.c_str(), port_, &stats);
  if (stats == nullptr) {
    return result;
  }

  guint64 packets_sent = 0;
  guint64 bytes_sent = 0;
  const bool has_packets =
    gst_structure_get_uint64(stats, "packets-sent", &packets_sent) != FALSE;
  const bool has_bytes = gst_structure_get_uint64(stats, "bytes-sent", &bytes_sent) != FALSE;
  result.available = has_packets && has_bytes;
  result.packets_sent = packets_sent;
  result.bytes_sent = bytes_sent;
  gst_structure_free(stats);
  return result;
}

void ImageRtpSenderComponent::poll_bus()
{
  if (pipeline_ == nullptr) {
    return;
  }

  GstBus * bus = gst_element_get_bus(pipeline_);
  if (bus == nullptr) {
    return;
  }

  while (GstMessage * message = gst_bus_pop(bus)) {
    switch (GST_MESSAGE_TYPE(message)) {
      case GST_MESSAGE_ERROR: {
        GError * error = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(message, &error, &debug);
        RCLCPP_ERROR(
          get_logger(), "GStreamer error from %s: %s%s%s",
          GST_OBJECT_NAME(message->src),
          error != nullptr ? error->message : "unknown",
          debug != nullptr ? " / " : "",
          debug != nullptr ? debug : "");
        ++bus_error_count_;
        if (error != nullptr) {
          g_error_free(error);
        }
        if (debug != nullptr) {
          g_free(debug);
        }
        break;
      }
      case GST_MESSAGE_WARNING: {
        GError * error = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_warning(message, &error, &debug);
        RCLCPP_WARN(
          get_logger(), "GStreamer warning from %s: %s%s%s",
          GST_OBJECT_NAME(message->src),
          error != nullptr ? error->message : "unknown",
          debug != nullptr ? " / " : "",
          debug != nullptr ? debug : "");
        if (error != nullptr) {
          g_error_free(error);
        }
        if (debug != nullptr) {
          g_free(debug);
        }
        break;
      }
      default:
        break;
    }
    gst_message_unref(message);
  }

  gst_object_unref(bus);
}

void ImageRtpSenderComponent::status_callback()
{
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  poll_bus();
  if (!pipeline_started_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "RTP sender still waiting for image topic %s", image_topic_.c_str());
    return;
  }

  const auto delta = pushed_frames_ - last_reported_pushed_frames_;
  last_reported_pushed_frames_ = pushed_frames_;
  const auto sink_packets = sink_packets_.load(std::memory_order_relaxed);
  const auto sink_bytes = sink_bytes_.load(std::memory_order_relaxed);
  const auto udp_stats = read_udp_sink_stats();
  const auto udp_packets = udp_stats.available ? udp_stats.packets_sent : sink_packets;
  const auto udp_bytes = udp_stats.available ? udp_stats.bytes_sent : sink_bytes;
  const auto udp_packet_delta = udp_packets >= last_reported_udp_packets_ ?
    udp_packets - last_reported_udp_packets_ : udp_packets;
  last_reported_udp_packets_ = udp_packets;
  GstState state = GST_STATE_NULL;
  gst_element_get_state(pipeline_, &state, nullptr, 0);
  RCLCPP_INFO(
    get_logger(),
    "RTP sender status: appsrc_pushed=%" PRIu64 " (+%" PRIu64 "/2s), appsrc_dropped=%"
    PRIu64 ", rtp_packets=%" PRIu64 " (+%" PRIu64 "/2s), rtp_bytes=%" PRIu64
    ", stats_source=%s, last_flow=%s, state=%s, bus_errors=%" PRIu64
    ", destination=%s:%d",
    pushed_frames_, delta, dropped_frames_, udp_packets, udp_packet_delta, udp_bytes,
    udp_stats.available ? "udpsink" : "sink-pad", gst_flow_get_name(last_flow_return_),
    gst_element_state_get_name(state), bus_error_count_, host_.c_str(), port_);
  if (delta > 0 && udp_packet_delta == 0) {
    RCLCPP_WARN(
      get_logger(),
      "Images reached appsrc but no RTP packets reached udpsink in the last 2 seconds; "
      "check the encoder/payloader and GStreamer errors above");
  }
}

bool ImageRtpSenderComponent::copy_image_to_buffer(
  const sensor_msgs::msg::Image & msg,
  const ImageFormat & format,
  GstBuffer * buffer) const
{
  const std::size_t packed_step = static_cast<std::size_t>(msg.width) * format.bytes_per_pixel;
  const std::size_t packed_size = packed_step * static_cast<std::size_t>(msg.height);
  if (msg.step < packed_step) {
    return false;
  }
  if (msg.data.size() < static_cast<std::size_t>(msg.step) * static_cast<std::size_t>(msg.height)) {
    return false;
  }

  GstMapInfo map_info;
  if (!gst_buffer_map(buffer, &map_info, GST_MAP_WRITE)) {
    return false;
  }

  if (map_info.size < packed_size) {
    gst_buffer_unmap(buffer, &map_info);
    return false;
  }

  if (msg.step == packed_step) {
    std::memcpy(map_info.data, msg.data.data(), packed_size);
  } else {
    for (std::uint32_t row = 0; row < msg.height; ++row) {
      const auto * src = msg.data.data() + static_cast<std::size_t>(row) * msg.step;
      auto * dst = map_info.data + static_cast<std::size_t>(row) * packed_step;
      std::memcpy(dst, src, packed_step);
    }
  }

  gst_buffer_unmap(buffer, &map_info);
  return true;
}

void ImageRtpSenderComponent::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  ImageFormat format;
  try {
    format = image_format_from_encoding(msg->encoding);
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Dropping image: %s", error.what());
    return;
  }

  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (!pipeline_started_ && !start_pipeline(*msg, format)) {
    return;
  }

  const std::size_t packed_step = static_cast<std::size_t>(msg->width) * format.bytes_per_pixel;
  const std::size_t packed_size = packed_step * static_cast<std::size_t>(msg->height);
  GstBuffer * buffer = gst_buffer_new_allocate(nullptr, packed_size, nullptr);
  if (buffer == nullptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to allocate GstBuffer");
    return;
  }

  if (!copy_image_to_buffer(*msg, format, buffer)) {
    gst_buffer_unref(buffer);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to copy image into GstBuffer");
    return;
  }

  GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
  GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
  GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(GST_SECOND, 1, fps_);
  ++frame_index_;

  const GstFlowReturn result = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  last_flow_return_ = result;
  if (result != GST_FLOW_OK) {
    ++dropped_frames_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Failed to push RTP frame to GStreamer: %s", gst_flow_get_name(result));
  } else {
    ++pushed_frames_;
  }
  poll_bus();
}

void ImageRtpSenderComponent::stop_pipeline()
{
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (appsrc_ != nullptr) {
    gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
  }
  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }
  if (udp_sink_pad_ != nullptr && udp_sink_probe_id_ != 0) {
    gst_pad_remove_probe(udp_sink_pad_, udp_sink_probe_id_);
  }
  udp_sink_probe_id_ = 0;
  if (udp_sink_pad_ != nullptr) {
    gst_object_unref(udp_sink_pad_);
    udp_sink_pad_ = nullptr;
  }
  if (udp_sink_ != nullptr) {
    gst_object_unref(udp_sink_);
    udp_sink_ = nullptr;
  }
  if (appsrc_ != nullptr) {
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
  }
  if (pipeline_ != nullptr) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  pipeline_started_ = false;
}

}  // namespace jetpilot_rtp_tools

RCLCPP_COMPONENTS_REGISTER_NODE(jetpilot_rtp_tools::ImageRtpSenderComponent)
