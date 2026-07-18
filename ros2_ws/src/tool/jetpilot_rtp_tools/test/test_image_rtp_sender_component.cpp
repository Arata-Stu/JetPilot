#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "jetpilot_rtp_tools/image_rtp_sender_component.hpp"

namespace jetpilot_rtp_tools
{

class ImageRtpSenderComponentTestAccess
{
public:
  static std::string pipeline_description(const ImageRtpSenderComponent & node)
  {
    return node.build_pipeline_description();
  }

  static bool start_raw_pipeline(
    ImageRtpSenderComponent & node, const sensor_msgs::msg::Image & image)
  {
    return node.start_pipeline(image, {"RGB", 3});
  }

  static GstElement * udp_sink(const ImageRtpSenderComponent & node)
  {
    return node.udp_sink_;
  }

  static bool udp_stats_available(const ImageRtpSenderComponent & node)
  {
    return node.read_udp_sink_stats().available;
  }
};

namespace
{

bool has_element(const char * name)
{
  GstElementFactory * factory = gst_element_factory_find(name);
  if (factory == nullptr) {
    return false;
  }
  gst_object_unref(factory);
  return true;
}

rclcpp::NodeOptions options_with_destination(const std::string & host, const int port)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("host", host),
      rclcpp::Parameter("port", port),
      rclcpp::Parameter("codec", std::string("raw")),
      rclcpp::Parameter("fps", 30),
    });
  return options;
}

class ImageRtpSenderComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    gst_init(nullptr, nullptr);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ImageRtpSenderComponentTest, RequiresDestinationHost)
{
  EXPECT_THROW(
    ImageRtpSenderComponent(options_with_destination("", 5004)),
    std::invalid_argument);
}

TEST_F(ImageRtpSenderComponentTest, RejectsInvalidPort)
{
  EXPECT_THROW(
    ImageRtpSenderComponent(options_with_destination("192.0.2.10", 70000)),
    std::invalid_argument);
}

TEST_F(ImageRtpSenderComponentTest, ConfiguresUnquotedUdpDestinationAfterParsing)
{
  const std::vector<const char *> required_elements = {
    "appsrc", "videoconvert", "queue", "rtpvrawpay", "udpsink"};
  for (const auto * element : required_elements) {
    if (!has_element(element)) {
      GTEST_SKIP() << "GStreamer element is unavailable: " << element;
    }
  }

  constexpr char destination[] = "127.0.0.1";
  auto node = std::make_shared<ImageRtpSenderComponent>(
    options_with_destination(destination, 5004));
  const auto description = ImageRtpSenderComponentTestAccess::pipeline_description(*node);
  EXPECT_NE(description.find("udpsink name=rtp_sink"), std::string::npos);
  EXPECT_EQ(description.find("host="), std::string::npos);

  sensor_msgs::msg::Image image;
  image.width = 2;
  image.height = 2;
  image.step = 6;
  image.encoding = "rgb8";
  image.data.resize(12);
  ASSERT_TRUE(ImageRtpSenderComponentTestAccess::start_raw_pipeline(*node, image));

  GstElement * sink = ImageRtpSenderComponentTestAccess::udp_sink(*node);
  ASSERT_NE(sink, nullptr);
  gchar * actual_host = nullptr;
  gchar * clients = nullptr;
  gint actual_port = 0;
  g_object_get(
    G_OBJECT(sink),
    "host", &actual_host,
    "port", &actual_port,
    "clients", &clients,
    nullptr);
  ASSERT_NE(actual_host, nullptr);
  ASSERT_NE(clients, nullptr);
  EXPECT_EQ(std::string(actual_host), destination);
  EXPECT_EQ(actual_port, 5004);
  EXPECT_FALSE(std::string(clients).empty());
  EXPECT_TRUE(ImageRtpSenderComponentTestAccess::udp_stats_available(*node));
  g_free(actual_host);
  g_free(clients);
}

}  // namespace
}  // namespace jetpilot_rtp_tools
