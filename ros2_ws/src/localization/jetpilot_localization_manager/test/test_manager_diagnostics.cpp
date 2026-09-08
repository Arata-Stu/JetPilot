#include <chrono>
#include <functional>
#include <thread>
#include "gtest/gtest.h"
#include "jetpilot_localization_manager/localization_manager_node.hpp"

TEST(ManagerDiagnostics, ManualInputAndTrackingAreDistinctFromTfHealth)
{
  rclcpp::init(0, nullptr);
  {
    using Array = diagnostic_msgs::msg::DiagnosticArray;
    using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
    auto manager = std::make_shared<jetpilot_localization_manager::LocalizationManagerNode>();
    auto peer = std::make_shared<rclcpp::Node>("manager_diagnostics_test");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(manager);
    executor.add_node(peer);
    Array latest;
    auto status_sub = peer->create_subscription<Array>("/localization/manager/diagnostics",
      rclcpp::QoS(1).reliable().transient_local(),
      [&](Array::SharedPtr message) { latest = *message; });
    auto hint_sub = peer->create_subscription<Pose>("/localization/pose_hint", 10,
      [](Pose::SharedPtr) {});
    auto manual = peer->create_publisher<Pose>("/initialpose", 10);
    auto diagnostics = peer->create_publisher<Array>("/localization/vslam/diagnostics", 10);
    const auto wait = [&](const std::function<bool()> & condition) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (std::chrono::steady_clock::now() < deadline)
      {
        executor.spin_some();
        if (condition()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return false;
    };
    const auto value = [&](const std::string & component, const std::string & key) {
      for (const auto & status : latest.status)
        if (status.name == "Localization/" + component)
          for (const auto & entry : status.values)
            if (entry.key == key) return entry.value;
      return std::string{};
    };
    EXPECT_TRUE(wait([&] { return manual->get_subscription_count() > 0 &&
      diagnostics->get_subscription_count() > 0 && !latest.status.empty(); }));

    Pose pose;
    pose.header.frame_id = "wrong_frame";
    pose.pose.pose.orientation.w = 1.0;
    manual->publish(pose);
    EXPECT_TRUE(wait([&] { return value("Manager", "last_input_result").find("rejected:") == 0; }));
    EXPECT_EQ(value("Manager", "last_input_source"), "manual");

    Array report;
    report.header.stamp = peer->now();
    diagnostic_msgs::msg::DiagnosticStatus raw;
    raw.hardware_id = "visual_slam";
    diagnostic_msgs::msg::KeyValue localized, tracking;
    localized.key = "localized_in_exist_map";
    localized.value = "No";
    tracking.key = "vo_status";
    tracking.value = "OK";
    raw.values = {localized, tracking};
    report.status.push_back(raw);
    diagnostics->publish(report);
    EXPECT_TRUE(wait([&] { return value("VSLAM", "localized_in_exist_map") == "false"; }));
    pose.header.frame_id = "map";
    manual->publish(pose);
    EXPECT_TRUE(wait([&] { return value("Manager", "state") == "awaiting_vslam"; }));
    EXPECT_EQ(value("VGL", "stage"), "bypassed_manual");
    EXPECT_EQ(value("Manager", "last_input_result"), "accepted");
    EXPECT_EQ(value("Manager", "localization_state_allows_control"), "false");

    report.header.stamp = peer->now();
    report.status[0].values[0].value = "Yes";
    diagnostics->publish(report);
    EXPECT_TRUE(wait([&] { return value("Manager", "state") == "localized"; }));
    // A successful diagnostic must not be misrepresented as an observed TF update.
    EXPECT_EQ(value("TF", "map_to_odom_fresh"), "false");
    EXPECT_EQ(value("TF", "map_to_odom_updated_after_hint"), "false");
    EXPECT_EQ(value("TF", "pose_correction_verified"), "unknown");

    auto joy = peer->create_publisher<std_msgs::msg::Bool>("/localization/trigger", 10);
    EXPECT_TRUE(wait([&] { return joy->get_subscription_count() > 0; }));
    std_msgs::msg::Bool trigger;
    trigger.data = true;
    joy->publish(trigger);
    EXPECT_TRUE(wait([&] { return value("Manager", "last_input_source") == "joy_topic"; }));
    EXPECT_EQ(value("Manager", "state"), "waiting_for_manual");
    EXPECT_EQ(value("VGL", "stage"), "disabled");
    EXPECT_EQ(value("Manager", "localization_state_allows_control"), "false");
  }
  rclcpp::shutdown();
}
