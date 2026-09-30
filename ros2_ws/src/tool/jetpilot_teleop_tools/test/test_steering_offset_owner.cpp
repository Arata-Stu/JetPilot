#include <algorithm>
#include <gtest/gtest.h>
#include "jetpilot_teleop_tools/teleop_cmd_node.hpp"

class SteeringOffsetOwner : public testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(SteeringOffsetOwner, VehicleOwnerClearsSavedJoyTrimAndRejectsRuntimeOffset)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("steering_offset_enabled", false),
    rclcpp::Parameter("steering_offset", -0.30)});
  auto node = std::make_shared<jetpilot_teleop_tools::TeleopCmdNode>(options);
  EXPECT_DOUBLE_EQ(node->get_parameter("steering_offset").as_double(), 0.0);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("steering_offset", 0.1)).successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("steering_offset").as_double(), 0.0);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("steering_offset", 0.0)).successful);
  const auto tuning = node->get_parameter("dynamic_tuning_parameters").as_string_array();
  EXPECT_EQ(std::find(tuning.begin(), tuning.end(), "steering_offset"), tuning.end());
}

TEST_F(SteeringOffsetOwner, DefaultOwnerPreservesOtherVehicleProfiles)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("steering_offset", 0.06)});
  auto node = std::make_shared<jetpilot_teleop_tools::TeleopCmdNode>(options);
  EXPECT_TRUE(node->get_parameter("steering_offset_enabled").as_bool());
  EXPECT_DOUBLE_EQ(node->get_parameter("steering_offset").as_double(), 0.06);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("steering_offset", 0.10)).successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("steering_offset").as_double(), 0.10);
}
