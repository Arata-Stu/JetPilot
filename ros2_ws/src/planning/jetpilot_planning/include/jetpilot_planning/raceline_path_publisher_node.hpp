#ifndef JETPILOT_PLANNING__RACELINE_PATH_PUBLISHER_NODE_HPP_
#define JETPILOT_PLANNING__RACELINE_PATH_PUBLISHER_NODE_HPP_

#include <string>

#include "jetpilot_planning/raceline_csv.hpp"
#include "jetpilot_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jetpilot_planning
{

class RacelinePathPublisherNode : public rclcpp::Node
{
public:
  RacelinePathPublisherNode();

private:
  void publish_outputs(const std::string & frame_id, const std::string & line_id,
                       const std::string & line_name, const std::string & source_hash,
                       bool closed);

  RacelineData raceline_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<jetpilot_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
};

}  // namespace jetpilot_planning

#endif  // JETPILOT_PLANNING__RACELINE_PATH_PUBLISHER_NODE_HPP_
