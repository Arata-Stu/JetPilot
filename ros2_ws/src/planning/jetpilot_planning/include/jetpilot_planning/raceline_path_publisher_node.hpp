#ifndef JETPILOT_PLANNING__RACELINE_PATH_PUBLISHER_NODE_HPP_
#define JETPILOT_PLANNING__RACELINE_PATH_PUBLISHER_NODE_HPP_

#include <filesystem>
#include <optional>
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
  void publish_outputs();
  void publish_invalidation();
  void check_for_reload();
  void fail_closed(const std::string & issue_key, const std::string & message);
  void log_reload_success(bool content_changed);

  RacelineData raceline_;
  RacelineCsvLimits limits_;
  std::filesystem::path raceline_root_;
  std::filesystem::path raceline_csv_;
  std::optional<RacelineFileSignature> loaded_signature_;
  std::optional<RacelineFileSignature> rejected_signature_;
  std::string output_topic_;
  std::string trajectory_output_topic_;
  std::string frame_id_;
  std::string line_id_;
  std::string line_name_;
  std::string last_reload_issue_key_;
  bool closed_{true};
  bool publish_typed_trajectory_{true};
  bool source_available_{false};
  double reload_interval_sec_{0.5};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<jetpilot_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr reload_timer_;
};

}  // namespace jetpilot_planning

#endif  // JETPILOT_PLANNING__RACELINE_PATH_PUBLISHER_NODE_HPP_
