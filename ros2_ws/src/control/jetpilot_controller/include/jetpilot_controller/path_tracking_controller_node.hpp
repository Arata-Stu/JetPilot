#ifndef JETPILOT_CONTROLLER__PATH_TRACKING_CONTROLLER_NODE_HPP_
#define JETPILOT_CONTROLLER__PATH_TRACKING_CONTROLLER_NODE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jetpilot_controller/kinematic_mpc.hpp"
#include "jetpilot_controller/longitudinal_controller.hpp"
#include "jetpilot_controller/map_pursuit.hpp"
#include "jetpilot_controller/path_tracking_controller.hpp"
#include "jetpilot_controller/pure_pursuit.hpp"
#include "jetpilot_controller/trailing_controller.hpp"
#include "jetpilot_controller/trajectory_speed_profile.hpp"
#include "jetpilot_msgs/msg/control_command.hpp"
#include "jetpilot_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace jetpilot_controller
{

class PathTrackingControllerNode : public rclcpp::Node
{
public:
  PathTrackingControllerNode();

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct StationProjection
  {
    bool valid{false};
    double station_m{0.0};
  };

  void declare_and_read_parameters();
  void create_controller();
  void create_interfaces();
  bool transform_trajectory(const nav_msgs::msg::Path & trajectory, std::vector<Point2d> & points,
                            std::string & reason);
  bool transform_point_to_base(const geometry_msgs::msg::Point & point,
                               const std::string & source_frame, Point2d & transformed_point,
                               std::string & reason);
  StationProjection project_station(const std::vector<Point2d> & path, const Point2d & point,
                                    bool path_closed) const;
  double path_length(const std::vector<Point2d> & path, bool path_closed) const;
  std::optional<double> current_speed(std::string & reason) const;
  std::optional<double> requested_target_speed(std::string & reason) const;
  double apply_steering_rate_limit(double requested);
  void control_cycle();
  TrailingResult apply_trailing_limit(double planned_speed_mps, double current_speed_mps,
                                      const std::vector<Point2d> & path, bool path_closed);
  void publish_safety_stop(const std::string & reason);
  void publish_tracking_visualization(const std::vector<Point2d> & path,
                                      const TrackingResult & tracking, bool reverse_motion,
                                      const std::string & line_id,
                                      const builtin_interfaces::msg::Time & stamp);
  void publish_tracking_stop_visualization(const std::string & reason,
                                           const builtin_interfaces::msg::Time & stamp);
  void publish_state(bool ready, const std::string & message, double current_speed_mps,
                     double target_speed_mps, double steering_command, double curvature,
                     const TrailingResult & trailing);

  std::string algorithm_;
  std::string base_frame_;
  std::string trajectory_topic_;
  std::string trajectory_profile_topic_;
  std::string target_speed_topic_;
  std::string planning_ready_topic_;
  std::string localization_state_topic_;
  std::string odometry_topic_;
  std::string opponent_odometry_topic_;
  std::string command_topic_;
  std::string diagnostics_topic_;
  std::string tracking_markers_topic_;

  double control_rate_hz_{30.0};
  double trajectory_timeout_s_{0.5};
  double odometry_timeout_s_{0.3};
  double target_speed_timeout_s_{0.5};
  double transform_timeout_s_{0.05};
  double transform_max_age_s_{0.5};
  bool require_planning_ready_{true};
  bool require_localization_state_{true};
  double localization_state_timeout_s_{1.5};
  bool require_target_speed_{true};
  double fallback_target_speed_mps_{0.5};
  double max_target_speed_mps_{3.0};
  double goal_tolerance_m_{0.25};
  bool stop_at_path_end_{true};
  double safety_brake_command_{0.0};
  double max_steering_rate_per_s_{4.0};
  double max_lateral_accel_mps2_{2.0};
  double trajectory_speed_lookahead_m_{0.5};
  bool use_typed_trajectory_profiles_{true};
  double opponent_odometry_timeout_s_{0.3};
  PurePursuitParams pure_pursuit_params_;
  MapPursuitParams map_pursuit_params_;
  KinematicMpcParams kinematic_mpc_params_;

  std::unique_ptr<PathTrackingController> lateral_controller_;
  std::unique_ptr<LongitudinalController> longitudinal_controller_;
  std::unique_ptr<TrailingController> trailing_controller_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::optional<nav_msgs::msg::Path> trajectory_;
  std::optional<jetpilot_msgs::msg::Trajectory> trajectory_profile_;
  std::optional<nav_msgs::msg::Odometry> odometry_;
  std::optional<nav_msgs::msg::Odometry> opponent_odometry_;
  std::optional<SteadyTime> trajectory_received_at_;
  std::optional<SteadyTime> trajectory_profile_received_at_;
  std::optional<SteadyTime> odometry_received_at_;
  std::optional<SteadyTime> opponent_odometry_received_at_;
  std::optional<SteadyTime> target_speed_received_at_;
  std::optional<SteadyTime> localization_state_received_at_;
  bool planning_ready_{false};
  bool planning_ready_received_{false};
  bool localization_confirmed_{false};
  double target_speed_mps_{0.0};
  double previous_steering_command_{0.0};
  std::optional<SteadyTime> last_control_at_;
  std::optional<SteadyTime> last_trailing_update_at_;
  std::optional<SteadyTime> last_diagnostic_at_;
  std::string last_diagnostic_message_;
  bool last_ready_{false};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<jetpilot_msgs::msg::Trajectory>::SharedPtr trajectory_profile_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planning_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opponent_odometry_sub_;
  rclcpp::Publisher<jetpilot_msgs::msg::ControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracking_markers_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace jetpilot_controller

#endif  // JETPILOT_CONTROLLER__PATH_TRACKING_CONTROLLER_NODE_HPP_
