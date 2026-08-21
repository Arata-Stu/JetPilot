#ifndef JETPILOT_LOCALIZATION_MANAGER__LOCALIZATION_MANAGER_NODE_HPP_
#define JETPILOT_LOCALIZATION_MANAGER__LOCALIZATION_MANAGER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "jetpilot_localization_manager/pose_validation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace jetpilot_localization_manager
{

class LocalizationManagerNode : public rclcpp::Node
{
public:
  LocalizationManagerNode();

private:
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;
  using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Trigger = std_srvs::srv::Trigger;

  enum class State
  {
    kIdle,
    kWaitingForDependencies,
    kWaitingForTriggerResponse,
    kWaitingForVglPose,
    kRetryBackoff,
    kWaitingToForwardPose,
    kAwaitingVslam,
    kLocalized,
    kLocalizedUnconfirmed,
    kUnlocalized,
    kWaitingForManual,
    kMapOriginRestartRequired,
  };

  double nonnegative_parameter(const std::string & name, double default_value);
  static const char * state_name(State state);
  static std::string json_escape(const std::string & value);
  bool deadline_expired() const;
  void set_deadline(double timeout_sec);
  bool vslam_subscriber_ready() const;
  bool can_forward_pose() const;
  bool map_origin_job_may_be_running() const;
  void transition(State state, std::string reason);
  void begin_localization(const std::string & source);
  void start_attempt();
  void request_vgl();
  void on_vgl_trigger_response(const rclcpp::Client<Trigger>::SharedFuture & future,
                               std::uint64_t generation);
  void schedule_retry(const std::string & reason);
  void wait_for_manual(const std::string & reason);
  void queue_pose(const Pose & pose, const std::string & source);
  void forward_pending_pose();
  void on_vslam_hint_request(const Pose::SharedPtr message);
  void on_manual_pose(const Pose::SharedPtr message);
  void on_vgl_pose(const Pose::SharedPtr message);
  void clear_pending_vgl_request();
  void on_localization_trigger(const std_msgs::msg::Bool::SharedPtr message);
  void on_relocalize_service(const Trigger::Request::SharedPtr request,
                             Trigger::Response::SharedPtr response);
  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message);
  void on_tick();
  void publish_status();

  bool use_vgl_{false};
  bool autostart_{false};
  bool origin_startup_{false};
  bool wait_for_vslam_subscriber_{true};
  bool wait_for_vslam_diagnostics_{true};
  double dependency_wait_timeout_sec_{30.0};
  double vgl_response_timeout_sec_{30.0};
  double vslam_confirmation_timeout_sec_{60.0};
  double origin_diagnostics_timeout_sec_{120.0};
  double retry_backoff_sec_{1.0};
  double poll_period_sec_{0.1};
  double status_publish_period_sec_{0.5};
  int max_attempts_{3};
  PoseValidationOptions validation_options_;

  std::string vslam_hint_request_topic_;
  std::string vslam_pose_hint_topic_;
  std::string manual_pose_topic_;
  std::string vgl_trigger_service_;
  std::string vgl_pose_topic_;
  std::string localization_trigger_topic_;
  std::string localization_trigger_service_;
  std::string diagnostics_topic_;
  std::string vslam_diagnostics_hardware_id_;
  std::string vslam_localized_key_;
  std::string pose_hint_required_topic_;
  std::string pose_hint_state_topic_;

  State state_{State::kIdle};
  bool pose_hint_required_{false};
  int attempts_{0};
  std::uint64_t request_generation_{0};
  std::string request_source_{"none"};
  std::string last_hint_source_{"none"};
  std::string reason_{"startup"};
  std::optional<Pose> pending_pose_;
  std::string pending_pose_source_;
  std::optional<bool> last_vslam_localized_;
  bool saw_vslam_diagnostics_{false};
  bool saw_not_localized_since_hint_{false};
  SteadyTime steady_now_;
  SteadyTime last_status_publish_{};
  std::optional<SteadyTime> deadline_;
  std::optional<SteadyTime> origin_diagnostics_deadline_;
  std::optional<std::int64_t> pending_vgl_request_id_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pose_hint_required_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_hint_state_pub_;
  rclcpp::Publisher<Pose>::SharedPtr vslam_pose_hint_pub_;
  rclcpp::Client<Trigger>::SharedPtr vgl_trigger_client_;
  rclcpp::Service<Trigger>::SharedPtr relocalize_service_;
  rclcpp::Subscription<Pose>::SharedPtr vslam_hint_request_sub_;
  rclcpp::Subscription<Pose>::SharedPtr manual_pose_sub_;
  rclcpp::Subscription<Pose>::SharedPtr vgl_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_trigger_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace jetpilot_localization_manager

#endif  // JETPILOT_LOCALIZATION_MANAGER__LOCALIZATION_MANAGER_NODE_HPP_
