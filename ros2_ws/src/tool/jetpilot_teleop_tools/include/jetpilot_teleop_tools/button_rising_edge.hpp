#ifndef JETPILOT_TELEOP_TOOLS__BUTTON_RISING_EDGE_HPP_
#define JETPILOT_TELEOP_TOOLS__BUTTON_RISING_EDGE_HPP_

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string_view>
#include <vector>

namespace jetpilot_teleop_tools
{

struct ButtonAssignment
{
  std::string_view name;
  int index;
};

struct ButtonManagerAssignments
{
  int auto_button{-1};
  int manual_button{-1};
  int stop_button{-1};
  int back_button{-1};
  int bag_start_button{-1};
  int bag_stop_button{-1};
  int steer_offset_inc_button{-1};
  int steer_offset_dec_button{-1};
};

inline std::optional<std::string_view> find_button_conflict(
  const int candidate, const std::initializer_list<ButtonAssignment> assignments)
{
  if (candidate < 0) {
    return std::nullopt;
  }

  for (const auto & assignment : assignments) {
    if (assignment.index >= 0 && assignment.index == candidate) {
      return assignment.name;
    }
  }
  return std::nullopt;
}

inline std::optional<std::string_view> find_localization_button_conflict(
  const int candidate, const ButtonManagerAssignments & assignments)
{
  // back_button is a modifier and does not publish an action by itself, so it may be shared.
  return find_button_conflict(
    candidate,
    {
      {"auto_button", assignments.auto_button},
      {"manual_button", assignments.manual_button},
      {"stop_button", assignments.stop_button},
      {"bag_start_button", assignments.bag_start_button},
      {"bag_stop_button", assignments.bag_stop_button},
      {"steer_offset_inc_button", assignments.steer_offset_inc_button},
      {"steer_offset_dec_button", assignments.steer_offset_dec_button},
    });
}

class ButtonRisingEdge
{
public:
  void configure(const int index, const bool enabled = true)
  {
    index_ = index;
    enabled_ = enabled && index >= 0;
    was_pressed_ = false;
  }

  bool update(const std::vector<int32_t> & buttons)
  {
    const bool pressed = enabled_ && static_cast<std::size_t>(index_) < buttons.size() &&
      buttons[static_cast<std::size_t>(index_)] != 0;
    const bool rising_edge = pressed && !was_pressed_;
    was_pressed_ = pressed;
    return rising_edge;
  }

  bool enabled() const
  {
    return enabled_;
  }

private:
  int index_{-1};
  bool enabled_{false};
  bool was_pressed_{false};
};

}  // namespace jetpilot_teleop_tools

#endif  // JETPILOT_TELEOP_TOOLS__BUTTON_RISING_EDGE_HPP_
