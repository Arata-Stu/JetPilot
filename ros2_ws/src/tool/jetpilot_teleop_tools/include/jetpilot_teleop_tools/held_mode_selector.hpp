#ifndef JETPILOT_TELEOP_TOOLS__HELD_MODE_SELECTOR_HPP_
#define JETPILOT_TELEOP_TOOLS__HELD_MODE_SELECTOR_HPP_

#include <algorithm>

namespace jetpilot_teleop_tools
{

enum class HeldMode
{
  STOP,
  MANUAL,
  AUTO,
};

class HeldModeSelector
{
public:
  void set_hold_time(const double hold_time_s)
  {
    hold_time_s_ = std::max(0.0, hold_time_s);
    auto_state_ = {};
    manual_state_ = {};
    stop_state_ = {};
  }

  HeldMode update(const bool auto_pressed, const bool manual_pressed,
                  const bool stop_pressed, const double current_time_s)
  {
    const bool auto_ready = held(auto_state_, auto_pressed, current_time_s);
    const bool manual_ready = held(manual_state_, manual_pressed, current_time_s);
    const bool stop_ready = held(stop_state_, stop_pressed, current_time_s);

    if (stop_ready || (auto_pressed && manual_pressed))
    {
      return HeldMode::STOP;
    }
    if (auto_ready)
    {
      return HeldMode::AUTO;
    }
    if (manual_ready)
    {
      return HeldMode::MANUAL;
    }
    return HeldMode::STOP;
  }

private:
  struct HoldState
  {
    bool pressed{false};
    double since_s{0.0};
  };

  bool held(HoldState & state, const bool pressed, const double current_time_s) const
  {
    if (!pressed)
    {
      state = {};
      return false;
    }
    if (!state.pressed || current_time_s < state.since_s)
    {
      state.pressed = true;
      state.since_s = current_time_s;
    }
    return current_time_s - state.since_s >= hold_time_s_;
  }

  double hold_time_s_{0.1};
  HoldState auto_state_;
  HoldState manual_state_;
  HoldState stop_state_;
};

}  // namespace jetpilot_teleop_tools

#endif  // JETPILOT_TELEOP_TOOLS__HELD_MODE_SELECTOR_HPP_
