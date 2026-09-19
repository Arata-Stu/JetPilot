#ifndef JETPILOT_TELEOP_TOOLS__DEADMAN_CONTROL_HPP_
#define JETPILOT_TELEOP_TOOLS__DEADMAN_CONTROL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace jetpilot_teleop_tools
{

inline bool deadman_is_pressed(const std::vector<int32_t> & buttons, const int index)
{
  // A negative binding explicitly disables the deadman for legacy configurations.
  // Fixed-throttle mode rejects that configuration at node startup.
  return index < 0 ||
         (static_cast<std::size_t>(index) < buttons.size() &&
          buttons[static_cast<std::size_t>(index)] != 0);
}

}  // namespace jetpilot_teleop_tools

#endif  // JETPILOT_TELEOP_TOOLS__DEADMAN_CONTROL_HPP_
