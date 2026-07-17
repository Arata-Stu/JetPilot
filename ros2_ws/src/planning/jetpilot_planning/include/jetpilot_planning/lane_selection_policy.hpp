#ifndef JETPILOT_PLANNING__LANE_SELECTION_POLICY_HPP_
#define JETPILOT_PLANNING__LANE_SELECTION_POLICY_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace jetpilot_planning
{

enum class SelectionSource
{
  kNone,
  kRequestedLane,
  kSectionRule,
  kDefaultLane,
};

struct SelectionDecision
{
  std::string lane_id;
  SelectionSource source{SelectionSource::kNone};
  std::string reason{"no_available_path"};

  bool ready() const noexcept
  {
    return !lane_id.empty();
  }
};

/// Parse entries such as "shortcut_section=shortcut_lane".
/// Throws std::invalid_argument when an entry is malformed or duplicated.
std::unordered_map<std::string, std::string> parse_section_lane_rules(
  const std::vector<std::string> & entries);

std::string selection_source_name(SelectionSource source);

class LaneSelectionPolicy
{
public:
  LaneSelectionPolicy(
    std::string default_lane_id,
    std::unordered_map<std::string, std::string> section_lane_rules,
    bool fallback_to_default_lane);

  SelectionDecision select(
    const std::string & requested_lane_id,
    const std::string & current_section_id,
    const std::unordered_set<std::string> & available_lane_ids) const;

private:
  SelectionDecision fallback_or_stop(
    const std::string & unavailable_reason,
    const std::unordered_set<std::string> & available_lane_ids) const;

  std::string default_lane_id_;
  std::unordered_map<std::string, std::string> section_lane_rules_;
  bool fallback_to_default_lane_;
};

}  // namespace jetpilot_planning

#endif  // JETPILOT_PLANNING__LANE_SELECTION_POLICY_HPP_
