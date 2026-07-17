#include "jetpilot_planning/lane_selection_policy.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace jetpilot_planning
{
namespace
{

std::string trim(const std::string & value)
{
  const auto is_not_space = [](unsigned char character) {return !std::isspace(character);};
  const auto first = std::find_if(value.begin(), value.end(), is_not_space);
  if (first == value.end()) {
    return {};
  }
  const auto last = std::find_if(value.rbegin(), value.rend(), is_not_space).base();
  return std::string(first, last);
}

}  // namespace

std::unordered_map<std::string, std::string> parse_section_lane_rules(
  const std::vector<std::string> & entries)
{
  std::unordered_map<std::string, std::string> rules;
  for (const auto & entry : entries) {
    const auto separator = entry.find('=');
    if (separator == std::string::npos || entry.find('=', separator + 1U) != std::string::npos) {
      throw std::invalid_argument(
              "section_lane_rules entries must use exactly one '=': 'section_id=lane_id'");
    }
    const auto section_id = trim(entry.substr(0U, separator));
    const auto lane_id = trim(entry.substr(separator + 1U));
    if (section_id.empty() || lane_id.empty()) {
      throw std::invalid_argument("section_lane_rules section and lane IDs must not be empty");
    }
    if (!rules.emplace(section_id, lane_id).second) {
      throw std::invalid_argument("duplicate section_lane_rules entry for section: " + section_id);
    }
  }
  return rules;
}

std::string selection_source_name(const SelectionSource source)
{
  switch (source) {
    case SelectionSource::kRequestedLane:
      return "requested_lane";
    case SelectionSource::kSectionRule:
      return "section_rule";
    case SelectionSource::kDefaultLane:
      return "default_lane";
    case SelectionSource::kNone:
    default:
      return "none";
  }
}

LaneSelectionPolicy::LaneSelectionPolicy(
  std::string default_lane_id,
  std::unordered_map<std::string, std::string> section_lane_rules,
  const bool fallback_to_default_lane)
: default_lane_id_(std::move(default_lane_id)),
  section_lane_rules_(std::move(section_lane_rules)),
  fallback_to_default_lane_(fallback_to_default_lane)
{
  if (default_lane_id_.empty()) {
    throw std::invalid_argument("default_lane_id must not be empty");
  }
}

SelectionDecision LaneSelectionPolicy::fallback_or_stop(
  const std::string & unavailable_reason,
  const std::unordered_set<std::string> & available_lane_ids) const
{
  if (fallback_to_default_lane_ && available_lane_ids.count(default_lane_id_) != 0U) {
    return {
      default_lane_id_, SelectionSource::kDefaultLane,
      unavailable_reason + "_fallback_to_default"};
  }
  return {{}, SelectionSource::kNone, unavailable_reason};
}

SelectionDecision LaneSelectionPolicy::select(
  const std::string & requested_lane_id,
  const std::string & current_section_id,
  const std::unordered_set<std::string> & available_lane_ids) const
{
  if (!requested_lane_id.empty()) {
    if (available_lane_ids.count(requested_lane_id) != 0U) {
      return {requested_lane_id, SelectionSource::kRequestedLane, "requested_lane_selected"};
    }
    return fallback_or_stop("requested_lane_unavailable", available_lane_ids);
  }

  const auto section_rule = section_lane_rules_.find(current_section_id);
  if (section_rule != section_lane_rules_.end()) {
    if (available_lane_ids.count(section_rule->second) != 0U) {
      return {section_rule->second, SelectionSource::kSectionRule, "section_rule_selected"};
    }
    return fallback_or_stop("section_lane_unavailable", available_lane_ids);
  }

  if (available_lane_ids.count(default_lane_id_) != 0U) {
    return {default_lane_id_, SelectionSource::kDefaultLane, "default_lane_selected"};
  }
  return {{}, SelectionSource::kNone, "default_lane_unavailable"};
}

}  // namespace jetpilot_planning
