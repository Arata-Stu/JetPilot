#include "jetpilot_object_detection/yolov8_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace jetpilot_object_detection
{
namespace
{

float intersection_over_union(const Detection & lhs, const Detection & rhs)
{
  const float x_min = std::max(lhs.x_min, rhs.x_min);
  const float y_min = std::max(lhs.y_min, rhs.y_min);
  const float x_max = std::min(lhs.x_max, rhs.x_max);
  const float y_max = std::min(lhs.y_max, rhs.y_max);
  const float intersection =
    std::max(0.0F, x_max - x_min) * std::max(0.0F, y_max - y_min);
  const float lhs_area =
    std::max(0.0F, lhs.x_max - lhs.x_min) * std::max(0.0F, lhs.y_max - lhs.y_min);
  const float rhs_area =
    std::max(0.0F, rhs.x_max - rhs.x_min) * std::max(0.0F, rhs.y_max - rhs.y_min);
  const float union_area = lhs_area + rhs_area - intersection;
  return union_area > 0.0F ? intersection / union_area : 0.0F;
}

void restore_source_coordinates(Detection & detection, const DecoderConfig & config)
{
  if (config.resize_mode == ResizeMode::kStretch) {
    const float scale_x =
      static_cast<float>(config.source_width) / static_cast<float>(config.network_width);
    const float scale_y =
      static_cast<float>(config.source_height) / static_cast<float>(config.network_height);
    detection.x_min *= scale_x;
    detection.x_max *= scale_x;
    detection.y_min *= scale_y;
    detection.y_max *= scale_y;
  } else {
    const float scale = std::min(
      static_cast<float>(config.network_width) / static_cast<float>(config.source_width),
      static_cast<float>(config.network_height) / static_cast<float>(config.source_height));
    const float pad_x =
      (static_cast<float>(config.network_width) - config.source_width * scale) * 0.5F;
    const float pad_y =
      (static_cast<float>(config.network_height) - config.source_height * scale) * 0.5F;
    detection.x_min = (detection.x_min - pad_x) / scale;
    detection.x_max = (detection.x_max - pad_x) / scale;
    detection.y_min = (detection.y_min - pad_y) / scale;
    detection.y_max = (detection.y_max - pad_y) / scale;
  }

  detection.x_min = std::clamp(detection.x_min, 0.0F, static_cast<float>(config.source_width));
  detection.x_max = std::clamp(detection.x_max, 0.0F, static_cast<float>(config.source_width));
  detection.y_min = std::clamp(detection.y_min, 0.0F, static_cast<float>(config.source_height));
  detection.y_max = std::clamp(detection.y_max, 0.0F, static_cast<float>(config.source_height));
}

}  // namespace

std::size_t infer_candidate_count(const std::size_t element_count, const std::size_t num_classes)
{
  if (num_classes == 0U) {
    throw std::invalid_argument("num_classes must be positive");
  }
  const std::size_t channels = 4U + num_classes;
  if (element_count == 0U || element_count % channels != 0U) {
    throw std::invalid_argument(
            "YOLO tensor element count must be divisible by 4 + num_classes");
  }
  return element_count / channels;
}

TensorLayout parse_tensor_layout(const std::string & value)
{
  if (value == "channel_major") {
    return TensorLayout::kChannelMajor;
  }
  if (value == "candidate_major") {
    return TensorLayout::kCandidateMajor;
  }
  throw std::invalid_argument("tensor_layout must be channel_major or candidate_major");
}

ResizeMode parse_resize_mode(const std::string & value)
{
  if (value == "letterbox") {
    return ResizeMode::kLetterbox;
  }
  if (value == "stretch") {
    return ResizeMode::kStretch;
  }
  throw std::invalid_argument("resize_mode must be letterbox or stretch");
}

std::vector<Detection> decode_yolov8(
  const float * values,
  const std::size_t element_count,
  const DecoderConfig & config)
{
  if (values == nullptr) {
    throw std::invalid_argument("values must not be null");
  }
  if (config.network_width <= 0 || config.network_height <= 0 ||
    config.source_width <= 0 || config.source_height <= 0)
  {
    throw std::invalid_argument("network and source dimensions must be positive");
  }
  if (config.confidence_threshold < 0.0F || config.confidence_threshold > 1.0F ||
    config.nms_threshold < 0.0F || config.nms_threshold > 1.0F)
  {
    throw std::invalid_argument("confidence and NMS thresholds must be in [0, 1]");
  }
  if (config.max_detections == 0U) {
    throw std::invalid_argument("max_detections must be positive");
  }

  const std::size_t candidates = infer_candidate_count(element_count, config.num_classes);
  const std::size_t channels = 4U + config.num_classes;
  const auto value_at = [&](const std::size_t candidate, const std::size_t channel) {
      return config.tensor_layout == TensorLayout::kChannelMajor ?
             values[channel * candidates + candidate] :
             values[candidate * channels + channel];
    };

  std::vector<Detection> filtered;
  filtered.reserve(std::min(candidates, config.max_detections * 4U));
  for (std::size_t candidate = 0U; candidate < candidates; ++candidate) {
    int best_class = 0;
    float best_score = -std::numeric_limits<float>::infinity();
    for (std::size_t class_id = 0U; class_id < config.num_classes; ++class_id) {
      const float score = value_at(candidate, 4U + class_id);
      if (score > best_score) {
        best_score = score;
        best_class = static_cast<int>(class_id);
      }
    }
    if (!std::isfinite(best_score) || best_score < config.confidence_threshold) {
      continue;
    }

    const float center_x = value_at(candidate, 0U);
    const float center_y = value_at(candidate, 1U);
    const float width = value_at(candidate, 2U);
    const float height = value_at(candidate, 3U);
    if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
      !std::isfinite(width) || !std::isfinite(height) || width <= 0.0F || height <= 0.0F)
    {
      continue;
    }

    Detection detection{
      best_class, best_score,
      center_x - width * 0.5F, center_y - height * 0.5F,
      center_x + width * 0.5F, center_y + height * 0.5F};
    restore_source_coordinates(detection, config);
    if (detection.x_max > detection.x_min && detection.y_max > detection.y_min) {
      filtered.push_back(detection);
    }
  }

  std::sort(filtered.begin(), filtered.end(), [](const Detection & lhs, const Detection & rhs) {
      return lhs.score > rhs.score;
    });
  std::vector<Detection> kept;
  kept.reserve(std::min(filtered.size(), config.max_detections));
  for (const auto & detection : filtered) {
    bool suppressed = false;
    for (const auto & accepted : kept) {
      if (accepted.class_id == detection.class_id &&
        intersection_over_union(accepted, detection) > config.nms_threshold)
      {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      kept.push_back(detection);
      if (kept.size() >= config.max_detections) {
        break;
      }
    }
  }
  return kept;
}

}  // namespace jetpilot_object_detection
