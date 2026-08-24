#ifndef JETPILOT_OBJECT_DETECTION__YOLOV8_DECODER_HPP_
#define JETPILOT_OBJECT_DETECTION__YOLOV8_DECODER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace jetpilot_object_detection
{

enum class TensorLayout
{
  kChannelMajor,
  kCandidateMajor,
};

enum class ResizeMode
{
  kLetterbox,
  kStretch,
};

struct DecoderConfig
{
  std::size_t num_classes{2U};
  int network_width{224};
  int network_height{224};
  int source_width{424};
  int source_height{240};
  float confidence_threshold{0.35F};
  float nms_threshold{0.45F};
  std::size_t max_detections{50U};
  TensorLayout tensor_layout{TensorLayout::kChannelMajor};
  ResizeMode resize_mode{ResizeMode::kLetterbox};
};

struct Detection
{
  int class_id{0};
  float score{0.0F};
  float x_min{0.0F};
  float y_min{0.0F};
  float x_max{0.0F};
  float y_max{0.0F};
};

std::size_t infer_candidate_count(std::size_t element_count, std::size_t num_classes);
TensorLayout parse_tensor_layout(const std::string & value);
ResizeMode parse_resize_mode(const std::string & value);

std::vector<Detection> decode_yolov8(
  const float * values,
  std::size_t element_count,
  const DecoderConfig & config);

}  // namespace jetpilot_object_detection

#endif  // JETPILOT_OBJECT_DETECTION__YOLOV8_DECODER_HPP_
