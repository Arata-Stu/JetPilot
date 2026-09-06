#pragma once
#include "jetpilot_object_detection/reid_core.hpp"

namespace jetpilot_object_detection::reid
{
// Backend boundary deliberately contains no ROS, CUDA or TensorRT types.
// A future NITROS adapter may submit this tensor to TensorRTNode and await a
// correlated result on an independent callback group with a bounded timeout.
struct InferenceRequest
{
  uint64_t request_id{};
  FrameKey frame;
  std::string track_id;
  int width{}, height{};
  std::vector<float> rgb_nchw;
};
struct InferenceResult
{
  uint64_t request_id{};
  FrameKey frame;
  std::string track_id;
  std::vector<float> embedding;
};
inline void validate_result(const InferenceRequest & request, const InferenceResult & result,
  size_t embedding_size)
{
  if (result.request_id != request.request_id || !(result.frame == request.frame) ||
    result.track_id != request.track_id || result.embedding.size() != embedding_size)
  {throw std::invalid_argument("ReID backend result correlation/shape mismatch");}
}
class EmbeddingBackend
{
public:
  virtual ~EmbeddingBackend() = default;
  // Called only on the dedicated inference worker. Never retains input after returning.
  virtual InferenceResult infer(const InferenceRequest & request) = 0;
};
struct TensorRtConfig
{
  std::string engine_path, input_name{"images"}, output_name{"embeddings"};
  int width{128}, height{128}, embedding_size{256}, device{0};
};
std::unique_ptr<EmbeddingBackend> make_tensor_rt_backend(const TensorRtConfig & config);
}  // namespace jetpilot_object_detection::reid
