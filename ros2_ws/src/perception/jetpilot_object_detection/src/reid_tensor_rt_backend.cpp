#include "jetpilot_object_detection/reid_backend.hpp"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <iterator>

#if NV_TENSORRT_MAJOR < 8 || (NV_TENSORRT_MAJOR == 8 && NV_TENSORRT_MINOR < 5)
#error "ReID requires TensorRT 8.5 or newer"
#endif

namespace jetpilot_object_detection::reid
{
namespace
{
void check(cudaError_t result)
{
  if (result != cudaSuccess) throw std::runtime_error(cudaGetErrorString(result));
}
class Logger : public nvinfer1::ILogger
{
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) std::cerr << "ReID TensorRT: " << message << '\n';
  }
};
struct DeviceBuffer
{
  void * pointer{};
  ~DeviceBuffer() {if (pointer) cudaFree(pointer);}
  void allocate(size_t bytes) {check(cudaMalloc(&pointer, bytes));}
};
struct Stream
{
  cudaStream_t stream{};
  ~Stream() {if (stream) {cudaStreamSynchronize(stream); cudaStreamDestroy(stream);}}
};
class TensorRtBackend final : public EmbeddingBackend
{
public:
  explicit TensorRtBackend(TensorRtConfig config) : config_(std::move(config))
  {
    check(cudaSetDevice(config_.device));
    std::ifstream file(config_.engine_path, std::ios::binary | std::ios::ate);
    if (!file || file.tellg() <= 0 || file.tellg() > 1024LL * 1024 * 1024) {
      throw std::runtime_error("missing, empty or oversized ReID engine: " + config_.engine_path);
    }
    const auto size = static_cast<size_t>(file.tellg());
    file.seekg(0);
    std::vector<char> bytes(size);
    if (!file.read(bytes.data(), static_cast<std::streamsize>(size))) {
      throw std::runtime_error("cannot read ReID engine");
    }
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) throw std::runtime_error("cannot create TensorRT runtime");
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) throw std::runtime_error("cannot deserialize ReID engine on this device");
    if (engine_->getNbIOTensors() != 2) {
      throw std::runtime_error("ReID engine must have exactly one input and one output");
    }
    for (const auto & name : {config_.input_name, config_.output_name}) {
      if (engine_->getTensorDataType(name.c_str()) != nvinfer1::DataType::kFLOAT ||
        engine_->getTensorLocation(name.c_str()) != nvinfer1::TensorLocation::kDEVICE ||
        engine_->getTensorFormat(name.c_str()) != nvinfer1::TensorFormat::kLINEAR)
      {throw std::runtime_error("ReID I/O must be device, linear float32: " + name);}
    }
    if (engine_->getTensorIOMode(config_.input_name.c_str()) != nvinfer1::TensorIOMode::kINPUT ||
      engine_->getTensorIOMode(config_.output_name.c_str()) != nvinfer1::TensorIOMode::kOUTPUT)
    {throw std::runtime_error("incorrect ReID input/output binding names");}
    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("cannot create ReID execution context");
    const nvinfer1::Dims4 expected{1, 3, config_.height, config_.width};
    const auto declared = engine_->getTensorShape(config_.input_name.c_str());
    if (declared.nbDims != 4) throw std::runtime_error("ReID input must be NCHW");
    bool dynamic = false;
    for (int i = 0; i < 4; ++i) {
      dynamic |= declared.d[i] == -1;
      if (declared.d[i] != -1 && declared.d[i] != expected.d[i]) {
        throw std::runtime_error("ReID engine input shape does not match parameters");
      }
    }
    if (dynamic && !context_->setInputShape(config_.input_name.c_str(), expected)) {
      throw std::runtime_error("ReID profile does not support configured input shape");
    }
    const auto output_shape = context_->getTensorShape(config_.output_name.c_str());
    // Accept [1,D] or [1,D,1,1], never arbitrary feature maps/logits of another size.
    if (output_shape.nbDims < 2 || output_shape.d[0] != 1 ||
      output_shape.d[1] != config_.embedding_size)
    {throw std::runtime_error("ReID output must be [1, embedding_size] with optional trailing ones");}
    for (int i = 2; i < output_shape.nbDims; ++i) {
      if (output_shape.d[i] != 1) throw std::runtime_error("unsupported ReID output shape");
    }
    input_bytes_ = static_cast<size_t>(config_.width) * config_.height * 3 * sizeof(float);
    output_bytes_ = static_cast<size_t>(config_.embedding_size) * sizeof(float);
    input_.allocate(input_bytes_);
    output_.allocate(output_bytes_);
    check(cudaStreamCreateWithFlags(&stream_.stream, cudaStreamNonBlocking));
    if (!context_->setTensorAddress(config_.input_name.c_str(), input_.pointer) ||
      !context_->setTensorAddress(config_.output_name.c_str(), output_.pointer))
    {throw std::runtime_error("cannot bind ReID device buffers");}
  }
  InferenceResult infer(const InferenceRequest & request) override
  {
    if (request.width != config_.width || request.height != config_.height ||
      request.rgb_nchw.size() * sizeof(float) != input_bytes_)
    {throw std::invalid_argument("ReID request shape mismatch");}
    InferenceResult result{request.request_id, request.frame, request.track_id,
      std::vector<float>(config_.embedding_size)};
    // All buffers remain alive through stream completion, including on errors.
    try {
      check(cudaMemcpyAsync(input_.pointer, request.rgb_nchw.data(), input_bytes_,
        cudaMemcpyHostToDevice, stream_.stream));
      if (!context_->enqueueV3(stream_.stream)) throw std::runtime_error("ReID enqueueV3 failed");
      check(cudaMemcpyAsync(result.embedding.data(), output_.pointer, output_bytes_,
        cudaMemcpyDeviceToHost, stream_.stream));
      check(cudaStreamSynchronize(stream_.stream));
    } catch (...) {
      cudaStreamSynchronize(stream_.stream);
      throw;
    }
    return result;
  }
private:
  TensorRtConfig config_;
  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  DeviceBuffer input_, output_;
  Stream stream_;  // Destroy/synchronize stream before freeing buffers and engine.
  size_t input_bytes_{}, output_bytes_{};
};
}  // namespace
std::unique_ptr<EmbeddingBackend> make_tensor_rt_backend(const TensorRtConfig & config)
{return std::make_unique<TensorRtBackend>(config);}
}  // namespace jetpilot_object_detection::reid
