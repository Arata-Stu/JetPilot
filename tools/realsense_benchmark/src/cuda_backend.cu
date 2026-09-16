#include "realsense_benchmark/benchmark.hpp"

#include <cuda_runtime.h>

#include <chrono>
#include <cstring>
#include <ctime>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace realsense_benchmark
{
namespace
{
void check(cudaError_t status, const char * operation)
{
  if (status != cudaSuccess) {throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));}
}

__global__ void preprocess_kernel(
  const std::uint8_t * input, float * output, std::uint32_t input_width,
  std::uint32_t input_height, std::uint32_t output_width, std::uint32_t output_height,
  float mean0, float mean1, float mean2, float std0, float std1, float std2)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto pixels = static_cast<std::size_t>(output_width) * output_height;
  if (index >= pixels) {return;}
  const auto x = index % output_width;
  const auto y = index / output_width;
  const auto source_x = x * input_width / output_width;
  const auto source_y = y * input_height / output_height;
  const auto source = 3U * (source_y * input_width + source_x);
  output[index] = (static_cast<float>(input[source]) / 255.0F - mean0) / std0;
  output[pixels + index] = (static_cast<float>(input[source + 1U]) / 255.0F - mean1) / std1;
  output[2U * pixels + index] = (static_cast<float>(input[source + 2U]) / 255.0F - mean2) / std2;
}

float elapsed(cudaEvent_t start, cudaEvent_t end)
{
  float value = 0;
  check(cudaEventElapsedTime(&value, start, end), "cudaEventElapsedTime");
  return value;
}

double checksum(const std::vector<float> & tensor)
{
  double value = 0;
  for (std::size_t index = 0; index < tensor.size(); ++index) {
    value += tensor[index] * static_cast<double>((index % 251U) + 1U);
  }
  return value;
}
}  // namespace

bool cuda_available()
{
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

std::string cuda_device_description()
{
  int device = 0;
  check(cudaGetDevice(&device), "cudaGetDevice");
  cudaDeviceProp properties{};
  check(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties");
  std::ostringstream output;
  output << properties.name << "; sm=" << properties.major << '.' << properties.minor <<
    "; memory_bytes=" << properties.totalGlobalMem;
  return output.str();
}

Result run_cuda(const Dataset & dataset, const Config & config, const std::size_t trial)
{
  validate(dataset, config);
  if (!cuda_available()) {throw std::runtime_error("no CUDA device");}
  Result result;
  result.backend = "cuda";
  result.trial = trial;
  result.frames = dataset.frame_count();
  const auto input_bytes = dataset.frame_bytes();
  const auto output_elements = 3U * static_cast<std::size_t>(config.width) * config.height;
  std::uint8_t * host_input = nullptr;
  std::uint8_t * device_input = nullptr;
  float * device_output = nullptr;
  cudaStream_t stream = nullptr;
  cudaEvent_t copy_start = nullptr, copy_end = nullptr, kernel_start = nullptr, kernel_end = nullptr;
  auto cleanup = [&]() {
      if (copy_start) {cudaEventDestroy(copy_start);} if (copy_end) {cudaEventDestroy(copy_end);}
      if (kernel_start) {cudaEventDestroy(kernel_start);} if (kernel_end) {cudaEventDestroy(kernel_end);}
      if (stream) {cudaStreamDestroy(stream);} if (device_output) {cudaFree(device_output);}
      if (device_input) {cudaFree(device_input);} if (host_input) {cudaFreeHost(host_input);}
    };
  try {
    check(cudaMallocHost(reinterpret_cast<void **>(&host_input), input_bytes), "cudaMallocHost");
    check(cudaMalloc(reinterpret_cast<void **>(&device_input), input_bytes), "cudaMalloc input");
    check(cudaMalloc(reinterpret_cast<void **>(&device_output), output_elements * sizeof(float)), "cudaMalloc output");
    check(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), "cudaStreamCreate");
    check(cudaEventCreate(&copy_start), "cudaEventCreate"); check(cudaEventCreate(&copy_end), "cudaEventCreate");
    check(cudaEventCreate(&kernel_start), "cudaEventCreate"); check(cudaEventCreate(&kernel_end), "cudaEventCreate");
    const auto wall_start = std::chrono::steady_clock::now();
    const auto cpu_start = std::clock();
    for (std::size_t frame = 0; frame < dataset.frame_count(); ++frame) {
      const auto staging_start = std::chrono::steady_clock::now();
      std::memcpy(host_input, dataset.rgb.data() + frame * input_bytes, input_bytes);
      result.host_staging_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - staging_start).count();
      check(cudaEventRecord(copy_start, stream), "copy start");
      check(cudaMemcpyAsync(device_input, host_input, input_bytes, cudaMemcpyHostToDevice, stream), "H2D");
      check(cudaEventRecord(copy_end, stream), "copy end");
      check(cudaEventRecord(kernel_start, stream), "kernel start");
      const auto pixels = static_cast<std::size_t>(config.width) * config.height;
      preprocess_kernel<<<static_cast<unsigned>((pixels + 255U) / 256U), 256, 0, stream>>>(
        device_input, device_output, dataset.width, dataset.height, config.width, config.height,
        config.mean[0], config.mean[1], config.mean[2], config.std[0], config.std[1], config.std[2]);
      check(cudaGetLastError(), "preprocess kernel");
      check(cudaEventRecord(kernel_end, stream), "kernel end");
      check(cudaEventSynchronize(kernel_end), "kernel sync");
      result.h2d_ms += elapsed(copy_start, copy_end);
      result.preprocess_ms += elapsed(kernel_start, kernel_end);
    }
    result.wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wall_start).count();
    result.cpu_ms = 1000.0 * static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
    result.cpu_util_pct = result.wall_ms > 0 ? result.cpu_ms / result.wall_ms * 100.0 : 0.0;
    result.gpu_total_ms = result.h2d_ms + result.preprocess_ms;
    std::vector<float> output(output_elements);
    check(cudaMemcpy(output.data(), device_output, output_elements * sizeof(float), cudaMemcpyDeviceToHost), "correctness D2H");
    result.checksum = checksum(output);
    cleanup();
    return result;
  } catch (...) {cleanup(); throw;}
}
}  // namespace realsense_benchmark
