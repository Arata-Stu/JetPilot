#include "evs_benchmark/benchmark.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace evs_benchmark
{
namespace
{

using Clock = std::chrono::steady_clock;

void check_cuda(const cudaError_t status, const char * operation)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(
            std::string(operation) + ": " + cudaGetErrorString(status));
  }
}

__global__ void clear_bins_kernel(
  float * ring, const std::size_t pixels, const std::size_t ring_bins,
  const std::int64_t first_bin, const std::size_t count)
{
  const auto linear = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto total = 2U * count * pixels;
  if (linear >= total) {
    return;
  }
  const auto polarity = linear / (count * pixels);
  const auto within = linear % (count * pixels);
  const auto offset = within / pixels;
  const auto pixel = within % pixels;
  const auto global_bin = first_bin + static_cast<std::int64_t>(offset);
  const auto slot = static_cast<std::size_t>(global_bin % static_cast<std::int64_t>(ring_bins));
  ring[(polarity * ring_bins + slot) * pixels + pixel] = 0.0F;
}

__global__ void accumulate_kernel(
  const Event * events, const std::size_t count, float * ring,
  const std::uint32_t source_width, const std::uint32_t source_height,
  const std::uint32_t output_width, const std::uint32_t output_height,
  const std::size_t ring_bins, const std::int64_t origin_us,
  const std::int64_t bin_width_us)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const auto event = events[index];
  if (event.x >= source_width || event.y >= source_height || event.timestamp_us < origin_us) {
    return;
  }
  const auto global_bin = (event.timestamp_us - origin_us) / bin_width_us;
  const auto slot = static_cast<std::size_t>(global_bin % static_cast<std::int64_t>(ring_bins));
  const auto polarity = event.polarity != 0 ? 0U : 1U;
  auto x = static_cast<std::size_t>(event.x) * output_width / source_width;
  auto y = static_cast<std::size_t>(event.y) * output_height / source_height;
  if (x >= output_width) {x = output_width - 1U;}
  if (y >= output_height) {y = output_height - 1U;}
  const auto pixels = static_cast<std::size_t>(output_width) * output_height;
  atomicAdd(&ring[(polarity * ring_bins + slot) * pixels + y * output_width + x], 1.0F);
}

__global__ void snapshot_kernel(
  const float * ring, float * output, const std::size_t pixels,
  const std::size_t bins, const std::size_t ring_bins, const std::int64_t end_bin)
{
  const auto linear = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto total = 2U * bins * pixels;
  if (linear >= total) {
    return;
  }
  const auto polarity = linear / (bins * pixels);
  const auto within = linear % (bins * pixels);
  const auto output_bin = within / pixels;
  const auto pixel = within % pixels;
  const auto global_bin = end_bin - static_cast<std::int64_t>(bins) +
    static_cast<std::int64_t>(output_bin);
  const auto slot = static_cast<std::size_t>(global_bin % static_cast<std::int64_t>(ring_bins));
  output[linear] = ring[(polarity * ring_bins + slot) * pixels + pixel];
}

float elapsed(cudaEvent_t start, cudaEvent_t end)
{
  float milliseconds = 0.0F;
  check_cuda(cudaEventElapsedTime(&milliseconds, start, end), "cudaEventElapsedTime");
  return milliseconds;
}

double checksum(const std::vector<float> & tensor)
{
  double sum = 0.0;
  for (std::size_t index = 0; index < tensor.size(); ++index) {
    sum += static_cast<double>(tensor[index]) * static_cast<double>((index % 251U) + 1U);
  }
  return sum;
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
  check_cuda(cudaGetDevice(&device), "cudaGetDevice");
  cudaDeviceProp properties{};
  check_cuda(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties");
  std::ostringstream output;
  output << properties.name << "; sm=" << properties.major << '.' << properties.minor <<
    "; memory_bytes=" << properties.totalGlobalMem;
  return output.str();
}

Result run_cuda_rolling(
  const Dataset & dataset, const Config & config, const std::size_t trial)
{
  validate(dataset, config);
  if (!cuda_available()) {
    throw std::runtime_error("no CUDA device is available");
  }

  Result result;
  result.backend = "cuda";
  result.algorithm = "rolling";
  result.trial = trial;
  result.events = dataset.events.size();
  result.sequence_checksum = 1469598103934665603ULL;

  const auto pixels = static_cast<std::size_t>(config.width) * config.height;
  const auto output_elements = 2U * config.bins * pixels;
  const auto ring_bins = 2U * config.bins;
  const auto ring_elements = 2U * ring_bins * pixels;
  const auto bin_width_us = config.window_us / static_cast<std::int64_t>(config.bins);
  const auto shift_bins = static_cast<std::size_t>(config.stride_us / bin_width_us);
  const auto first_end = dataset.events.front().timestamp_us + config.window_us;
  const auto last_end = dataset.events.back().timestamp_us;
  const auto origin_us = first_end - config.window_us;

  std::size_t maximum_batch = 1;
  std::size_t scan_begin = 0;
  for (auto end_us = first_end; end_us <= last_end; end_us += config.stride_us) {
    const auto end_it = std::lower_bound(
      dataset.events.begin() + static_cast<std::ptrdiff_t>(scan_begin),
      dataset.events.end(), end_us,
      [](const Event & event, const std::int64_t timestamp) {
        return event.timestamp_us < timestamp;
      });
    const auto scan_end = static_cast<std::size_t>(end_it - dataset.events.begin());
    maximum_batch = std::max(maximum_batch, scan_end - scan_begin);
    scan_begin = scan_end;
  }

  Event * host_events = nullptr;
  Event * device_events = nullptr;
  float * device_ring = nullptr;
  float * device_output = nullptr;
  cudaStream_t stream = nullptr;
  cudaEvent_t h2d_start = nullptr, h2d_end = nullptr;
  cudaEvent_t update_start = nullptr, update_end = nullptr;
  cudaEvent_t snapshot_start = nullptr, snapshot_end = nullptr;

  auto cleanup = [&]() {
      if (h2d_start) {cudaEventDestroy(h2d_start);}
      if (h2d_end) {cudaEventDestroy(h2d_end);}
      if (update_start) {cudaEventDestroy(update_start);}
      if (update_end) {cudaEventDestroy(update_end);}
      if (snapshot_start) {cudaEventDestroy(snapshot_start);}
      if (snapshot_end) {cudaEventDestroy(snapshot_end);}
      if (stream) {cudaStreamDestroy(stream);}
      if (device_output) {cudaFree(device_output);}
      if (device_ring) {cudaFree(device_ring);}
      if (device_events) {cudaFree(device_events);}
      if (host_events) {cudaFreeHost(host_events);}
    };

  try {
    check_cuda(cudaMallocHost(
        reinterpret_cast<void **>(&host_events), maximum_batch * sizeof(Event)), "cudaMallocHost");
    check_cuda(cudaMalloc(
        reinterpret_cast<void **>(&device_events), maximum_batch * sizeof(Event)), "cudaMalloc events");
    check_cuda(cudaMalloc(
        reinterpret_cast<void **>(&device_ring), ring_elements * sizeof(float)), "cudaMalloc ring");
    check_cuda(cudaMalloc(
        reinterpret_cast<void **>(&device_output), output_elements * sizeof(float)), "cudaMalloc output");
    check_cuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking), "cudaStreamCreate");
    check_cuda(cudaEventCreate(&h2d_start), "cudaEventCreate");
    check_cuda(cudaEventCreate(&h2d_end), "cudaEventCreate");
    check_cuda(cudaEventCreate(&update_start), "cudaEventCreate");
    check_cuda(cudaEventCreate(&update_end), "cudaEventCreate");
    check_cuda(cudaEventCreate(&snapshot_start), "cudaEventCreate");
    check_cuda(cudaEventCreate(&snapshot_end), "cudaEventCreate");
    check_cuda(cudaMemsetAsync(device_ring, 0, ring_elements * sizeof(float), stream), "cudaMemset ring");
    check_cuda(cudaStreamSynchronize(stream), "initial synchronize");

    const auto wall_start = Clock::now();
    const auto cpu_start = std::clock();
    std::vector<float> correctness_output;
    if (config.capture_sequence_checksums) {
      correctness_output.resize(output_elements);
    }
    std::size_t begin = 0;
    bool initialized = false;
    for (auto end_us = first_end; end_us <= last_end; end_us += config.stride_us) {
      const auto end_it = std::lower_bound(
        dataset.events.begin() + static_cast<std::ptrdiff_t>(begin),
        dataset.events.end(), end_us,
        [](const Event & event, const std::int64_t timestamp) {
          return event.timestamp_us < timestamp;
        });
      const auto end = static_cast<std::size_t>(end_it - dataset.events.begin());
      const auto count = end - begin;

      const auto staging_start = Clock::now();
      if (count > 0) {
        std::memcpy(host_events, dataset.events.data() + begin, count * sizeof(Event));
      }
      const auto host_staging_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - staging_start).count();
      result.host_staging_ms += host_staging_ms;

      check_cuda(cudaEventRecord(h2d_start, stream), "record H2D start");
      if (count > 0) {
        check_cuda(cudaMemcpyAsync(
            device_events, host_events, count * sizeof(Event), cudaMemcpyHostToDevice, stream),
          "cudaMemcpyAsync H2D");
      }
      check_cuda(cudaEventRecord(h2d_end, stream), "record H2D end");

      check_cuda(cudaEventRecord(update_start, stream), "record update start");
      const auto end_bin = (end_us - origin_us) / bin_width_us;
      const auto bins_to_clear = initialized ? shift_bins : ring_bins;
      if (initialized) {
        const auto clear_total = 2U * bins_to_clear * pixels;
        clear_bins_kernel<<<static_cast<unsigned>((clear_total + 255U) / 256U), 256, 0, stream>>>(
          device_ring, pixels, ring_bins, end_bin - static_cast<std::int64_t>(shift_bins),
          bins_to_clear);
      }
      if (count > 0) {
        accumulate_kernel<<<static_cast<unsigned>((count + 255U) / 256U), 256, 0, stream>>>(
          device_events, count, device_ring, dataset.width, dataset.height,
          config.width, config.height, ring_bins, origin_us, bin_width_us);
      }
      check_cuda(cudaGetLastError(), "rolling update kernel launch");
      check_cuda(cudaEventRecord(update_end, stream), "record update end");

      check_cuda(cudaEventRecord(snapshot_start, stream), "record snapshot start");
      snapshot_kernel<<<static_cast<unsigned>((output_elements + 255U) / 256U), 256, 0, stream>>>(
        device_ring, device_output, pixels, config.bins, ring_bins, end_bin);
      check_cuda(cudaGetLastError(), "snapshot kernel launch");
      check_cuda(cudaEventRecord(snapshot_end, stream), "record snapshot end");
      check_cuda(cudaEventSynchronize(snapshot_end), "snapshot synchronize");

      const auto h2d_ms = static_cast<double>(elapsed(h2d_start, h2d_end));
      const auto update_ms = static_cast<double>(elapsed(update_start, update_end));
      const auto snapshot_ms = static_cast<double>(elapsed(snapshot_start, snapshot_end));
      result.h2d_ms += h2d_ms;
      result.update_ms += update_ms;
      result.snapshot_ms += snapshot_ms;

      if (config.capture_trace) {
        const auto window_begin_it = std::lower_bound(
          dataset.events.begin(), dataset.events.begin() + static_cast<std::ptrdiff_t>(end),
          end_us - config.window_us,
          [](const Event & event, const std::int64_t timestamp) {
            return event.timestamp_us < timestamp;
          });
        SnapshotTrace trace;
        trace.snapshot_index = result.snapshots;
        trace.end_timestamp_us = end_us;
        trace.new_events = count;
        trace.window_events = end - static_cast<std::size_t>(
          window_begin_it - dataset.events.begin());
        trace.wall_ms = std::chrono::duration<double, std::milli>(
          Clock::now() - staging_start).count();
        trace.host_staging_ms = host_staging_ms;
        trace.h2d_ms = h2d_ms;
        trace.update_ms = update_ms;
        trace.snapshot_ms = snapshot_ms;
        result.trace.push_back(trace);
      }

      if (config.capture_sequence_checksums) {
        check_cuda(cudaMemcpy(
            correctness_output.data(), device_output, output_elements * sizeof(float),
            cudaMemcpyDeviceToHost), "sequence correctness D2H");
        const auto snapshot_checksum = tensor_checksum64(correctness_output);
        result.snapshot_checksums.push_back(snapshot_checksum);
        result.sequence_checksum = append_sequence_checksum(
          result.sequence_checksum, snapshot_checksum, result.snapshots);
      }
      begin = end;
      initialized = true;
      ++result.snapshots;
    }
    result.wall_ms = std::chrono::duration<double, std::milli>(Clock::now() - wall_start).count();
    result.cpu_ms = 1000.0 * static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
    result.cpu_util_pct = result.wall_ms > 0.0 ? 100.0 * result.cpu_ms / result.wall_ms : 0.0;
    result.gpu_total_ms = result.h2d_ms + result.update_ms + result.snapshot_ms;

    std::vector<float> output(output_elements);
    check_cuda(cudaMemcpy(
        output.data(), device_output, output_elements * sizeof(float), cudaMemcpyDeviceToHost),
      "correctness D2H");
    result.checksum = checksum(output);
    cleanup();
    return result;
  } catch (...) {
    cleanup();
    throw;
  }
}

}  // namespace evs_benchmark
