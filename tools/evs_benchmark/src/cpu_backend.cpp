#include "evs_benchmark/benchmark.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <vector>

namespace evs_benchmark
{
namespace
{

using Clock = std::chrono::steady_clock;

std::size_t channel_index(
  const Event & event, const std::int64_t window_start_us,
  const std::int64_t bin_width_us, const std::size_t bins)
{
  const auto bin = static_cast<std::size_t>(
    (event.timestamp_us - window_start_us) / bin_width_us);
  return (event.polarity != 0 ? 0U : bins) + std::min(bin, bins - 1U);
}

void accumulate(
  const Dataset & dataset, const Config & config, const std::int64_t window_start_us,
  const std::size_t begin, const std::size_t end, std::vector<float> & tensor)
{
  const auto pixels = static_cast<std::size_t>(config.width) * config.height;
  const auto bin_width_us = config.window_us / static_cast<std::int64_t>(config.bins);
  for (auto index = begin; index < end; ++index) {
    const auto & event = dataset.events[index];
    if (event.x >= dataset.width || event.y >= dataset.height) {
      continue;
    }
    const auto x = std::min<std::size_t>(
      config.width - 1U,
      static_cast<std::size_t>(event.x) * config.width / dataset.width);
    const auto y = std::min<std::size_t>(
      config.height - 1U,
      static_cast<std::size_t>(event.y) * config.height / dataset.height);
    const auto channel = channel_index(event, window_start_us, bin_width_us, config.bins);
    tensor[channel * pixels + y * config.width + x] += 1.0F;
  }
}

std::size_t lower_bound_index(const Dataset & dataset, const std::int64_t timestamp_us)
{
  return static_cast<std::size_t>(std::lower_bound(
      dataset.events.begin(), dataset.events.end(), timestamp_us,
      [](const Event & event, const std::int64_t timestamp) {
        return event.timestamp_us < timestamp;
      }) - dataset.events.begin());
}

double checksum(const std::vector<float> & tensor)
{
  double sum = 0.0;
  for (std::size_t index = 0; index < tensor.size(); ++index) {
    sum += static_cast<double>(tensor[index]) * static_cast<double>((index % 251U) + 1U);
  }
  return sum;
}

Result finish_result(
  Result result, const Clock::time_point wall_start, const std::clock_t cpu_start)
{
  result.wall_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - wall_start).count();
  result.cpu_ms = 1000.0 * static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
  result.cpu_util_pct = result.wall_ms > 0.0 ? 100.0 * result.cpu_ms / result.wall_ms : 0.0;
  return result;
}

}  // namespace

Result run_cpu_full(const Dataset & dataset, const Config & config, const std::size_t trial)
{
  validate(dataset, config);
  Result result;
  result.backend = "cpu";
  result.algorithm = "full";
  result.trial = trial;
  result.events = dataset.events.size();
  const auto pixels = static_cast<std::size_t>(config.width) * config.height;
  std::vector<float> tensor(2U * config.bins * pixels, 0.0F);
  const auto first_end = dataset.events.front().timestamp_us + config.window_us;
  const auto last_end = dataset.events.back().timestamp_us;
  auto previous_end = first_end - config.stride_us;
  bool initialized = false;
  result.sequence_checksum = 1469598103934665603ULL;
  const auto wall_start = Clock::now();
  const auto cpu_start = std::clock();
  for (auto end_us = first_end; end_us <= last_end; end_us += config.stride_us) {
    Clock::time_point snapshot_wall_start;
    if (config.capture_trace) {snapshot_wall_start = Clock::now();}
    const auto start_us = end_us - config.window_us;
    const auto begin = lower_bound_index(dataset, start_us);
    const auto end = lower_bound_index(dataset, end_us);
    const auto new_begin = config.capture_trace && initialized ?
      lower_bound_index(dataset, previous_end) : begin;
    std::fill(tensor.begin(), tensor.end(), 0.0F);
    accumulate(dataset, config, start_us, begin, end, tensor);
    if (config.capture_sequence_checksums) {
      const auto snapshot_checksum = tensor_checksum64(tensor);
      result.snapshot_checksums.push_back(snapshot_checksum);
      result.sequence_checksum = append_sequence_checksum(
        result.sequence_checksum, snapshot_checksum, result.snapshots);
    }
    if (config.capture_trace) {
      SnapshotTrace trace;
      trace.snapshot_index = result.snapshots;
      trace.end_timestamp_us = end_us;
      trace.new_events = end - new_begin;
      trace.window_events = end - begin;
      trace.wall_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - snapshot_wall_start).count();
      result.trace.push_back(trace);
    }
    previous_end = end_us;
    initialized = true;
    ++result.snapshots;
  }
  result = finish_result(std::move(result), wall_start, cpu_start);
  result.checksum = checksum(tensor);
  return result;
}

Result run_cpu_incremental(
  const Dataset & dataset, const Config & config, const std::size_t trial)
{
  validate(dataset, config);
  Result result;
  result.backend = "cpu";
  result.algorithm = "incremental";
  result.trial = trial;
  result.events = dataset.events.size();
  const auto pixels = static_cast<std::size_t>(config.width) * config.height;
  const auto channels = 2U * config.bins;
  const auto shift_bins = static_cast<std::size_t>(
    config.stride_us / (config.window_us / static_cast<std::int64_t>(config.bins)));
  std::vector<float> tensor(channels * pixels, 0.0F);
  const auto first_end = dataset.events.front().timestamp_us + config.window_us;
  const auto last_end = dataset.events.back().timestamp_us;
  auto previous_end = first_end - config.stride_us;
  bool initialized = false;
  result.sequence_checksum = 1469598103934665603ULL;
  const auto wall_start = Clock::now();
  const auto cpu_start = std::clock();
  for (auto end_us = first_end; end_us <= last_end; end_us += config.stride_us) {
    Clock::time_point snapshot_wall_start;
    if (config.capture_trace) {snapshot_wall_start = Clock::now();}
    const bool first_snapshot = !initialized;
    const auto start_us = end_us - config.window_us;
    const auto window_end = lower_bound_index(dataset, end_us);
    const auto new_begin = first_snapshot ? 0U : lower_bound_index(dataset, previous_end);
    const auto window_begin = first_snapshot || config.capture_trace ?
      lower_bound_index(dataset, start_us) : 0U;
    if (!initialized) {
      accumulate(dataset, config, start_us, window_begin, window_end, tensor);
      initialized = true;
    } else {
      for (std::size_t polarity = 0; polarity < 2U; ++polarity) {
        auto * block = tensor.data() + polarity * config.bins * pixels;
        const auto retained = (config.bins - shift_bins) * pixels;
        std::memmove(
          block, block + shift_bins * pixels, retained * sizeof(float));
        std::fill(
          block + retained, block + config.bins * pixels, 0.0F);
      }
      accumulate(dataset, config, start_us, new_begin, window_end, tensor);
    }
    if (config.capture_sequence_checksums) {
      const auto snapshot_checksum = tensor_checksum64(tensor);
      result.snapshot_checksums.push_back(snapshot_checksum);
      result.sequence_checksum = append_sequence_checksum(
        result.sequence_checksum, snapshot_checksum, result.snapshots);
    }
    if (config.capture_trace) {
      SnapshotTrace trace;
      trace.snapshot_index = result.snapshots;
      trace.end_timestamp_us = end_us;
      trace.new_events = first_snapshot ? window_end - window_begin : window_end - new_begin;
      trace.window_events = window_end - window_begin;
      trace.wall_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - snapshot_wall_start).count();
      result.trace.push_back(trace);
    }
    previous_end = end_us;
    ++result.snapshots;
  }
  result = finish_result(std::move(result), wall_start, cpu_start);
  result.checksum = checksum(tensor);
  return result;
}

}  // namespace evs_benchmark
