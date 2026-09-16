#include "realsense_benchmark/benchmark.hpp"

#include <chrono>
#include <ctime>

namespace realsense_benchmark
{
namespace
{
double checksum(const std::vector<float> & tensor)
{
  double value = 0;
  for (std::size_t index = 0; index < tensor.size(); ++index) {
    value += tensor[index] * static_cast<double>((index % 251U) + 1U);
  }
  return value;
}
}  // namespace

Result run_cpu(const Dataset & dataset, const Config & config, const std::size_t trial)
{
  validate(dataset, config);
  Result result;
  result.backend = "cpu";
  result.trial = trial;
  result.frames = dataset.frame_count();
  const auto output_pixels = static_cast<std::size_t>(config.width) * config.height;
  std::vector<float> output(3U * output_pixels);
  const auto wall_start = std::chrono::steady_clock::now();
  const auto cpu_start = std::clock();
  for (std::size_t frame = 0; frame < dataset.frame_count(); ++frame) {
    const auto * input = dataset.rgb.data() + frame * dataset.frame_bytes();
    for (std::uint32_t y = 0; y < config.height; ++y) {
      const auto source_y = static_cast<std::size_t>(y) * dataset.height / config.height;
      for (std::uint32_t x = 0; x < config.width; ++x) {
        const auto source_x = static_cast<std::size_t>(x) * dataset.width / config.width;
        const auto source = 3U * (source_y * dataset.width + source_x);
        const auto target = static_cast<std::size_t>(y) * config.width + x;
        for (std::size_t channel = 0; channel < 3U; ++channel) {
          const auto scaled = static_cast<float>(input[source + channel]) / 255.0F;
          output[channel * output_pixels + target] =
            (scaled - config.mean[channel]) / config.std[channel];
        }
      }
    }
  }
  result.wall_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - wall_start).count();
  result.preprocess_ms = result.wall_ms;
  result.cpu_ms = 1000.0 * static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
  result.cpu_util_pct = result.wall_ms > 0 ? result.cpu_ms / result.wall_ms * 100.0 : 0.0;
  result.checksum = checksum(output);
  return result;
}
}  // namespace realsense_benchmark
