#include "realsense_benchmark/benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct Options
{
  std::string input, output{"results.csv"}, backend{"all"};
  std::int64_t segment_start_us{0}, segment_duration_us{2000000};
  std::uint32_t source_width{848}, source_height{480};
  std::size_t synthetic_frames{180}, warmup{2}, trials{10};
  std::uint64_t seed{42};
  bool check_cuda{false};
  realsense_benchmark::Config config;
};

std::string value(int & index, int argc, char ** argv)
{
  if (++index >= argc) {throw std::invalid_argument("missing argument value");}
  return argv[index];
}

Options parse(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string arg(argv[index]);
    if (arg == "--input") {options.input = value(index, argc, argv);}
    else if (arg == "--output") {options.output = value(index, argc, argv);}
    else if (arg == "--backend") {options.backend = value(index, argc, argv);}
    else if (arg == "--segment-start-us") {options.segment_start_us = std::stoll(value(index, argc, argv));}
    else if (arg == "--segment-duration-us") {options.segment_duration_us = std::stoll(value(index, argc, argv));}
    else if (arg == "--source-width") {options.source_width = std::stoul(value(index, argc, argv));}
    else if (arg == "--source-height") {options.source_height = std::stoul(value(index, argc, argv));}
    else if (arg == "--synthetic-frames") {options.synthetic_frames = std::stoull(value(index, argc, argv));}
    else if (arg == "--width") {options.config.width = std::stoul(value(index, argc, argv));}
    else if (arg == "--height") {options.config.height = std::stoul(value(index, argc, argv));}
    else if (arg == "--warmup") {options.warmup = std::stoull(value(index, argc, argv));}
    else if (arg == "--trials") {options.trials = std::stoull(value(index, argc, argv));}
    else if (arg == "--seed") {options.seed = std::stoull(value(index, argc, argv));}
    else if (arg == "--check-cuda") {options.check_cuda = true;}
    else if (arg == "--help") {
      std::cout << "rgb_bench [--input INPUT.rgbbin] [--backend all|cpu|cuda] "
        "[--width 212 --height 120] [--segment-start-us 0 --segment-duration-us 2000000] "
        "[--warmup 2 --trials 10 --output results.csv]\n";
      std::exit(0);
    } else {throw std::invalid_argument("unknown argument: " + arg);}
  }
  if (!options.trials) {throw std::invalid_argument("trials must be positive");}
  return options;
}

void write_results(const std::string & path, const std::vector<realsense_benchmark::Result> & results)
{
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {std::filesystem::create_directories(parent);}
  std::ofstream output(path, std::ios::trunc);
  if (!output) {throw std::runtime_error("cannot create result CSV");}
  output << "backend,trial,frames,wall_ms,cpu_ms,cpu_util_pct,host_staging_ms,h2d_ms,"
    "preprocess_ms,gpu_total_ms,wall_us_per_frame,h2d_us_per_frame,"
    "preprocess_us_per_frame,checksum\n" << std::setprecision(12);
  for (const auto & result : results) {
    output << result.backend << ',' << result.trial << ',' << result.frames << ',' <<
      result.wall_ms << ',' << result.cpu_ms << ',' << result.cpu_util_pct << ',' <<
      result.host_staging_ms << ',' << result.h2d_ms << ',' << result.preprocess_ms << ',' <<
      result.gpu_total_ms << ',' << 1000.0 * result.wall_ms / result.frames << ',' <<
      1000.0 * result.h2d_ms / result.frames << ',' <<
      1000.0 * result.preprocess_ms / result.frames << ',' << result.checksum << '\n';
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse(argc, argv);
    if (options.check_cuda) {
      if (!realsense_benchmark::cuda_available()) {return 1;}
      std::cout << realsense_benchmark::cuda_device_description() << '\n';
      return 0;
    }
    const auto dataset = options.input.empty() ?
      realsense_benchmark::make_synthetic(
        options.source_width, options.source_height, options.synthetic_frames, options.seed) :
      realsense_benchmark::read_rgbbin(
        options.input, options.segment_start_us, options.segment_duration_us);
    realsense_benchmark::validate(dataset, options.config);
    std::cout << "input=" << dataset.width << 'x' << dataset.height <<
      " frames=" << dataset.frame_count() << " output=" <<
      options.config.width << 'x' << options.config.height << '\n';
    using Runner = realsense_benchmark::Result (*)(
      const realsense_benchmark::Dataset &, const realsense_benchmark::Config &, std::size_t);
    std::vector<Runner> runners;
    if (options.backend == "all" || options.backend == "cpu") {runners.push_back(realsense_benchmark::run_cpu);}
    if (options.backend == "all" || options.backend == "cuda") {
      if (realsense_benchmark::cuda_available()) {runners.push_back(realsense_benchmark::run_cuda);}
      else if (options.backend == "cuda") {throw std::runtime_error("CUDA unavailable");}
    }
    if (runners.empty()) {throw std::invalid_argument("invalid backend");}
    for (std::size_t trial = 0; trial < options.warmup; ++trial) {
      for (auto runner : runners) {(void)runner(dataset, options.config, trial);}
    }
    std::vector<realsense_benchmark::Result> results;
    for (std::size_t trial = 0; trial < options.trials; ++trial) {
      double reference = std::numeric_limits<double>::quiet_NaN();
      for (auto runner : runners) {
        auto result = runner(dataset, options.config, trial);
        if (std::isnan(reference)) {reference = result.checksum;}
        else {
          const auto tolerance = std::max(1.0, std::abs(reference)) * 1.0e-5;
          if (std::abs(reference - result.checksum) > tolerance) {
            throw std::runtime_error("CPU/CUDA output checksum mismatch");
          }
        }
        std::cout << result.backend << " trial=" << trial << " wall=" <<
          result.wall_ms << " ms H2D=" << result.h2d_ms <<
          " ms preprocess=" << result.preprocess_ms << " ms\n";
        results.push_back(std::move(result));
      }
    }
    write_results(options.output, results);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "rgb_bench: " << error.what() << '\n';
    return 1;
  }
}
