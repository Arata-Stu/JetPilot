#include "evs_benchmark/benchmark.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>

int main()
{
  try {
    const auto dataset = evs_benchmark::make_synthetic(640, 480, 0.2, 0.05, 7);
    evs_benchmark::Config config;
    config.width = 64;
    config.height = 48;
    config.bins = 10;
    config.window_us = 40000;
    config.stride_us = 4000;
    const auto full = evs_benchmark::run_cpu_full(dataset, config, 0);
    const auto incremental = evs_benchmark::run_cpu_incremental(dataset, config, 0);
    if (full.snapshots != incremental.snapshots ||
      std::abs(full.checksum - incremental.checksum) > 1.0e-6)
    {
      std::cerr << "CPU backends differ: full=" << full.checksum <<
        " incremental=" << incremental.checksum << '\n';
      return 1;
    }
    const auto path = std::filesystem::temp_directory_path() / "evs_benchmark_test.evbin";
    evs_benchmark::write_evbin(path.string(), dataset);
    const auto segment = evs_benchmark::read_evbin(path.string(), 50000, 40000);
    std::filesystem::remove(path);
    if (segment.events.empty() || segment.events.front().timestamp_us < 50000 ||
      segment.events.back().timestamp_us >= 90000)
    {
      std::cerr << "EVSBIN segment selection is incorrect\n";
      return 1;
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
