#include "realsense_benchmark/benchmark.hpp"

#include <filesystem>
#include <iostream>

int main()
{
  try {
    const auto dataset = realsense_benchmark::make_synthetic(64, 48, 20, 9);
    realsense_benchmark::Config config;
    config.width = 32;
    config.height = 24;
    const auto result = realsense_benchmark::run_cpu(dataset, config, 0);
    if (result.frames != 20 || result.checksum == 0) {return 1;}
    const auto path = std::filesystem::temp_directory_path() / "realsense_benchmark.rgbbin";
    realsense_benchmark::write_rgbbin(path.string(), dataset);
    const auto segment = realsense_benchmark::read_rgbbin(path.string(), 50000, 50000);
    std::filesystem::remove(path);
    if (segment.frame_count() == 0 || segment.timestamp_us.front() < 50000 ||
      segment.timestamp_us.back() >= 100000)
    {return 1;}
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
