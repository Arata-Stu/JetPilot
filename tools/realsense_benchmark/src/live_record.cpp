#include "realsense_benchmark/benchmark.hpp"

#include <librealsense2/rs.hpp>

#include <chrono>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  try {
    std::uint32_t width = 848, height = 480, fps = 90;
    double duration_s = 30, warmup_s = 2;
    std::string output_path = "d455.rgbbin";
    for (int index = 1; index < argc; ++index) {
      const std::string arg(argv[index]);
      auto next = [&]() {if (++index >= argc) {throw std::invalid_argument("missing value");} return std::string(argv[index]);};
      if (arg == "--width") {width = std::stoul(next());}
      else if (arg == "--height") {height = std::stoul(next());}
      else if (arg == "--fps") {fps = std::stoul(next());}
      else if (arg == "--duration-s") {duration_s = std::stod(next());}
      else if (arg == "--warmup-s") {warmup_s = std::stod(next());}
      else if (arg == "--output") {output_path = next();}
      else {throw std::invalid_argument("unknown argument: " + arg);}
    }
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);
    const auto profile = pipeline.start(config);
    const auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    const auto warmup_start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(
        std::chrono::steady_clock::now() - warmup_start).count() < warmup_s)
    {
      (void)pipeline.wait_for_frames(5000);
    }
    realsense_benchmark::RgbbinWriter writer(output_path, stream.width(), stream.height());
    const auto packed_stride = static_cast<std::size_t>(stream.width()) * 3U;
    std::vector<std::uint8_t> packed(packed_stride * stream.height());
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < duration_s) {
      const auto frames = pipeline.wait_for_frames(5000);
      const auto frame = frames.get_color_frame();
      if (!frame) {continue;}
      const auto * source = static_cast<const std::uint8_t *>(frame.get_data());
      const std::uint8_t * rgb = source;
      if (static_cast<std::size_t>(frame.get_stride_in_bytes()) != packed_stride) {
        for (int y = 0; y < frame.get_height(); ++y) {
          std::memcpy(
            packed.data() + static_cast<std::size_t>(y) * packed_stride,
            source + static_cast<std::size_t>(y) * frame.get_stride_in_bytes(), packed_stride);
        }
        rgb = packed.data();
      }
      writer.append(
        static_cast<std::uint64_t>(frame.get_timestamp() * 1000.0), frame.get_frame_number(),
        rgb);
    }
    pipeline.stop();
    writer.close();
    std::cout << "frames=" << writer.frame_count() << " output=" << output_path << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "realsense_record_rgbbin: " << error.what() << '\n';
    return 1;
  }
}
