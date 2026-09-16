#include "realsense_benchmark/benchmark.hpp"

#include <librealsense2/rs.hpp>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: realsense_bag_to_rgbbin INPUT.bag OUTPUT.rgbbin\n";
    return 2;
  }
  try {
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_device_from_file(argv[1], false);
    const auto profile = pipeline.start(config);
    auto playback = profile.get_device().as<rs2::playback>();
    playback.set_real_time(false);
    auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    realsense_benchmark::RgbbinWriter writer(argv[2], stream.width(), stream.height());
    std::vector<std::uint8_t> converted(
      static_cast<std::size_t>(stream.width()) * stream.height() * 3U);
    auto previous_position = playback.get_position();
    std::uint64_t previous_frame_number = 0;
    bool have_frame = false;
    while (true) {
      rs2::frameset frames;
      if (!pipeline.try_wait_for_frames(&frames, 5000)) {break;}
      const auto position = playback.get_position();
      if (have_frame && position <= previous_position) {break;}
      previous_position = position;
      auto frame = frames.get_color_frame();
      if (!frame) {continue;}
      if (have_frame && frame.get_frame_number() == previous_frame_number) {continue;}
      previous_frame_number = frame.get_frame_number();
      have_frame = true;
      const auto format = frame.get_profile().format();
      const auto * source = static_cast<const std::uint8_t *>(frame.get_data());
      const std::uint8_t * rgb = source;
      if (format == RS2_FORMAT_BGR8) {
        for (int y = 0; y < frame.get_height(); ++y) {
          const auto * row = source + static_cast<std::size_t>(y) * frame.get_stride_in_bytes();
          for (int x = 0; x < frame.get_width(); ++x) {
            const auto pixel = static_cast<std::size_t>(y) * frame.get_width() + x;
            converted[3U * pixel] = row[3U * x + 2U];
            converted[3U * pixel + 1U] = row[3U * x + 1U];
            converted[3U * pixel + 2U] = row[3U * x];
          }
        }
        rgb = converted.data();
      } else if (format != RS2_FORMAT_RGB8) {
        throw std::runtime_error("only RGB8/BGR8 color recordings are supported");
      } else if (static_cast<std::size_t>(frame.get_stride_in_bytes()) !=
        static_cast<std::size_t>(frame.get_width()) * 3U)
      {
        const auto packed_stride = static_cast<std::size_t>(frame.get_width()) * 3U;
        for (int y = 0; y < frame.get_height(); ++y) {
          std::memcpy(
            converted.data() + static_cast<std::size_t>(y) * packed_stride,
            source + static_cast<std::size_t>(y) * frame.get_stride_in_bytes(), packed_stride);
        }
        rgb = converted.data();
      }
      writer.append(
        static_cast<std::uint64_t>(frame.get_timestamp() * 1000.0),
        frame.get_frame_number(), rgb);
    }
    pipeline.stop();
    writer.close();
    std::cout << "frames=" << writer.frame_count() << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "realsense_bag_to_rgbbin: " << error.what() << '\n';
    return 1;
  }
}
