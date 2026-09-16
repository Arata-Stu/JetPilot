#include <librealsense2/rs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {return 0;}
  std::sort(values.begin(), values.end());
  const auto position = (values.size() - 1U) * fraction;
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  return values[lower] + (values[upper] - values[lower]) * (position - lower);
}

struct Stats
{
  std::uint64_t frames{0}, bytes{0}, sequence_gaps{0};
  std::uint64_t previous_sequence{0};
  double previous_sensor_ms{0};
  Clock::time_point previous_arrival{};
  bool initialized{false};
  std::vector<double> arrival_interval_ms;
  std::vector<double> sensor_interval_ms;
};
}  // namespace

int main(int argc, char ** argv)
{
  try {
    std::uint32_t width = 848, height = 480, fps = 90;
    double warmup_s = 2, duration_s = 30;
    std::string output_path = "realsense_live.csv";
    for (int index = 1; index < argc; ++index) {
      const std::string arg(argv[index]);
      auto next = [&]() {if (++index >= argc) {throw std::invalid_argument("missing value");} return std::string(argv[index]);};
      if (arg == "--width") {width = std::stoul(next());}
      else if (arg == "--height") {height = std::stoul(next());}
      else if (arg == "--fps") {fps = std::stoul(next());}
      else if (arg == "--warmup-s") {warmup_s = std::stod(next());}
      else if (arg == "--duration-s") {duration_s = std::stod(next());}
      else if (arg == "--output") {output_path = next();}
      else {throw std::invalid_argument("unknown argument: " + arg);}
    }
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);
    std::mutex mutex;
    Stats stats;
    bool collecting = false;
    auto profile = pipeline.start(config, [&](const rs2::frame & incoming) {
      rs2::video_frame frame;
      if (auto frames = incoming.as<rs2::frameset>()) {frame = frames.get_color_frame();}
      else {frame = incoming.as<rs2::video_frame>();}
      if (!frame) {return;}
      const auto arrival = Clock::now();
      std::lock_guard<std::mutex> lock(mutex);
      if (!collecting) {return;}
      const auto sequence = frame.get_frame_number();
      const auto sensor_ms = frame.get_timestamp();
      if (stats.initialized) {
        stats.arrival_interval_ms.push_back(
          std::chrono::duration<double, std::milli>(arrival - stats.previous_arrival).count());
        stats.sensor_interval_ms.push_back(sensor_ms - stats.previous_sensor_ms);
        if (sequence > stats.previous_sequence + 1U) {
          stats.sequence_gaps += sequence - stats.previous_sequence - 1U;
        }
      }
      stats.previous_sequence = sequence;
      stats.previous_sensor_ms = sensor_ms;
      stats.previous_arrival = arrival;
      stats.initialized = true;
      ++stats.frames;
      stats.bytes += static_cast<std::uint64_t>(frame.get_stride_in_bytes()) * frame.get_height();
    });
    const auto device = profile.get_device();
    const auto sensor = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    std::this_thread::sleep_for(std::chrono::duration<double>(warmup_s));
    {
      std::lock_guard<std::mutex> lock(mutex);
      stats = Stats{};
      collecting = true;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(duration_s));
    {
      std::lock_guard<std::mutex> lock(mutex);
      collecting = false;
    }
    pipeline.stop();

    std::ofstream output(output_path, std::ios::trunc);
    if (!output) {throw std::runtime_error("cannot create output CSV");}
    output << "device,serial,width,height,configured_fps,duration_s,frames,actual_fps,"
      "mib_s,sequence_gaps,arrival_p50_ms,arrival_p95_ms,arrival_p99_ms,arrival_max_ms,"
      "sensor_p50_ms,sensor_p95_ms,sensor_p99_ms,sensor_max_ms\n" << std::setprecision(12);
    const auto actual_fps = stats.frames / duration_s;
    output << '"' << device.get_info(RS2_CAMERA_INFO_NAME) << "\",\"" <<
      device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "\"," << sensor.width() << ',' <<
      sensor.height() << ',' << fps << ',' << duration_s << ',' << stats.frames << ',' <<
      actual_fps << ',' << stats.bytes / duration_s / 1048576.0 << ',' << stats.sequence_gaps << ',' <<
      percentile(stats.arrival_interval_ms, .50) << ',' << percentile(stats.arrival_interval_ms, .95) << ',' <<
      percentile(stats.arrival_interval_ms, .99) << ',' << percentile(stats.arrival_interval_ms, 1.0) << ',' <<
      percentile(stats.sensor_interval_ms, .50) << ',' << percentile(stats.sensor_interval_ms, .95) << ',' <<
      percentile(stats.sensor_interval_ms, .99) << ',' << percentile(stats.sensor_interval_ms, 1.0) << '\n';
    std::cout << "frames=" << stats.frames << " actual_fps=" << actual_fps <<
      " sequence_gaps=" << stats.sequence_gaps << " arrival_p99_ms=" <<
      percentile(stats.arrival_interval_ms, .99) << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "realsense_live_bench: " << error.what() << '\n';
    return 1;
  }
}
