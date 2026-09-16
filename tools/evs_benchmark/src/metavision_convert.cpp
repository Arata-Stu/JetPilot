#include "evs_benchmark/benchmark.hpp"

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 4) {
    std::cerr << "Usage: evs_raw_to_evbin INPUT.raw OUTPUT.evbin [callbacks.csv]\n";
    return 2;
  }
  try {
    const std::string input_path(argv[1]);
    const std::string output_path(argv[2]);
    const std::string callback_path = argc == 4 ? argv[3] : "";
    Metavision::FileConfigHints file_hints;
    file_hints.real_time_playback(false);
    file_hints.time_shift(false);
    auto camera = Metavision::Camera::from_file(input_path, file_hints);
    const auto & geometry = camera.geometry();
    evs_benchmark::EvbinWriter writer(
      output_path, static_cast<std::uint32_t>(geometry.get_width()),
      static_cast<std::uint32_t>(geometry.get_height()));
    std::ofstream callback_output;
    if (!callback_path.empty()) {
      callback_output.open(callback_path, std::ios::trunc);
      if (!callback_output) {
        throw std::runtime_error("cannot create callback CSV: " + callback_path);
      }
      callback_output << "callback_index,event_count,first_timestamp_us,last_timestamp_us,"
        "sensor_span_us,wall_since_previous_us\n";
    }

    std::uint64_t callback_index = 0;
    auto previous_callback = std::chrono::steady_clock::now();
    const auto start = previous_callback;
    camera.cd().add_callback(
      [&](const Metavision::EventCD * begin, const Metavision::EventCD * end) {
        const auto now = std::chrono::steady_clock::now();
        const auto count = static_cast<std::size_t>(end - begin);
        std::vector<evs_benchmark::Event> converted(count);
        for (std::size_t index = 0; index < count; ++index) {
          converted[index].timestamp_us = begin[index].t;
          converted[index].x = begin[index].x;
          converted[index].y = begin[index].y;
          converted[index].polarity = begin[index].p ? 1U : 0U;
        }
        writer.append(converted.data(), converted.size());
        if (callback_output && count > 0) {
          callback_output << callback_index << ',' << count << ',' << begin->t << ',' <<
            (end - 1)->t << ',' << ((end - 1)->t - begin->t) << ',' <<
            std::chrono::duration<double, std::micro>(now - previous_callback).count() << '\n';
        }
        previous_callback = now;
        ++callback_index;
      });

    camera.start();
    while (camera.is_running()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    camera.stop();
    writer.close();
    const auto wall_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
    std::cout << "converted_events=" << writer.event_count() <<
      " callbacks=" << callback_index << " wall_s=" << std::fixed << std::setprecision(3) <<
      wall_s << " throughput_mev_s=" <<
      (wall_s > 0.0 ? static_cast<double>(writer.event_count()) / wall_s / 1.0e6 : 0.0) << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "evs_raw_to_evbin: " << error.what() << '\n';
    return 1;
  }
}
