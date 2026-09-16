#include "evs_benchmark/benchmark.hpp"

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

enum class TimestampPolicy
{
  DropNonmonotonic,
  Preserve,
};

struct Options
{
  std::string input_path;
  std::string output_path;
  std::string callback_path;
  std::string stats_path;
  TimestampPolicy timestamp_policy{TimestampPolicy::DropNonmonotonic};
};

const char * timestamp_policy_name(const TimestampPolicy policy)
{
  return policy == TimestampPolicy::DropNonmonotonic ? "drop_nonmonotonic" : "preserve";
}

std::string json_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += character; break;
    }
  }
  return escaped;
}

Options parse_options(const int argc, char ** argv)
{
  if (argc < 3) {
    throw std::invalid_argument(
            "Usage: evs_raw_to_evbin INPUT.raw OUTPUT.evbin [callbacks.csv] "
            "[--timestamp-policy drop-nonmonotonic|preserve] [--stats OUTPUT.json]");
  }
  Options options;
  options.input_path = argv[1];
  options.output_path = argv[2];
  options.stats_path = options.output_path + ".conversion.json";
  int index = 3;
  if (index < argc && std::string(argv[index]).rfind("--", 0) != 0) {
    options.callback_path = argv[index++];
  }
  for (; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--timestamp-policy" && index + 1 < argc) {
      const std::string policy(argv[++index]);
      if (policy == "drop-nonmonotonic") {
        options.timestamp_policy = TimestampPolicy::DropNonmonotonic;
      } else if (policy == "preserve") {
        options.timestamp_policy = TimestampPolicy::Preserve;
      } else {
        throw std::invalid_argument("unknown timestamp policy: " + policy);
      }
    } else if (argument == "--stats" && index + 1 < argc) {
      options.stats_path = argv[++index];
    } else {
      throw std::invalid_argument("unknown or incomplete argument: " + argument);
    }
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    Metavision::FileConfigHints file_hints;
    file_hints.real_time_playback(false);
    file_hints.time_shift(false);
    auto camera = Metavision::Camera::from_file(options.input_path, file_hints);
    const auto & geometry = camera.geometry();
    evs_benchmark::EvbinWriter writer(
      options.output_path, static_cast<std::uint32_t>(geometry.get_width()),
      static_cast<std::uint32_t>(geometry.get_height()));
    std::ofstream callback_output;
    if (!options.callback_path.empty()) {
      callback_output.open(options.callback_path, std::ios::trunc);
      if (!callback_output) {
        throw std::runtime_error("cannot create callback CSV: " + options.callback_path);
      }
      callback_output << "callback_index,decoded_event_count,written_event_count,"
        "dropped_nonmonotonic_events,first_timestamp_us,last_timestamp_us,"
        "sensor_span_us,wall_since_previous_us\n";
    }

    std::uint64_t callback_index = 0;
    std::uint64_t decoded_events = 0;
    std::uint64_t dropped_nonmonotonic_events = 0;
    std::uint64_t callbacks_with_drops = 0;
    std::int64_t max_lateness_us = 0;
    std::int64_t timestamp_watermark_us = std::numeric_limits<std::int64_t>::min();
    auto previous_callback = std::chrono::steady_clock::now();
    const auto start = previous_callback;
    camera.cd().add_callback(
      [&](const Metavision::EventCD * begin, const Metavision::EventCD * end) {
        const auto now = std::chrono::steady_clock::now();
        const auto count = static_cast<std::size_t>(end - begin);
        std::vector<evs_benchmark::Event> converted;
        converted.reserve(count);
        std::uint64_t callback_drops = 0;
        for (std::size_t index = 0; index < count; ++index) {
          ++decoded_events;
          const auto timestamp_us = static_cast<std::int64_t>(begin[index].t);
          if (options.timestamp_policy == TimestampPolicy::DropNonmonotonic &&
            timestamp_us < timestamp_watermark_us)
          {
            ++callback_drops;
            ++dropped_nonmonotonic_events;
            max_lateness_us = std::max(
              max_lateness_us, timestamp_watermark_us - timestamp_us);
            continue;
          }
          timestamp_watermark_us = std::max(timestamp_watermark_us, timestamp_us);
          converted.push_back(evs_benchmark::Event{
            timestamp_us,
            begin[index].x,
            begin[index].y,
            static_cast<std::uint8_t>(begin[index].p ? 1U : 0U),
          });
        }
        if (!converted.empty()) {
          writer.append(converted.data(), converted.size());
        }
        callbacks_with_drops += callback_drops > 0 ? 1U : 0U;
        if (callback_output && count > 0) {
          callback_output << callback_index << ',' << count << ',' << converted.size() << ',' <<
            callback_drops << ',' << begin->t << ',' <<
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
    std::ofstream stats_output(options.stats_path, std::ios::trunc);
    if (!stats_output) {
      throw std::runtime_error("cannot create conversion stats: " + options.stats_path);
    }
    const auto dropped_fraction = decoded_events > 0 ?
      static_cast<double>(dropped_nonmonotonic_events) /
      static_cast<double>(decoded_events) : 0.0;
    stats_output << std::setprecision(12) <<
      "{\n"
      "  \"input_raw\": \"" << json_escape(options.input_path) << "\",\n"
      "  \"output_evbin\": \"" << json_escape(options.output_path) << "\",\n"
      "  \"timestamp_policy\": \"" << timestamp_policy_name(options.timestamp_policy) << "\",\n"
      "  \"decoded_events\": " << decoded_events << ",\n"
      "  \"written_events\": " << writer.event_count() << ",\n"
      "  \"dropped_nonmonotonic_events\": " << dropped_nonmonotonic_events << ",\n"
      "  \"dropped_fraction\": " << dropped_fraction << ",\n"
      "  \"callbacks\": " << callback_index << ",\n"
      "  \"callbacks_with_drops\": " << callbacks_with_drops << ",\n"
      "  \"max_lateness_us\": " << max_lateness_us << ",\n"
      "  \"conversion_wall_s\": " << wall_s << "\n"
      "}\n";
    stats_output.close();
    if (!stats_output) {
      throw std::runtime_error("failed writing conversion stats: " + options.stats_path);
    }
    std::cout << "converted_events=" << writer.event_count() <<
      " decoded_events=" << decoded_events <<
      " dropped_nonmonotonic=" << dropped_nonmonotonic_events <<
      " dropped_fraction=" << std::fixed << std::setprecision(8) << dropped_fraction <<
      " max_lateness_us=" << max_lateness_us <<
      " callbacks=" << callback_index << " wall_s=" << std::setprecision(3) << wall_s <<
      " throughput_mev_s=" <<
      (wall_s > 0.0 ? static_cast<double>(writer.event_count()) / wall_s / 1.0e6 : 0.0) << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "evs_raw_to_evbin: " << error.what() << '\n';
    return 1;
  }
}
