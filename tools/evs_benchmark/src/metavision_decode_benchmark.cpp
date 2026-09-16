#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using Clock = std::chrono::steady_clock;

struct TrialResult
{
  std::size_t trial{0};
  std::uint64_t events{0};
  std::uint64_t callbacks{0};
  std::size_t max_events_per_callback{0};
  std::uint64_t backward_events{0};
  std::int64_t max_backward_us{0};
  std::int64_t max_sensor_gap_us{0};
  double open_ms{0.0};
  double decode_wall_ms{0.0};
  double decode_cpu_ms{0.0};
  double callback_interval_mean_us{0.0};
  double callback_interval_max_us{0.0};
};

TrialResult run_trial(
  const std::string & path, const std::size_t trial, const bool validate_timestamps)
{
  TrialResult result;
  result.trial = trial;
  const auto open_start = Clock::now();
  Metavision::FileConfigHints hints;
  hints.real_time_playback(false);
  hints.time_shift(false);
  auto camera = Metavision::Camera::from_file(path, hints);
  result.open_ms = std::chrono::duration<double, std::milli>(Clock::now() - open_start).count();

  bool have_timestamp = false;
  std::int64_t previous_timestamp = 0;
  bool have_callback = false;
  auto previous_callback = Clock::now();
  double callback_interval_sum_us = 0.0;
  std::uint64_t callback_intervals = 0;
  camera.cd().add_callback(
    [&](const Metavision::EventCD * begin, const Metavision::EventCD * end) {
      const auto now = Clock::now();
      if (have_callback) {
        const auto interval_us = std::chrono::duration<double, std::micro>(
          now - previous_callback).count();
        callback_interval_sum_us += interval_us;
        result.callback_interval_max_us = std::max(
          result.callback_interval_max_us, interval_us);
        ++callback_intervals;
      }
      previous_callback = now;
      have_callback = true;
      const auto count = static_cast<std::size_t>(end - begin);
      result.events += count;
      result.max_events_per_callback = std::max(result.max_events_per_callback, count);
      ++result.callbacks;
      if (!validate_timestamps) {
        return;
      }
      for (auto event = begin; event != end; ++event) {
        if (have_timestamp) {
          const auto difference = static_cast<std::int64_t>(event->t) - previous_timestamp;
          if (difference < 0) {
            ++result.backward_events;
            result.max_backward_us = std::max(result.max_backward_us, -difference);
          } else {
            result.max_sensor_gap_us = std::max(result.max_sensor_gap_us, difference);
          }
        }
        previous_timestamp = event->t;
        have_timestamp = true;
      }
    });

  const auto wall_start = Clock::now();
  const auto cpu_start = std::clock();
  camera.start();
  while (camera.is_running()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  camera.stop();
  result.decode_wall_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - wall_start).count();
  result.decode_cpu_ms = 1000.0 * static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
  result.callback_interval_mean_us = callback_intervals > 0 ?
    callback_interval_sum_us / static_cast<double>(callback_intervals) : 0.0;
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: evs_raw_decode_bench INPUT.raw OUTPUT.csv "
      "[--trials N] [--warmup N] [--validate-timestamps]\n";
    return 2;
  }
  try {
    const std::string input(argv[1]);
    const std::string output_path(argv[2]);
    std::size_t trials = 5;
    std::size_t warmup = 1;
    bool validate_timestamps = false;
    for (int index = 3; index < argc; ++index) {
      const std::string argument(argv[index]);
      if (argument == "--trials" && index + 1 < argc) {
        trials = std::stoull(argv[++index]);
      } else if (argument == "--warmup" && index + 1 < argc) {
        warmup = std::stoull(argv[++index]);
      } else if (argument == "--validate-timestamps") {
        validate_timestamps = true;
      } else {
        throw std::invalid_argument("unknown or incomplete argument: " + argument);
      }
    }
    for (std::size_t trial = 0; trial < warmup; ++trial) {
      (void)run_trial(input, trial, validate_timestamps);
    }
    std::ofstream output(output_path, std::ios::trunc);
    if (!output) {throw std::runtime_error("cannot create CSV: " + output_path);}
    output << "trial,validate_timestamps,events,callbacks,mean_events_per_callback,"
      "max_events_per_callback,open_ms,decode_wall_ms,decode_cpu_ms,throughput_mev_s,"
      "callback_interval_mean_us,callback_interval_max_us,backward_events,"
      "max_backward_us,max_sensor_gap_us\n" << std::setprecision(12);
    for (std::size_t trial = 0; trial < trials; ++trial) {
      const auto result = run_trial(input, trial, validate_timestamps);
      const auto mean_events = result.callbacks > 0 ?
        static_cast<double>(result.events) / result.callbacks : 0.0;
      const auto throughput = result.decode_wall_ms > 0.0 ?
        static_cast<double>(result.events) / result.decode_wall_ms / 1000.0 : 0.0;
      output << result.trial << ',' << (validate_timestamps ? 1 : 0) << ',' <<
        result.events << ',' << result.callbacks << ',' << mean_events << ',' <<
        result.max_events_per_callback << ',' << result.open_ms << ',' <<
        result.decode_wall_ms << ',' << result.decode_cpu_ms << ',' << throughput << ',' <<
        result.callback_interval_mean_us << ',' << result.callback_interval_max_us << ',' <<
        result.backward_events << ',' << result.max_backward_us << ',' <<
        result.max_sensor_gap_us << '\n';
      std::cout << "trial=" << trial << " events=" << result.events <<
        " throughput=" << std::fixed << std::setprecision(3) << throughput << " Mevent/s\n";
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "evs_raw_decode_bench: " << error.what() << '\n';
    return 1;
  }
}
