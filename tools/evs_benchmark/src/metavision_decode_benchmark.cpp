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
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using Clock = std::chrono::steady_clock;

constexpr std::int64_t kEvt3WrapPeriodUs = 1LL << 24;

std::int64_t evt3_phase_us(const std::int64_t timestamp_us)
{
  const auto phase = timestamp_us % kEvt3WrapPeriodUs;
  return phase >= 0 ? phase : phase + kEvt3WrapPeriodUs;
}

std::int64_t distance_to_evt3_wrap_us(const std::int64_t timestamp_us)
{
  const auto phase = evt3_phase_us(timestamp_us);
  return std::min(phase, kEvt3WrapPeriodUs - phase);
}

struct TrialResult
{
  std::size_t trial{0};
  std::uint64_t events{0};
  std::uint64_t callbacks{0};
  std::size_t max_events_per_callback{0};
  std::uint64_t backward_events{0};
  std::uint64_t backward_at_callback_boundary{0};
  std::uint64_t backward_within_callback{0};
  std::uint64_t backward_near_evt3_wrap{0};
  std::uint64_t large_forward_gaps{0};
  std::uint64_t large_forward_gaps_near_evt3_wrap{0};
  std::uint64_t evt3_wrap_boundaries_crossed{0};
  std::uint64_t evt3_wrap_boundaries_with_backward{0};
  std::int64_t first_timestamp_us{0};
  std::int64_t last_timestamp_us{0};
  std::int64_t max_backward_us{0};
  std::int64_t max_sensor_gap_us{0};
  double open_ms{0.0};
  double decode_wall_ms{0.0};
  double decode_cpu_ms{0.0};
  double callback_interval_mean_us{0.0};
  double callback_interval_max_us{0.0};
};

TrialResult run_trial(
  const std::string & path, const std::size_t trial, const bool validate_timestamps,
  const bool time_shift, const std::int64_t wrap_proximity_us,
  const std::int64_t trace_forward_gap_us, std::ostream * anomaly_output)
{
  TrialResult result;
  result.trial = trial;
  const auto open_start = Clock::now();
  Metavision::FileConfigHints hints;
  hints.real_time_playback(false);
  hints.time_shift(time_shift);
  auto camera = Metavision::Camera::from_file(path, hints);
  result.open_ms = std::chrono::duration<double, std::milli>(Clock::now() - open_start).count();

  bool have_timestamp = false;
  std::int64_t previous_timestamp = 0;
  std::set<std::int64_t> wrap_boundaries_with_backward;
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
      const auto callback_index = result.callbacks;
      const auto callback_start_event_index = result.events;
      const auto count = static_cast<std::size_t>(end - begin);
      result.events += count;
      result.max_events_per_callback = std::max(result.max_events_per_callback, count);
      ++result.callbacks;
      if (!validate_timestamps) {
        return;
      }
      std::size_t index_in_callback = 0;
      for (auto event = begin; event != end; ++event, ++index_in_callback) {
        if (!have_timestamp) {
          result.first_timestamp_us = event->t;
        }
        if (have_timestamp) {
          const auto difference = static_cast<std::int64_t>(event->t) - previous_timestamp;
          const auto previous_wrap_distance = distance_to_evt3_wrap_us(previous_timestamp);
          const auto current_wrap_distance = distance_to_evt3_wrap_us(event->t);
          const auto near_wrap = std::min(previous_wrap_distance, current_wrap_distance) <=
            wrap_proximity_us;
          if (difference < 0) {
            ++result.backward_events;
            result.max_backward_us = std::max(result.max_backward_us, -difference);
            if (index_in_callback == 0) {
              ++result.backward_at_callback_boundary;
            } else {
              ++result.backward_within_callback;
            }
            if (near_wrap) {
              ++result.backward_near_evt3_wrap;
              const auto nearest_timestamp = previous_wrap_distance <= current_wrap_distance ?
                previous_timestamp : static_cast<std::int64_t>(event->t);
              wrap_boundaries_with_backward.insert(
                (nearest_timestamp + kEvt3WrapPeriodUs / 2) / kEvt3WrapPeriodUs);
            }
            if (anomaly_output) {
              *anomaly_output << trial << ",backward," << callback_index << ',' <<
                (callback_start_event_index + index_in_callback) << ',' << index_in_callback << ',' <<
                (index_in_callback == 0 ? 1 : 0) << ',' << previous_timestamp << ',' <<
                event->t << ',' << difference << ',' << evt3_phase_us(previous_timestamp) << ',' <<
                evt3_phase_us(event->t) << ',' <<
                std::min(previous_wrap_distance, current_wrap_distance) << '\n';
            }
          } else {
            result.max_sensor_gap_us = std::max(result.max_sensor_gap_us, difference);
            if (difference >= trace_forward_gap_us) {
              ++result.large_forward_gaps;
              if (near_wrap) {
                ++result.large_forward_gaps_near_evt3_wrap;
              }
              if (anomaly_output) {
                *anomaly_output << trial << ",forward_gap," << callback_index << ',' <<
                  (callback_start_event_index + index_in_callback) << ',' << index_in_callback << ',' <<
                  (index_in_callback == 0 ? 1 : 0) << ',' << previous_timestamp << ',' <<
                  event->t << ',' << difference << ',' << evt3_phase_us(previous_timestamp) << ',' <<
                  evt3_phase_us(event->t) << ',' <<
                  std::min(previous_wrap_distance, current_wrap_distance) << '\n';
              }
            }
          }
        }
        previous_timestamp = event->t;
        result.last_timestamp_us = event->t;
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
  if (have_timestamp && result.last_timestamp_us >= result.first_timestamp_us) {
    result.evt3_wrap_boundaries_crossed = static_cast<std::uint64_t>(
      result.last_timestamp_us / kEvt3WrapPeriodUs -
      result.first_timestamp_us / kEvt3WrapPeriodUs);
  }
  result.evt3_wrap_boundaries_with_backward = wrap_boundaries_with_backward.size();
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: evs_raw_decode_bench INPUT.raw OUTPUT.csv "
      "[--trials N] [--warmup N] [--validate-timestamps] "
      "[--time-shift | --no-time-shift] [--anomalies OUTPUT.csv] "
      "[--evt3-wrap-proximity-us N] [--trace-forward-gap-us N]\n";
    return 2;
  }
  try {
    const std::string input(argv[1]);
    const std::string output_path(argv[2]);
    std::size_t trials = 5;
    std::size_t warmup = 1;
    bool validate_timestamps = false;
    bool time_shift = false;
    std::string anomaly_path;
    std::int64_t wrap_proximity_us = 10000;
    std::int64_t trace_forward_gap_us = 1000000;
    for (int index = 3; index < argc; ++index) {
      const std::string argument(argv[index]);
      if (argument == "--trials" && index + 1 < argc) {
        trials = std::stoull(argv[++index]);
      } else if (argument == "--warmup" && index + 1 < argc) {
        warmup = std::stoull(argv[++index]);
      } else if (argument == "--validate-timestamps") {
        validate_timestamps = true;
      } else if (argument == "--time-shift") {
        time_shift = true;
      } else if (argument == "--no-time-shift") {
        time_shift = false;
      } else if (argument == "--anomalies" && index + 1 < argc) {
        anomaly_path = argv[++index];
      } else if (argument == "--evt3-wrap-proximity-us" && index + 1 < argc) {
        wrap_proximity_us = std::stoll(argv[++index]);
      } else if (argument == "--trace-forward-gap-us" && index + 1 < argc) {
        trace_forward_gap_us = std::stoll(argv[++index]);
      } else {
        throw std::invalid_argument("unknown or incomplete argument: " + argument);
      }
    }
    if (!anomaly_path.empty() && !validate_timestamps) {
      throw std::invalid_argument("--anomalies requires --validate-timestamps");
    }
    if (wrap_proximity_us < 0 || trace_forward_gap_us < 0) {
      throw std::invalid_argument("timestamp thresholds must be non-negative");
    }
    for (std::size_t trial = 0; trial < warmup; ++trial) {
      (void)run_trial(
        input, trial, validate_timestamps, time_shift, wrap_proximity_us,
        trace_forward_gap_us, nullptr);
    }
    std::ofstream anomaly_output;
    if (!anomaly_path.empty()) {
      anomaly_output.open(anomaly_path, std::ios::trunc);
      if (!anomaly_output) {throw std::runtime_error("cannot create anomaly CSV: " + anomaly_path);}
      anomaly_output << "trial,kind,callback_index,event_index,index_in_callback,"
        "at_callback_boundary,previous_timestamp_us,current_timestamp_us,delta_us,"
        "previous_evt3_phase_us,current_evt3_phase_us,distance_to_evt3_wrap_us\n";
    }
    std::ofstream output(output_path, std::ios::trunc);
    if (!output) {throw std::runtime_error("cannot create CSV: " + output_path);}
    output << "trial,validate_timestamps,time_shift,events,callbacks,mean_events_per_callback,"
      "max_events_per_callback,open_ms,decode_wall_ms,decode_cpu_ms,throughput_mev_s,"
      "callback_interval_mean_us,callback_interval_max_us,backward_events,"
      "backward_at_callback_boundary,backward_within_callback,backward_near_evt3_wrap,"
      "max_backward_us,max_sensor_gap_us,large_forward_gaps,"
      "large_forward_gaps_near_evt3_wrap,evt3_wrap_period_us,evt3_wrap_proximity_us,"
      "trace_forward_gap_us,first_timestamp_us,last_timestamp_us,"
      "evt3_wrap_boundaries_crossed,evt3_wrap_boundaries_with_backward\n" <<
      std::setprecision(12);
    for (std::size_t trial = 0; trial < trials; ++trial) {
      const auto result = run_trial(
        input, trial, validate_timestamps, time_shift, wrap_proximity_us,
        trace_forward_gap_us, anomaly_output.is_open() ? &anomaly_output : nullptr);
      const auto mean_events = result.callbacks > 0 ?
        static_cast<double>(result.events) / result.callbacks : 0.0;
      const auto throughput = result.decode_wall_ms > 0.0 ?
        static_cast<double>(result.events) / result.decode_wall_ms / 1000.0 : 0.0;
      output << result.trial << ',' << (validate_timestamps ? 1 : 0) << ',' <<
        (time_shift ? 1 : 0) << ',' <<
        result.events << ',' << result.callbacks << ',' << mean_events << ',' <<
        result.max_events_per_callback << ',' << result.open_ms << ',' <<
        result.decode_wall_ms << ',' << result.decode_cpu_ms << ',' << throughput << ',' <<
        result.callback_interval_mean_us << ',' << result.callback_interval_max_us << ',' <<
        result.backward_events << ',' << result.backward_at_callback_boundary << ',' <<
        result.backward_within_callback << ',' << result.backward_near_evt3_wrap << ',' <<
        result.max_backward_us << ',' << result.max_sensor_gap_us << ',' <<
        result.large_forward_gaps << ',' << result.large_forward_gaps_near_evt3_wrap << ',' <<
        kEvt3WrapPeriodUs << ',' << wrap_proximity_us << ',' << trace_forward_gap_us << ',' <<
        result.first_timestamp_us << ',' << result.last_timestamp_us << ',' <<
        result.evt3_wrap_boundaries_crossed << ',' <<
        result.evt3_wrap_boundaries_with_backward << '\n';
      std::cout << "trial=" << trial << " events=" << result.events <<
        " throughput=" << std::fixed << std::setprecision(3) << throughput << " Mevent/s\n";
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "evs_raw_decode_bench: " << error.what() << '\n';
    return 1;
  }
}
