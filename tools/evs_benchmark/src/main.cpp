#include "evs_benchmark/benchmark.hpp"

#include <sys/utsname.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
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
  std::string input;
  std::string output_csv{"results.csv"};
  std::string metadata_json{"metadata.json"};
  std::string backend{"all"};
  std::string algorithm{"all"};
  std::uint32_t source_width{1280};
  std::uint32_t source_height{720};
  double synthetic_duration_s{2.0};
  double synthetic_rate_meps{5.0};
  std::uint64_t seed{42};
  std::int64_t segment_start_us{0};
  std::int64_t segment_duration_us{2000000};
  std::size_t warmup{2};
  std::size_t trials{10};
  bool check_cuda_only{false};
  evs_benchmark::Config config;
};

std::string require_value(int & index, const int argc, char ** argv)
{
  if (++index >= argc) {
    throw std::invalid_argument(std::string("missing value after ") + argv[index - 1]);
  }
  return argv[index];
}

Options parse_options(const int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--input") {options.input = require_value(index, argc, argv);}
    else if (argument == "--output") {options.output_csv = require_value(index, argc, argv);}
    else if (argument == "--metadata") {options.metadata_json = require_value(index, argc, argv);}
    else if (argument == "--backend") {options.backend = require_value(index, argc, argv);}
    else if (argument == "--algorithm") {options.algorithm = require_value(index, argc, argv);}
    else if (argument == "--width") {options.config.width = std::stoul(require_value(index, argc, argv));}
    else if (argument == "--height") {options.config.height = std::stoul(require_value(index, argc, argv));}
    else if (argument == "--bins") {options.config.bins = std::stoull(require_value(index, argc, argv));}
    else if (argument == "--window-us") {options.config.window_us = std::stoll(require_value(index, argc, argv));}
    else if (argument == "--stride-us") {options.config.stride_us = std::stoll(require_value(index, argc, argv));}
    else if (argument == "--trials") {options.trials = std::stoull(require_value(index, argc, argv));}
    else if (argument == "--warmup") {options.warmup = std::stoull(require_value(index, argc, argv));}
    else if (argument == "--source-width") {options.source_width = std::stoul(require_value(index, argc, argv));}
    else if (argument == "--source-height") {options.source_height = std::stoul(require_value(index, argc, argv));}
    else if (argument == "--synthetic-duration-s") {options.synthetic_duration_s = std::stod(require_value(index, argc, argv));}
    else if (argument == "--synthetic-rate-meps") {options.synthetic_rate_meps = std::stod(require_value(index, argc, argv));}
    else if (argument == "--seed") {options.seed = std::stoull(require_value(index, argc, argv));}
    else if (argument == "--segment-start-us") {options.segment_start_us = std::stoll(require_value(index, argc, argv));}
    else if (argument == "--segment-duration-us") {options.segment_duration_us = std::stoll(require_value(index, argc, argv));}
    else if (argument == "--check-cuda") {options.check_cuda_only = true;}
    else if (argument == "--help") {
      std::cout <<
        "evs_bench [--input INPUT.evbin] [--backend all|cpu|cuda] "
        "[--algorithm all|full|incremental|rolling]\n"
        "          [--width 212 --height 120 --bins 10 --window-us 40000 "
        "--stride-us 4000]\n"
        "          [--warmup 2 --trials 10 --output results.csv "
        "--metadata metadata.json]\n"
        "          [--segment-start-us 0 --segment-duration-us 2000000]\n"
        "Without --input, a deterministic synthetic stream is generated.\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.trials == 0) {throw std::invalid_argument("trials must be positive");}
  return options;
}

bool wants(const std::string & selected, const std::string & value)
{
  return selected == "all" || selected == value;
}

void write_csv(const std::string & path, const std::vector<evs_benchmark::Result> & results)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {throw std::runtime_error("cannot create result CSV: " + path);}
  output << "backend,algorithm,trial,events,snapshots,wall_ms,cpu_ms,cpu_util_pct,"
    "host_staging_ms,h2d_ms,update_ms,snapshot_ms,gpu_total_ms,checksum\n";
  output << std::setprecision(12);
  for (const auto & result : results) {
    output << result.backend << ',' << result.algorithm << ',' << result.trial << ',' <<
      result.events << ',' << result.snapshots << ',' << result.wall_ms << ',' << result.cpu_ms <<
      ',' << result.cpu_util_pct << ',' << result.host_staging_ms << ',' << result.h2d_ms << ',' <<
      result.update_ms << ',' << result.snapshot_ms << ',' << result.gpu_total_ms << ',' <<
      result.checksum << '\n';
  }
}

std::string json_escape(const std::string & input)
{
  std::string output;
  for (const char value : input) {
    if (value == '\\' || value == '"') {output.push_back('\\');}
    output.push_back(value);
  }
  return output;
}

void write_metadata(
  const std::string & path, const Options & options, const evs_benchmark::Dataset & dataset,
  const bool has_cuda)
{
  struct utsname system_info {};
  const bool uname_ok = uname(&system_info) == 0;
  std::ofstream output(path, std::ios::trunc);
  if (!output) {throw std::runtime_error("cannot create metadata JSON: " + path);}
  output << "{\n"
    << "  \"schema_version\": 1,\n"
    << "  \"input\": \"" << json_escape(options.input.empty() ? "synthetic" : options.input) << "\",\n"
    << "  \"source_width\": " << dataset.width << ",\n"
    << "  \"source_height\": " << dataset.height << ",\n"
    << "  \"event_count\": " << dataset.events.size() << ",\n"
    << "  \"segment_start_us\": " << options.segment_start_us << ",\n"
    << "  \"segment_duration_us\": " << options.segment_duration_us << ",\n"
    << "  \"output_width\": " << options.config.width << ",\n"
    << "  \"output_height\": " << options.config.height << ",\n"
    << "  \"bins\": " << options.config.bins << ",\n"
    << "  \"window_us\": " << options.config.window_us << ",\n"
    << "  \"stride_us\": " << options.config.stride_us << ",\n"
    << "  \"warmup_trials\": " << options.warmup << ",\n"
    << "  \"measured_trials\": " << options.trials << ",\n"
    << "  \"compiler\": \"" << json_escape(__VERSION__) << "\",\n"
    << "  \"system\": \"" << (uname_ok ? json_escape(system_info.sysname) : "unknown") << "\",\n"
    << "  \"machine\": \"" << (uname_ok ? json_escape(system_info.machine) : "unknown") << "\",\n"
    << "  \"release\": \"" << (uname_ok ? json_escape(system_info.release) : "unknown") << "\",\n"
    << "  \"cuda_available\": " << (has_cuda ? "true" : "false") << ",\n"
    << "  \"cuda_device\": \"" << (has_cuda ? json_escape(evs_benchmark::cuda_device_description()) : "") << "\",\n"
    << "  \"timing_note\": \"GPU stages use CUDA Events; correctness D2H and checksum are excluded from wall_ms\"\n"
    << "}\n";
}

void print_result(const evs_benchmark::Result & result)
{
  std::cout << result.backend << '/' << result.algorithm << " trial=" << result.trial <<
    " wall=" << std::fixed << std::setprecision(3) << result.wall_ms << " ms" <<
    " cpu=" << result.cpu_ms << " ms";
  if (result.backend == "cuda") {
    std::cout << " staging=" << result.host_staging_ms << " ms H2D=" << result.h2d_ms <<
      " ms update=" << result.update_ms << " ms snapshot=" << result.snapshot_ms << " ms";
  }
  std::cout << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    if (options.check_cuda_only) {
      if (!evs_benchmark::cuda_available()) {
        std::cerr << "CUDA unavailable\n";
        return 1;
      }
      std::cout << evs_benchmark::cuda_device_description() << '\n';
      return 0;
    }
    auto dataset = options.input.empty() ?
      evs_benchmark::make_synthetic(
      options.source_width, options.source_height, options.synthetic_duration_s,
      options.synthetic_rate_meps, options.seed) :
      evs_benchmark::read_evbin(
      options.input, options.segment_start_us, options.segment_duration_us);
    evs_benchmark::validate(dataset, options.config);
    const bool has_cuda = evs_benchmark::cuda_available();

    using Runner = evs_benchmark::Result (*)(
      const evs_benchmark::Dataset &, const evs_benchmark::Config &, std::size_t);
    std::vector<Runner> runners;
    if (wants(options.backend, "cpu") && wants(options.algorithm, "full")) {
      runners.push_back(evs_benchmark::run_cpu_full);
    }
    if (wants(options.backend, "cpu") && wants(options.algorithm, "incremental")) {
      runners.push_back(evs_benchmark::run_cpu_incremental);
    }
    if (wants(options.backend, "cuda") && wants(options.algorithm, "rolling")) {
      if (!has_cuda) {
        if (options.backend == "cuda") {throw std::runtime_error("CUDA backend requested but unavailable");}
        std::cerr << "CUDA unavailable; skipping cuda/rolling\n";
      } else {
        runners.push_back(evs_benchmark::run_cuda_rolling);
      }
    }
    if (runners.empty()) {throw std::invalid_argument("no backend/algorithm combination selected");}

    for (std::size_t trial = 0; trial < options.warmup; ++trial) {
      for (const auto runner : runners) {(void)runner(dataset, options.config, trial);}
    }
    std::vector<evs_benchmark::Result> results;
    std::vector<double> reference_checksums(
      options.trials, std::numeric_limits<double>::quiet_NaN());
    for (std::size_t trial = 0; trial < options.trials; ++trial) {
      for (const auto runner : runners) {
        auto result = runner(dataset, options.config, trial);
        if (std::isnan(reference_checksums[trial])) {
          reference_checksums[trial] = result.checksum;
        } else if (std::abs(reference_checksums[trial] - result.checksum) > 1.0e-6) {
          throw std::runtime_error(
                  "correctness check failed: backend checksums differ in trial " +
                  std::to_string(trial));
        }
        print_result(result);
        results.push_back(std::move(result));
      }
    }

    std::filesystem::create_directories(
      std::filesystem::path(options.output_csv).parent_path().empty() ? "." :
      std::filesystem::path(options.output_csv).parent_path());
    std::filesystem::create_directories(
      std::filesystem::path(options.metadata_json).parent_path().empty() ? "." :
      std::filesystem::path(options.metadata_json).parent_path());
    write_csv(options.output_csv, results);
    write_metadata(options.metadata_json, options, dataset, has_cuda);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "evs_bench: " << error.what() << '\n';
    return 1;
  }
}
