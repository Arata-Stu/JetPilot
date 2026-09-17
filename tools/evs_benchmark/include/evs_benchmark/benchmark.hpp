#ifndef EVS_BENCHMARK__BENCHMARK_HPP_
#define EVS_BENCHMARK__BENCHMARK_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace evs_benchmark
{

struct Event
{
  std::int64_t timestamp_us{0};
  std::uint16_t x{0};
  std::uint16_t y{0};
  std::uint8_t polarity{0};
  std::uint8_t padding[3]{0, 0, 0};
};
static_assert(sizeof(Event) == 16, "EVSBIN event layout must remain stable");

struct Dataset
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<Event> events;
};

class EvbinWriter
{
public:
  EvbinWriter(const std::string & path, std::uint32_t width, std::uint32_t height);
  ~EvbinWriter();
  EvbinWriter(EvbinWriter &&) noexcept;
  EvbinWriter & operator=(EvbinWriter &&) noexcept;
  EvbinWriter(const EvbinWriter &) = delete;
  EvbinWriter & operator=(const EvbinWriter &) = delete;

  void append(const Event * events, std::size_t count);
  void close();
  std::uint64_t event_count() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct Config
{
  std::uint32_t width{212};
  std::uint32_t height{120};
  std::size_t bins{10};
  std::int64_t window_us{40000};
  std::int64_t stride_us{4000};
  bool capture_sequence_checksums{false};
  bool capture_trace{false};
};

struct SnapshotTrace
{
  std::uint64_t snapshot_index{0};
  std::int64_t end_timestamp_us{0};
  std::uint64_t new_events{0};
  std::uint64_t window_events{0};
  double wall_ms{0.0};
  double host_staging_ms{0.0};
  double h2d_ms{0.0};
  double update_ms{0.0};
  double snapshot_ms{0.0};
};

struct Result
{
  std::string backend;
  std::string algorithm;
  std::size_t trial{0};
  std::uint64_t events{0};
  std::uint64_t snapshots{0};
  double wall_ms{0.0};
  double cpu_ms{0.0};
  double cpu_util_pct{0.0};
  double host_staging_ms{0.0};
  double h2d_ms{0.0};
  double update_ms{0.0};
  double snapshot_ms{0.0};
  double gpu_total_ms{0.0};
  double checksum{0.0};
  std::uint64_t sequence_checksum{0};
  std::vector<std::uint64_t> snapshot_checksums;
  std::vector<SnapshotTrace> trace;
};

Dataset read_evbin(
  const std::string & path, std::int64_t start_offset_us = 0,
  std::int64_t duration_us = 0);
void write_evbin(const std::string & path, const Dataset & dataset);
Dataset make_synthetic(
  std::uint32_t width, std::uint32_t height, double duration_s,
  double event_rate_meps, std::uint64_t seed);

void validate(const Dataset & dataset, const Config & config);
std::uint64_t tensor_checksum64(const std::vector<float> & tensor);
std::uint64_t append_sequence_checksum(
  std::uint64_t sequence_checksum, std::uint64_t snapshot_checksum,
  std::uint64_t snapshot_index);
Result run_cpu_full(const Dataset & dataset, const Config & config, std::size_t trial);
Result run_cpu_incremental(const Dataset & dataset, const Config & config, std::size_t trial);

bool cuda_available();
std::string cuda_device_description();
Result run_cuda_rolling(const Dataset & dataset, const Config & config, std::size_t trial);

}  // namespace evs_benchmark

#endif  // EVS_BENCHMARK__BENCHMARK_HPP_
