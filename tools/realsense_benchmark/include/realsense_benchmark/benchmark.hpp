#ifndef REALSENSE_BENCHMARK__BENCHMARK_HPP_
#define REALSENSE_BENCHMARK__BENCHMARK_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace realsense_benchmark
{

struct Dataset
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<std::uint64_t> timestamp_us;
  std::vector<std::uint64_t> frame_number;
  std::vector<std::uint8_t> rgb;
  std::size_t frame_count() const {return timestamp_us.size();}
  std::size_t frame_bytes() const {return static_cast<std::size_t>(width) * height * 3U;}
};

class RgbbinWriter
{
public:
  RgbbinWriter(const std::string & path, std::uint32_t width, std::uint32_t height);
  ~RgbbinWriter();
  RgbbinWriter(RgbbinWriter &&) noexcept;
  RgbbinWriter & operator=(RgbbinWriter &&) noexcept;
  RgbbinWriter(const RgbbinWriter &) = delete;
  RgbbinWriter & operator=(const RgbbinWriter &) = delete;
  void append(std::uint64_t timestamp_us, std::uint64_t frame_number, const std::uint8_t * rgb);
  void close();
  std::uint64_t frame_count() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct Config
{
  std::uint32_t width{212};
  std::uint32_t height{120};
  std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
  std::array<float, 3> std{0.229F, 0.224F, 0.225F};
};

struct Result
{
  std::string backend;
  std::size_t trial{0};
  std::uint64_t frames{0};
  double wall_ms{0.0};
  double cpu_ms{0.0};
  double cpu_util_pct{0.0};
  double host_staging_ms{0.0};
  double h2d_ms{0.0};
  double preprocess_ms{0.0};
  double gpu_total_ms{0.0};
  double checksum{0.0};
};

Dataset read_rgbbin(const std::string & path, std::int64_t start_offset_us, std::int64_t duration_us);
void write_rgbbin(const std::string & path, const Dataset & dataset);
Dataset make_synthetic(std::uint32_t width, std::uint32_t height, std::size_t frames, std::uint64_t seed);
void validate(const Dataset & dataset, const Config & config);
Result run_cpu(const Dataset & dataset, const Config & config, std::size_t trial);
bool cuda_available();
std::string cuda_device_description();
Result run_cuda(const Dataset & dataset, const Config & config, std::size_t trial);

}  // namespace realsense_benchmark

#endif
