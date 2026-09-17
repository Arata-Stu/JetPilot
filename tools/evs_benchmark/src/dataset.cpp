#include "evs_benchmark/benchmark.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>

namespace evs_benchmark
{

std::uint64_t tensor_checksum64(const std::vector<float> & tensor)
{
  std::uint64_t hash = 1469598103934665603ULL;
  for (const auto value : tensor) {
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    hash ^= bits;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t append_sequence_checksum(
  std::uint64_t sequence_checksum, const std::uint64_t snapshot_checksum,
  const std::uint64_t snapshot_index)
{
  sequence_checksum ^= snapshot_checksum;
  sequence_checksum *= 1099511628211ULL;
  sequence_checksum ^= snapshot_index;
  sequence_checksum *= 1099511628211ULL;
  return sequence_checksum;
}
namespace
{

struct FileHeader
{
  std::array<char, 8> magic{};
  std::uint32_t version{1};
  std::uint32_t header_bytes{sizeof(FileHeader)};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint64_t event_count{0};
  std::uint64_t timestamp_unit_ns{1000};
  std::array<std::uint8_t, 24> reserved{};
};
static_assert(sizeof(FileHeader) == 64, "EVSBIN header layout must remain stable");

constexpr std::array<char, 8> kMagic{'E', 'V', 'S', 'B', 'E', 'N', 'C', 'H'};

void require_little_endian()
{
  const std::uint16_t marker = 1U;
  if (*reinterpret_cast<const std::uint8_t *>(&marker) != 1U) {
    throw std::runtime_error("EVSBIN v1 supports little-endian hosts only");
  }
}

}  // namespace

struct EvbinWriter::Impl
{
  std::string path;
  FileHeader header;
  std::ofstream output;
  bool closed{false};
};

EvbinWriter::EvbinWriter(
  const std::string & path, const std::uint32_t width, const std::uint32_t height)
: impl_(std::make_unique<Impl>())
{
  require_little_endian();
  impl_->path = path;
  impl_->header.magic = kMagic;
  impl_->header.width = width;
  impl_->header.height = height;
  impl_->output.open(path, std::ios::binary | std::ios::trunc);
  if (!impl_->output) {
    throw std::runtime_error("cannot create EVSBIN output: " + path);
  }
  impl_->output.write(
    reinterpret_cast<const char *>(&impl_->header), sizeof(impl_->header));
}

EvbinWriter::~EvbinWriter()
{
  try {
    close();
  } catch (...) {
  }
}

EvbinWriter::EvbinWriter(EvbinWriter &&) noexcept = default;
EvbinWriter & EvbinWriter::operator=(EvbinWriter &&) noexcept = default;

void EvbinWriter::append(const Event * events, const std::size_t count)
{
  if (!impl_ || impl_->closed) {
    throw std::runtime_error("cannot append to a closed EVSBIN writer");
  }
  impl_->output.write(
    reinterpret_cast<const char *>(events),
    static_cast<std::streamsize>(count * sizeof(Event)));
  if (!impl_->output) {
    throw std::runtime_error("failed writing EVSBIN payload: " + impl_->path);
  }
  impl_->header.event_count += count;
}

void EvbinWriter::close()
{
  if (!impl_ || impl_->closed) {
    return;
  }
  impl_->output.seekp(0);
  impl_->output.write(
    reinterpret_cast<const char *>(&impl_->header), sizeof(impl_->header));
  impl_->output.close();
  impl_->closed = true;
  if (!impl_->output) {
    throw std::runtime_error("failed finalizing EVSBIN output: " + impl_->path);
  }
}

std::uint64_t EvbinWriter::event_count() const
{
  return impl_ ? impl_->header.event_count : 0U;
}

Dataset read_evbin(
  const std::string & path, const std::int64_t start_offset_us,
  const std::int64_t duration_us)
{
  require_little_endian();
  if (start_offset_us < 0 || duration_us < 0) {
    throw std::invalid_argument("EVSBIN segment offset and duration must be non-negative");
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open EVSBIN input: " + path);
  }
  FileHeader header;
  input.read(reinterpret_cast<char *>(&header), sizeof(header));
  if (!input || header.magic != kMagic || header.version != 1U ||
    header.header_bytes != sizeof(FileHeader) || header.timestamp_unit_ns != 1000U)
  {
    throw std::runtime_error("invalid or unsupported EVSBIN header: " + path);
  }
  if (header.event_count > std::numeric_limits<std::size_t>::max() / sizeof(Event)) {
    throw std::runtime_error("EVSBIN event count is too large");
  }
  const auto event_count = static_cast<std::size_t>(header.event_count);
  auto read_timestamp = [&](const std::size_t index) {
      Event event;
      input.seekg(
        static_cast<std::streamoff>(sizeof(FileHeader) + index * sizeof(Event)), std::ios::beg);
      input.read(reinterpret_cast<char *>(&event), sizeof(event));
      if (!input) {
        throw std::runtime_error("failed seeking EVSBIN event payload: " + path);
      }
      return event.timestamp_us;
    };
  std::size_t selected_begin = 0;
  std::size_t selected_end = event_count;
  if (event_count > 0 && (start_offset_us > 0 || duration_us > 0)) {
    const auto first_timestamp = read_timestamp(0);
    const auto lower_bound_record = [&](const std::int64_t timestamp) {
        std::size_t left = 0;
        std::size_t right = event_count;
        while (left < right) {
          const auto middle = left + (right - left) / 2U;
          if (read_timestamp(middle) < timestamp) {left = middle + 1U;} else {right = middle;}
        }
        return left;
      };
    const auto start_timestamp = first_timestamp + start_offset_us;
    selected_begin = lower_bound_record(start_timestamp);
    if (duration_us > 0) {
      selected_end = lower_bound_record(start_timestamp + duration_us);
    }
  }
  if (selected_end < selected_begin) {
    selected_end = selected_begin;
  }
  Dataset dataset;
  dataset.width = header.width;
  dataset.height = header.height;
  dataset.events.resize(selected_end - selected_begin);
  input.clear();
  input.seekg(
    static_cast<std::streamoff>(sizeof(FileHeader) + selected_begin * sizeof(Event)), std::ios::beg);
  input.read(
    reinterpret_cast<char *>(dataset.events.data()),
    static_cast<std::streamsize>(dataset.events.size() * sizeof(Event)));
  if (!input && !dataset.events.empty()) {
    throw std::runtime_error("truncated EVSBIN event payload: " + path);
  }
  return dataset;
}

void write_evbin(const std::string & path, const Dataset & dataset)
{
  EvbinWriter writer(path, dataset.width, dataset.height);
  writer.append(dataset.events.data(), dataset.events.size());
  writer.close();
}

Dataset make_synthetic(
  const std::uint32_t width, const std::uint32_t height, const double duration_s,
  const double event_rate_meps, const std::uint64_t seed)
{
  if (width == 0 || height == 0 || duration_s <= 0.0 || event_rate_meps <= 0.0) {
    throw std::invalid_argument("synthetic dimensions, duration, and event rate must be positive");
  }
  const auto count_double = duration_s * event_rate_meps * 1.0e6;
  if (count_double > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
    throw std::overflow_error("synthetic event count is too large");
  }
  const auto count = static_cast<std::size_t>(std::llround(count_double));
  Dataset dataset;
  dataset.width = width;
  dataset.height = height;
  dataset.events.resize(count);
  std::mt19937_64 generator(seed);
  std::uniform_int_distribution<std::uint32_t> x_distribution(0, width - 1U);
  std::uniform_int_distribution<std::uint32_t> y_distribution(0, height - 1U);
  std::bernoulli_distribution polarity_distribution(0.5);
  const double interval_us = 1.0 / event_rate_meps;
  for (std::size_t index = 0; index < count; ++index) {
    auto & event = dataset.events[index];
    event.timestamp_us = static_cast<std::int64_t>(std::floor(index * interval_us));
    event.x = static_cast<std::uint16_t>(x_distribution(generator));
    event.y = static_cast<std::uint16_t>(y_distribution(generator));
    event.polarity = polarity_distribution(generator) ? 1U : 0U;
  }
  return dataset;
}

void validate(const Dataset & dataset, const Config & config)
{
  if (dataset.width == 0 || dataset.height == 0 || dataset.events.empty()) {
    throw std::invalid_argument("dataset is empty or has invalid geometry");
  }
  if (config.width == 0 || config.height == 0 || config.bins == 0 ||
    config.window_us <= 0 || config.stride_us <= 0 ||
    config.window_us % static_cast<std::int64_t>(config.bins) != 0)
  {
    throw std::invalid_argument("invalid tensor geometry or timing");
  }
  const auto bin_width_us = config.window_us / static_cast<std::int64_t>(config.bins);
  if (config.stride_us % bin_width_us != 0 || config.stride_us > config.window_us) {
    throw std::invalid_argument("stride must be an integer number of bins and <= window");
  }
  if (!std::is_sorted(
      dataset.events.begin(), dataset.events.end(),
      [](const Event & left, const Event & right) {
        return left.timestamp_us < right.timestamp_us;
      }))
  {
    throw std::invalid_argument("events must be sorted by sensor timestamp");
  }
}

}  // namespace evs_benchmark
