#include "realsense_benchmark/benchmark.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>

namespace realsense_benchmark
{
namespace
{
struct FileHeader
{
  std::array<char, 8> magic{};
  std::uint32_t version{1};
  std::uint32_t header_bytes{sizeof(FileHeader)};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint64_t frame_count{0};
  std::uint32_t channels{3};
  std::uint32_t pixel_format{1};
  std::array<std::uint8_t, 24> reserved{};
};
static_assert(sizeof(FileHeader) == 64, "RGBBIN header must remain stable");
struct FrameHeader {std::uint64_t timestamp_us; std::uint64_t frame_number;};
static_assert(sizeof(FrameHeader) == 16, "RGBBIN frame header must remain stable");
constexpr std::array<char, 8> kMagic{'R', 'G', 'B', 'B', 'E', 'N', 'C', 'H'};

void require_little_endian()
{
  const std::uint16_t marker = 1;
  if (*reinterpret_cast<const std::uint8_t *>(&marker) != 1) {
    throw std::runtime_error("RGBBIN v1 supports little-endian hosts only");
  }
}
}  // namespace

struct RgbbinWriter::Impl
{
  std::string path;
  FileHeader header;
  std::ofstream output;
  std::size_t frame_bytes{0};
  bool closed{false};
};

RgbbinWriter::RgbbinWriter(const std::string & path, std::uint32_t width, std::uint32_t height)
: impl_(std::make_unique<Impl>())
{
  require_little_endian();
  impl_->path = path;
  impl_->header.magic = kMagic;
  impl_->header.width = width;
  impl_->header.height = height;
  impl_->frame_bytes = static_cast<std::size_t>(width) * height * 3U;
  impl_->output.open(path, std::ios::binary | std::ios::trunc);
  if (!impl_->output) {throw std::runtime_error("cannot create RGBBIN: " + path);}
  impl_->output.write(reinterpret_cast<const char *>(&impl_->header), sizeof(impl_->header));
}

RgbbinWriter::~RgbbinWriter() {try {close();} catch (...) {}}
RgbbinWriter::RgbbinWriter(RgbbinWriter &&) noexcept = default;
RgbbinWriter & RgbbinWriter::operator=(RgbbinWriter &&) noexcept = default;

void RgbbinWriter::append(
  std::uint64_t timestamp_us, std::uint64_t frame_number, const std::uint8_t * rgb)
{
  if (!impl_ || impl_->closed) {throw std::runtime_error("RGBBIN writer is closed");}
  const FrameHeader frame_header{timestamp_us, frame_number};
  impl_->output.write(reinterpret_cast<const char *>(&frame_header), sizeof(frame_header));
  impl_->output.write(reinterpret_cast<const char *>(rgb), impl_->frame_bytes);
  if (!impl_->output) {throw std::runtime_error("failed writing RGBBIN: " + impl_->path);}
  ++impl_->header.frame_count;
}

void RgbbinWriter::close()
{
  if (!impl_ || impl_->closed) {return;}
  impl_->output.seekp(0);
  impl_->output.write(reinterpret_cast<const char *>(&impl_->header), sizeof(impl_->header));
  impl_->output.close();
  impl_->closed = true;
  if (!impl_->output) {throw std::runtime_error("failed finalizing RGBBIN: " + impl_->path);}
}
std::uint64_t RgbbinWriter::frame_count() const {return impl_ ? impl_->header.frame_count : 0;}

Dataset read_rgbbin(
  const std::string & path, const std::int64_t start_offset_us, const std::int64_t duration_us)
{
  require_little_endian();
  if (start_offset_us < 0 || duration_us < 0) {throw std::invalid_argument("invalid segment");}
  std::ifstream input(path, std::ios::binary);
  FileHeader header;
  input.read(reinterpret_cast<char *>(&header), sizeof(header));
  if (!input || header.magic != kMagic || header.version != 1 || header.header_bytes != 64 ||
    header.channels != 3 || header.pixel_format != 1)
  {throw std::runtime_error("invalid RGBBIN: " + path);}
  const auto frame_bytes = static_cast<std::size_t>(header.width) * header.height * 3U;
  const auto record_bytes = sizeof(FrameHeader) + frame_bytes;
  const auto count = static_cast<std::size_t>(header.frame_count);
  auto timestamp_at = [&](std::size_t index) {
      FrameHeader frame_header{};
      input.seekg(static_cast<std::streamoff>(64U + index * record_bytes));
      input.read(reinterpret_cast<char *>(&frame_header), sizeof(frame_header));
      if (!input) {throw std::runtime_error("failed seeking RGBBIN");}
      return frame_header.timestamp_us;
    };
  std::size_t begin = 0, end = count;
  if (count && (start_offset_us || duration_us)) {
    const auto origin = timestamp_at(0);
    auto lower = [&](std::uint64_t target) {
        std::size_t left = 0, right = count;
        while (left < right) {
          const auto middle = left + (right - left) / 2U;
          if (timestamp_at(middle) < target) {left = middle + 1;} else {right = middle;}
        }
        return left;
      };
    begin = lower(origin + static_cast<std::uint64_t>(start_offset_us));
    if (duration_us) {end = lower(origin + start_offset_us + duration_us);}
  }
  Dataset dataset;
  dataset.width = header.width;
  dataset.height = header.height;
  dataset.timestamp_us.reserve(end - begin);
  dataset.frame_number.reserve(end - begin);
  dataset.rgb.resize((end - begin) * frame_bytes);
  input.clear();
  input.seekg(static_cast<std::streamoff>(64U + begin * record_bytes));
  for (std::size_t index = 0; index < end - begin; ++index) {
    FrameHeader frame_header{};
    input.read(reinterpret_cast<char *>(&frame_header), sizeof(frame_header));
    input.read(reinterpret_cast<char *>(dataset.rgb.data() + index * frame_bytes), frame_bytes);
    if (!input) {throw std::runtime_error("truncated RGBBIN: " + path);}
    dataset.timestamp_us.push_back(frame_header.timestamp_us);
    dataset.frame_number.push_back(frame_header.frame_number);
  }
  return dataset;
}

void write_rgbbin(const std::string & path, const Dataset & dataset)
{
  RgbbinWriter writer(path, dataset.width, dataset.height);
  for (std::size_t index = 0; index < dataset.frame_count(); ++index) {
    writer.append(dataset.timestamp_us[index], dataset.frame_number[index],
      dataset.rgb.data() + index * dataset.frame_bytes());
  }
  writer.close();
}

Dataset make_synthetic(
  std::uint32_t width, std::uint32_t height, std::size_t frames, std::uint64_t seed)
{
  if (!width || !height || !frames) {throw std::invalid_argument("invalid synthetic dataset");}
  Dataset dataset;
  dataset.width = width;
  dataset.height = height;
  dataset.timestamp_us.resize(frames);
  dataset.frame_number.resize(frames);
  dataset.rgb.resize(frames * dataset.frame_bytes());
  std::mt19937_64 generator(seed);
  std::uniform_int_distribution<unsigned> distribution(0, 255);
  for (std::size_t frame = 0; frame < frames; ++frame) {
    dataset.timestamp_us[frame] = frame * 11111U;
    dataset.frame_number[frame] = frame;
  }
  for (auto & value : dataset.rgb) {value = static_cast<std::uint8_t>(distribution(generator));}
  return dataset;
}

void validate(const Dataset & dataset, const Config & config)
{
  if (!dataset.width || !dataset.height || !dataset.frame_count() || !config.width || !config.height ||
    dataset.timestamp_us.size() != dataset.frame_number.size() ||
    dataset.rgb.size() != dataset.frame_count() * dataset.frame_bytes())
  {throw std::invalid_argument("invalid RGB dataset/config");}
  if (!std::is_sorted(dataset.timestamp_us.begin(), dataset.timestamp_us.end())) {
    throw std::invalid_argument("RGB timestamps are not sorted");
  }
  for (const auto value : config.std) {if (value <= 0) {throw std::invalid_argument("std must be positive");}}
}
}  // namespace realsense_benchmark
