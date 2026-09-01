#include "jetpilot_planning/raceline_csv.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <sys/stat.h>
#include <utility>

namespace jetpilot_planning
{
namespace
{

std::string trim(std::string value)
{
  const auto not_space = [](const unsigned char character) {
      return std::isspace(character) == 0;
    };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

bool contains_parent_reference(const std::filesystem::path & path)
{
  return std::any_of(
    path.begin(), path.end(), [](const std::filesystem::path & component) {
      return component == "..";
    });
}

bool is_below_or_equal(
  const std::filesystem::path & root, const std::filesystem::path & candidate)
{
  auto root_part = root.begin();
  auto candidate_part = candidate.begin();
  for (; root_part != root.end(); ++root_part, ++candidate_part) {
    if (candidate_part == candidate.end() || *candidate_part != *root_part) {
      return false;
    }
  }
  return true;
}

void validate_limits(const RacelineCsvLimits & limits)
{
  if (limits.max_file_bytes == 0U ||
    limits.max_file_bytes > RacelineCsvLimits::kHardMaxFileBytes)
  {
    throw std::invalid_argument("max_file_bytes is outside the supported safety range");
  }
  if (limits.min_points < 2U || limits.max_points < limits.min_points ||
    limits.max_points > RacelineCsvLimits::kHardMaxPoints)
  {
    throw std::invalid_argument("raceline point limits are outside the supported safety range");
  }
  if (limits.max_line_bytes == 0U ||
    limits.max_line_bytes > RacelineCsvLimits::kHardMaxLineBytes)
  {
    throw std::invalid_argument("max_line_bytes is outside the supported safety range");
  }
}

std::array<std::string, 7U> split_row(const std::string & line, const std::size_t line_number)
{
  std::array<std::string, 7U> fields;
  std::size_t field_start = 0U;
  for (std::size_t field_index = 0U; field_index < fields.size(); ++field_index) {
    const auto delimiter = line.find(';', field_start);
    fields[field_index] = trim(line.substr(field_start, delimiter - field_start));
    const bool last_field = field_index + 1U == fields.size();
    if ((last_field && delimiter != std::string::npos) ||
      (!last_field && delimiter == std::string::npos))
    {
      throw std::runtime_error(
              "raceline row " + std::to_string(line_number) +
              " must contain exactly 7 columns");
    }
    if (!last_field) {
      field_start = delimiter + 1U;
    }
  }
  return fields;
}

std::string compact_header(const std::string & line)
{
  std::string result;
  result.reserve(line.size());
  for (const unsigned char character : line) {
    if (std::isspace(character) == 0) {
      result.push_back(static_cast<char>(std::tolower(character)));
    }
  }
  return result;
}

bool is_supported_header(const std::string & line)
{
  const auto compact = compact_header(line);
  return compact == "s;x;y;psi;kappa;vx;ax" ||
         compact == "s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2";
}

double parse_finite_double(
  const std::string & value, const std::size_t line_number, const std::size_t column_number)
{
  if (value.empty()) {
    throw std::runtime_error(
            "empty value at raceline row " + std::to_string(line_number) + ", column " +
            std::to_string(column_number));
  }
  std::size_t parsed_characters = 0U;
  double parsed = 0.0;
  try {
    parsed = std::stod(value, &parsed_characters);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "invalid number at raceline row " + std::to_string(line_number) + ", column " +
            std::to_string(column_number));
  }
  if (parsed_characters != value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error(
            "non-finite or malformed number at raceline row " + std::to_string(line_number) +
            ", column " + std::to_string(column_number));
  }
  return parsed;
}

RacelinePoint parse_point(const std::string & line, const std::size_t line_number)
{
  const auto fields = split_row(line, line_number);
  return {
    parse_finite_double(fields[0], line_number, 1U),
    parse_finite_double(fields[1], line_number, 2U),
    parse_finite_double(fields[2], line_number, 3U),
    parse_finite_double(fields[3], line_number, 4U),
    parse_finite_double(fields[4], line_number, 5U),
    parse_finite_double(fields[5], line_number, 6U),
    parse_finite_double(fields[6], line_number, 7U)};
}

std::uint32_t rotate_right(const std::uint32_t value, const unsigned int bits)
{
  return (value >> bits) | (value << (32U - bits));
}

std::string sha256_hash(const std::string & contents)
{
  static constexpr std::array<std::uint32_t, 64U> constants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U,
    0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U,
    0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
    0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU,
    0x5b9cca4fU, 0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

  std::vector<std::uint8_t> bytes(contents.begin(), contents.end());
  const auto bit_length = static_cast<std::uint64_t>(bytes.size()) * 8ULL;
  bytes.push_back(0x80U);
  while (bytes.size() % 64U != 56U)
  {
    bytes.push_back(0U);
  }
  for (int shift = 56; shift >= 0; shift -= 8)
  {
    bytes.push_back(static_cast<std::uint8_t>((bit_length >> shift) & 0xffULL));
  }

  std::array<std::uint32_t, 8U> state{
    0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
    0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
  for (std::size_t offset = 0U; offset < bytes.size(); offset += 64U)
  {
    std::array<std::uint32_t, 64U> words{};
    for (std::size_t index = 0U; index < 16U; ++index)
    {
      const auto byte = offset + index * 4U;
      words[index] = (static_cast<std::uint32_t>(bytes[byte]) << 24U) |
        (static_cast<std::uint32_t>(bytes[byte + 1U]) << 16U) |
        (static_cast<std::uint32_t>(bytes[byte + 2U]) << 8U) |
        static_cast<std::uint32_t>(bytes[byte + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index)
    {
      const auto s0 = rotate_right(words[index - 15U], 7U) ^
        rotate_right(words[index - 15U], 18U) ^ (words[index - 15U] >> 3U);
      const auto s1 = rotate_right(words[index - 2U], 17U) ^
        rotate_right(words[index - 2U], 19U) ^ (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + s0 + words[index - 7U] + s1;
    }

    auto a = state[0];
    auto b = state[1];
    auto c = state[2];
    auto d = state[3];
    auto e = state[4];
    auto f = state[5];
    auto g = state[6];
    auto h = state[7];
    for (std::size_t index = 0U; index < words.size(); ++index)
    {
      const auto sum1 = rotate_right(e, 6U) ^ rotate_right(e, 11U) ^ rotate_right(e, 25U);
      const auto choice = (e & f) ^ ((~e) & g);
      const auto temporary1 = h + sum1 + choice + constants[index] + words[index];
      const auto sum0 = rotate_right(a, 2U) ^ rotate_right(a, 13U) ^ rotate_right(a, 22U);
      const auto majority = (a & b) ^ (a & c) ^ (b & c);
      const auto temporary2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temporary1;
      d = c;
      c = b;
      b = a;
      a = temporary1 + temporary2;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
  }

  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const auto word : state)
  {
    output << std::setw(8) << word;
  }
  return output.str();
}

}  // namespace

bool RacelineFileSignature::operator==(const RacelineFileSignature & other) const
{
  return device == other.device && inode == other.inode && size == other.size &&
         modified_seconds == other.modified_seconds &&
         modified_nanoseconds == other.modified_nanoseconds &&
         changed_seconds == other.changed_seconds &&
         changed_nanoseconds == other.changed_nanoseconds;
}

std::optional<RacelineFileSignature> raceline_file_signature(
  const std::filesystem::path & path)
{
  struct stat metadata {};
  if (::stat(path.c_str(), &metadata) != 0 || !S_ISREG(metadata.st_mode) || metadata.st_size < 0)
  {
    return std::nullopt;
  }
#if defined(__APPLE__)
  const auto modified_seconds = metadata.st_mtimespec.tv_sec;
  const auto modified_nanoseconds = metadata.st_mtimespec.tv_nsec;
  const auto changed_seconds = metadata.st_ctimespec.tv_sec;
  const auto changed_nanoseconds = metadata.st_ctimespec.tv_nsec;
#else
  const auto modified_seconds = metadata.st_mtim.tv_sec;
  const auto modified_nanoseconds = metadata.st_mtim.tv_nsec;
  const auto changed_seconds = metadata.st_ctim.tv_sec;
  const auto changed_nanoseconds = metadata.st_ctim.tv_nsec;
#endif
  return RacelineFileSignature{
    static_cast<std::uintmax_t>(metadata.st_dev),
    static_cast<std::uintmax_t>(metadata.st_ino),
    static_cast<std::uintmax_t>(metadata.st_size),
    static_cast<std::int64_t>(modified_seconds),
    static_cast<std::int64_t>(modified_nanoseconds),
    static_cast<std::int64_t>(changed_seconds),
    static_cast<std::int64_t>(changed_nanoseconds)};
}

std::filesystem::path resolve_raceline_csv_path(
  const std::filesystem::path & raceline_root,
  const std::filesystem::path & requested)
{
  if (requested.empty()) {
    throw std::invalid_argument("raceline_csv must not be empty");
  }
  if (contains_parent_reference(requested)) {
    throw std::invalid_argument("raceline_csv must not contain '..' path traversal");
  }

  std::error_code error;
  std::filesystem::path root = raceline_root;
  if (root.empty()) {
    if (!requested.is_absolute()) {
      throw std::invalid_argument(
              "raceline_root is required when raceline_csv is a relative path");
    }
    root = requested.parent_path();
  } else if (!root.is_absolute()) {
    throw std::invalid_argument("raceline_root must be an absolute path");
  }

  const auto canonical_root = std::filesystem::canonical(root, error);
  if (error || !std::filesystem::is_directory(canonical_root)) {
    throw std::runtime_error("raceline_root is not a readable directory: " + root.string());
  }

  const auto unresolved = requested.is_absolute() ? requested : canonical_root / requested;
  const auto unresolved_status = std::filesystem::symlink_status(unresolved, error);
  if (error || unresolved_status.type() == std::filesystem::file_type::not_found) {
    throw std::runtime_error("raceline CSV does not exist: " + unresolved.string());
  }
  if (std::filesystem::is_symlink(unresolved_status)) {
    throw std::runtime_error("raceline CSV must not be a symbolic link: " + unresolved.string());
  }

  const auto canonical_file = std::filesystem::canonical(unresolved, error);
  if (error || !std::filesystem::is_regular_file(canonical_file)) {
    throw std::runtime_error("raceline CSV is not a regular file: " + unresolved.string());
  }
  if (!is_below_or_equal(canonical_root, canonical_file)) {
    throw std::invalid_argument("raceline CSV resolves outside raceline_root");
  }
  if (canonical_file.extension() != ".csv") {
    throw std::invalid_argument("raceline file must use the .csv extension");
  }
  return canonical_file;
}

RacelineData load_raceline_csv(
  const std::filesystem::path & raceline_root,
  const std::filesystem::path & requested,
  const RacelineCsvLimits & limits)
{
  validate_limits(limits);
  const auto source_path = resolve_raceline_csv_path(raceline_root, requested);

  std::error_code error;
  const auto file_bytes = std::filesystem::file_size(source_path, error);
  if (error) {
    throw std::runtime_error("could not read raceline CSV size: " + source_path.string());
  }
  if (file_bytes == 0U || file_bytes > limits.max_file_bytes) {
    throw std::runtime_error(
            "raceline CSV size is outside the configured safety limit: " +
            std::to_string(file_bytes) + " bytes");
  }

  std::ifstream file_stream(source_path, std::ios::binary);
  if (!file_stream) {
    throw std::runtime_error("could not open raceline CSV: " + source_path.string());
  }
  const std::string contents{
    std::istreambuf_iterator<char>(file_stream), std::istreambuf_iterator<char>()};
  if (file_stream.bad())
  {
    throw std::runtime_error("I/O error while reading raceline CSV: " + source_path.string());
  }
  if (contents.size() != file_bytes)
  {
    throw std::runtime_error("raceline CSV changed while it was being opened");
  }
  std::istringstream stream(contents);

  RacelineData result;
  result.source_path = source_path;
  result.source_hash = sha256_hash(contents);
  std::string line;
  std::size_t line_number = 0U;
  bool saw_distinct_position = false;
  while (std::getline(stream, line)) {
    ++line_number;
    if (line.size() > limits.max_line_bytes) {
      throw std::runtime_error(
              "raceline row " + std::to_string(line_number) + " exceeds the line size limit");
    }
    if (line_number == 1U && line.size() >= 3U &&
      static_cast<unsigned char>(line[0]) == 0xEFU &&
      static_cast<unsigned char>(line[1]) == 0xBBU &&
      static_cast<unsigned char>(line[2]) == 0xBFU)
    {
      line.erase(0U, 3U);
    }
    line = trim(std::move(line));
    if (line.empty() || line.front() == '#') {
      continue;
    }
    if (result.points.empty() && is_supported_header(line)) {
      continue;
    }
    if (result.points.size() >= limits.max_points) {
      throw std::runtime_error("raceline CSV exceeds the configured point limit");
    }

    const auto point = parse_point(line, line_number);
    if (point.s < 0.0 || point.vx < 0.0) {
      throw std::runtime_error(
              "raceline s and vx must be non-negative at row " + std::to_string(line_number));
    }
    if (!result.points.empty()) {
      const auto & previous = result.points.back();
      if (point.s <= previous.s) {
        throw std::runtime_error(
                "raceline s must be strictly increasing at row " +
                std::to_string(line_number));
      }
      saw_distinct_position = saw_distinct_position || point.x != previous.x || point.y != previous.y;
    }
    result.points.push_back(point);
  }
  if (stream.bad()) {
    throw std::runtime_error("I/O error while reading raceline CSV: " + source_path.string());
  }
  if (result.points.size() < limits.min_points) {
    throw std::runtime_error(
            "raceline CSV has too few points: " + std::to_string(result.points.size()));
  }
  if (!saw_distinct_position) {
    throw std::runtime_error("raceline CSV has zero geometric length");
  }
  return result;
}

StableRacelineData load_stable_raceline_csv(
  const std::filesystem::path & raceline_root,
  const std::filesystem::path & requested,
  const RacelineCsvLimits & limits)
{
  const auto resolved_path = resolve_raceline_csv_path(raceline_root, requested);
  const auto signature_before = raceline_file_signature(resolved_path);
  if (!signature_before)
  {
    throw std::runtime_error(
            "raceline CSV is not a readable regular file: " + resolved_path.string());
  }
  auto data = load_raceline_csv(raceline_root, requested, limits);
  const auto signature_after = raceline_file_signature(data.source_path);
  if (!signature_after || *signature_after != *signature_before ||
      data.source_path != resolved_path)
  {
    throw std::runtime_error("raceline CSV changed while it was being loaded");
  }
  return {std::move(data), *signature_after};
}

}  // namespace jetpilot_planning
