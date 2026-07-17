#include "jetpilot_planning/raceline_csv.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
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

}  // namespace

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

  std::ifstream stream(source_path);
  if (!stream) {
    throw std::runtime_error("could not open raceline CSV: " + source_path.string());
  }

  RacelineData result;
  result.source_path = source_path;
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

}  // namespace jetpilot_planning
