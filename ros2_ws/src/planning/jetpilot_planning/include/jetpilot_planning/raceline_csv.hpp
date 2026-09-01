#ifndef JETPILOT_PLANNING__RACELINE_CSV_HPP_
#define JETPILOT_PLANNING__RACELINE_CSV_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace jetpilot_planning
{

struct RacelinePoint
{
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double psi{0.0};
  double kappa{0.0};
  double vx{0.0};
  double ax{0.0};
};

struct RacelineCsvLimits
{
  static constexpr std::uintmax_t kHardMaxFileBytes = 64U * 1024U * 1024U;
  static constexpr std::size_t kHardMaxPoints = 1000000U;
  static constexpr std::size_t kHardMaxLineBytes = 4096U;

  std::uintmax_t max_file_bytes{16U * 1024U * 1024U};
  std::size_t max_points{200000U};
  std::size_t min_points{2U};
  std::size_t max_line_bytes{kHardMaxLineBytes};
};

struct RacelineData
{
  std::filesystem::path source_path;
  std::vector<RacelinePoint> points;
  std::string source_hash;
};

struct RacelineFileSignature
{
  std::uintmax_t device{0U};
  std::uintmax_t inode{0U};
  std::uintmax_t size{0U};
  std::int64_t modified_seconds{0};
  std::int64_t modified_nanoseconds{0};
  std::int64_t changed_seconds{0};
  std::int64_t changed_nanoseconds{0};

  bool operator==(const RacelineFileSignature & other) const;
  bool operator!=(const RacelineFileSignature & other) const {return !(*this == other);}
};

struct StableRacelineData
{
  RacelineData data;
  RacelineFileSignature signature;
};

// Resolve a requested CSV below raceline_root. If root is empty, requested must
// be an absolute path and its parent directory becomes the safety boundary.
std::filesystem::path resolve_raceline_csv_path(
  const std::filesystem::path & raceline_root,
  const std::filesystem::path & requested);

RacelineData load_raceline_csv(
  const std::filesystem::path & raceline_root,
  const std::filesystem::path & requested,
  const RacelineCsvLimits & limits = RacelineCsvLimits{});

std::optional<RacelineFileSignature> raceline_file_signature(
  const std::filesystem::path & path);

// Resolve and parse a CSV only when its inode and metadata remain unchanged
// for the complete read. This makes an atomic GUI replace an all-or-nothing
// trajectory revision from the publisher's perspective.
StableRacelineData load_stable_raceline_csv(
  const std::filesystem::path & raceline_root,
  const std::filesystem::path & requested,
  const RacelineCsvLimits & limits = RacelineCsvLimits{});

}  // namespace jetpilot_planning

#endif  // JETPILOT_PLANNING__RACELINE_CSV_HPP_
