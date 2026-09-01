#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "jetpilot_planning/raceline_csv.hpp"

namespace
{

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    const auto unique = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    path_ = std::filesystem::temp_directory_path() / ("jetpilot_raceline_test_" + unique);
    std::filesystem::create_directories(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const {return path_;}

private:
  std::filesystem::path path_;
};

void write_file(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream stream(path);
  ASSERT_TRUE(stream.good());
  stream << content;
  ASSERT_TRUE(stream.good());
}

TEST(RacelineCsv, LoadsGeneratedF1tenthLayout)
{
  TemporaryDirectory temporary;
  const auto csv = temporary.path() / "course_raceline.csv";
  write_file(
    csv,
    "# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n"
    "0.0;1.0;2.0;0.1;0.01;1.5;0.2\n"
    "0.5;1.5;2.1;0.2;0.02;1.6;-0.1\n");

  const auto raceline = jetpilot_planning::load_raceline_csv(
    temporary.path(), csv.filename());
  ASSERT_EQ(raceline.points.size(), 2U);
  EXPECT_EQ(raceline.source_path, std::filesystem::canonical(csv));
  EXPECT_EQ(
    raceline.source_hash,
    "a2f52489a056bcbcf214962f32f162372d28f53f96af901fc4f0b2cdbd1d468a");
  EXPECT_DOUBLE_EQ(raceline.points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(raceline.points[1].vx, 1.6);
  EXPECT_DOUBLE_EQ(raceline.points[1].ax, -0.1);
}

TEST(RacelineCsv, ContentHashChangesWithSpeedProfile)
{
  TemporaryDirectory temporary;
  const auto csv = temporary.path() / "custom.csv";
  write_file(csv, "0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n");
  const auto first = jetpilot_planning::load_raceline_csv(temporary.path(), csv.filename());

  write_file(csv, "0;0;0;0;0;1;0\n1;1;0;0;0;0.5;-0.2\n");
  const auto second = jetpilot_planning::load_raceline_csv(temporary.path(), csv.filename());

  EXPECT_NE(first.source_hash, second.source_hash);
}

TEST(RacelineCsv, StableLoadDetectsAtomicReplacement)
{
  TemporaryDirectory temporary;
  const auto csv = temporary.path() / "custom.csv";
  const auto replacement = temporary.path() / "custom.next.csv";
  write_file(csv, "0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n");
  const auto first = jetpilot_planning::load_stable_raceline_csv(
    temporary.path(), csv.filename());

  write_file(replacement, "0;0;0;0;0;0.8;0\n1;1;0;0;0;0.6;-0.1\n");
  std::filesystem::rename(replacement, csv);
  const auto second = jetpilot_planning::load_stable_raceline_csv(
    temporary.path(), csv.filename());

  EXPECT_NE(first.signature, second.signature);
  EXPECT_NE(first.data.source_hash, second.data.source_hash);
  EXPECT_DOUBLE_EQ(second.data.points.back().vx, 0.6);
}

TEST(RacelineCsv, AcceptsUncommentedShortHeader)
{
  TemporaryDirectory temporary;
  const auto csv = temporary.path() / "course_raceline.csv";
  write_file(csv, "s;x;y;psi;kappa;vx;ax\n0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n");

  EXPECT_EQ(
    jetpilot_planning::load_raceline_csv(temporary.path(), csv.filename()).points.size(), 2U);
}

TEST(RacelineCsv, RejectsNonFiniteAndMalformedRows)
{
  TemporaryDirectory temporary;
  const auto nan_csv = temporary.path() / "nan.csv";
  write_file(nan_csv, "0;0;0;0;0;1;0\n1;nan;0;0;0;1;0\n");
  EXPECT_THROW(
    jetpilot_planning::load_raceline_csv(temporary.path(), nan_csv.filename()),
    std::runtime_error);

  const auto extra_csv = temporary.path() / "extra.csv";
  write_file(extra_csv, "0;0;0;0;0;1;0\n1;1;0;0;0;1;0;unexpected\n");
  EXPECT_THROW(
    jetpilot_planning::load_raceline_csv(temporary.path(), extra_csv.filename()),
    std::runtime_error);

  const auto short_csv = temporary.path() / "short.csv";
  write_file(short_csv, "0;0;0;0;0;1;0\n1;1;0;0;0;1\n");
  EXPECT_THROW(
    jetpilot_planning::load_raceline_csv(temporary.path(), short_csv.filename()),
    std::runtime_error);
}

TEST(RacelineCsv, RejectsInvalidDistanceAndSpeedSemantics)
{
  TemporaryDirectory temporary;
  const auto unordered_csv = temporary.path() / "unordered.csv";
  write_file(unordered_csv, "0;0;0;0;0;1;0\n0;1;0;0;0;1;0\n");
  EXPECT_THROW(
    jetpilot_planning::load_raceline_csv(temporary.path(), unordered_csv.filename()),
    std::runtime_error);

  const auto reverse_speed_csv = temporary.path() / "negative_speed.csv";
  write_file(reverse_speed_csv, "0;0;0;0;0;1;0\n1;1;0;0;0;-1;0\n");
  EXPECT_THROW(
    jetpilot_planning::load_raceline_csv(temporary.path(), reverse_speed_csv.filename()),
    std::runtime_error);

  const auto zero_length_csv = temporary.path() / "zero_length.csv";
  write_file(zero_length_csv, "0;1;1;0;0;1;0\n1;1;1;0;0;1;0\n");
  EXPECT_THROW(
    jetpilot_planning::load_raceline_csv(temporary.path(), zero_length_csv.filename()),
    std::runtime_error);
}

TEST(RacelineCsv, EnforcesFileAndPointLimits)
{
  TemporaryDirectory temporary;
  const auto csv = temporary.path() / "course.csv";
  write_file(csv, "0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n2;2;0;0;0;1;0\n");

  jetpilot_planning::RacelineCsvLimits file_limits;
  file_limits.max_file_bytes = 4U;
  EXPECT_THROW(
    jetpilot_planning::load_raceline_csv(temporary.path(), csv.filename(), file_limits),
    std::runtime_error);

  jetpilot_planning::RacelineCsvLimits point_limits;
  point_limits.max_points = 2U;
  EXPECT_THROW(
    jetpilot_planning::load_raceline_csv(temporary.path(), csv.filename(), point_limits),
    std::runtime_error);
}

TEST(RacelineCsv, RejectsTraversalAndFilesOutsideRoot)
{
  TemporaryDirectory temporary;
  const auto map_root = temporary.path() / "map";
  std::filesystem::create_directory(map_root);
  const auto outside = temporary.path() / "outside.csv";
  write_file(outside, "0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n");

  EXPECT_THROW(
    jetpilot_planning::resolve_raceline_csv_path(map_root, "../outside.csv"),
    std::invalid_argument);
  EXPECT_THROW(
    jetpilot_planning::resolve_raceline_csv_path(map_root, outside),
    std::invalid_argument);
  EXPECT_THROW(
    jetpilot_planning::resolve_raceline_csv_path({}, "relative.csv"),
    std::invalid_argument);
}

TEST(RacelineCsv, RejectsSymbolicLinkInput)
{
  TemporaryDirectory temporary;
  const auto target = temporary.path() / "target.csv";
  const auto link = temporary.path() / "linked.csv";
  write_file(target, "0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n");
  std::filesystem::create_symlink(target, link);

  EXPECT_THROW(
    jetpilot_planning::resolve_raceline_csv_path(temporary.path(), link.filename()),
    std::runtime_error);
}

}  // namespace
