#ifndef JETPILOT_OBJECT_DETECTION__BYTE_TRACKER_HPP_
#define JETPILOT_OBJECT_DETECTION__BYTE_TRACKER_HPP_
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "jetpilot_object_detection/yolov8_decoder.hpp"
namespace jetpilot_object_detection {
struct TrackerConfig {
  float low_score{0.1F}, high_score{0.35F}, new_score{0.5F}, match_iou{0.3F};
  double lost_seconds{0.5};
  unsigned min_hits{2};
  std::size_t max_tracks{128};
};
struct TrackMatch {std::size_t detection_index; std::uint64_t id;};
// Two-stage IoU association with a timestamp-based constant-velocity predictor.
// This is ByteTrack-inspired, not the reference Kalman-filter implementation.
class ByteTracker {
public:
  explicit ByteTracker(TrackerConfig config = {});
  std::vector<TrackMatch> update(const std::vector<Detection> &, double timestamp, const std::string & frame);
  std::size_t size() const {return tracks_.size();}
private:
  struct Track {
    Detection box;
    std::array<float, 4> velocity{};
    double seen;
    std::uint64_t id;
    unsigned hits{1};
    bool missed{false};
  };
  TrackerConfig config_;
  std::vector<Track> tracks_;
  std::uint64_t next_id_{1};
  double last_time_{0};
  bool initialized_{false};
  std::string frame_;
};
}
#endif
