#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace jetpilot_object_detection::reid
{
struct FrameKey
{
  int64_t stamp{};
  std::string frame;
  bool operator==(const FrameKey & other) const
  {return stamp == other.stamp && frame == other.frame;}
};

// Arrival-time TTL uses a monotonic clock, independent of bag pauses and /clock jumps.
// Both input orders are supported. Completed keys are tombstoned against duplicates.
template<class Image, class Detections>
class FrameBuffer
{
public:
  struct Entry
  {
    FrameKey key;
    double arrival;
    std::shared_ptr<const Image> image;
    std::shared_ptr<const Detections> detections;
    size_t bytes{};
  };
  FrameBuffer(size_t capacity, size_t byte_limit, double ttl)
  : capacity_(capacity), byte_limit_(byte_limit), ttl_(ttl) {}
  void clear() {entries_.clear(); done_.clear();}
  void expire(double now)
  {
    while (!entries_.empty() && now - entries_.front().arrival >= ttl_) {
      entries_.pop_front();
    }
    while (!done_.empty() && now - done_.front().second >= ttl_) done_.pop_front();
  }
  void image(FrameKey key, std::shared_ptr<const Image> image, size_t bytes, double now)
  {
    expire(now);
    if (bytes > byte_limit_ || completed(key)) return;
    auto & entry = get(std::move(key), now);
    entry.image = std::move(image);
    entry.bytes = bytes;
    trim();
  }
  void detections(FrameKey key, std::shared_ptr<const Detections> detections,
    bool empty, double now)
  {
    expire(now);
    if (completed(key)) return;
    if (empty) {
      entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
        [&](const Entry & e) {return e.key == key;}), entries_.end());
      finish(std::move(key), now);
      return;
    }
    get(std::move(key), now).detections = std::move(detections);
    trim();
  }
  std::vector<Entry> take_ready(double now)
  {
    expire(now);
    std::vector<Entry> ready;
    for (auto it = entries_.begin(); it != entries_.end();) {
      if (it->image && it->detections) {
        finish(it->key, now);
        ready.push_back(std::move(*it));
        it = entries_.erase(it);
      } else {++it;}
    }
    return ready;
  }
  size_t size() const {return entries_.size();}
private:
  bool completed(const FrameKey & key) const
  {
    return std::any_of(done_.begin(), done_.end(),
      [&](const auto & item) {return item.first == key;});
  }
  void finish(FrameKey key, double now)
  {
    done_.emplace_back(std::move(key), now);
    while (done_.size() > capacity_ * 2) done_.pop_front();
  }
  Entry & get(FrameKey key, double now)
  {
    for (auto & entry : entries_) if (entry.key == key) return entry;
    entries_.push_back({std::move(key), now, {}, {}, 0});
    return entries_.back();
  }
  void trim()
  {
    size_t bytes = 0;
    for (const auto & entry : entries_) bytes += entry.bytes;
    while (entries_.size() > capacity_ || bytes > byte_limit_) {
      bytes -= entries_.front().bytes;
      entries_.pop_front();
    }
  }
  size_t capacity_, byte_limit_;
  double ttl_;
  std::deque<Entry> entries_;
  std::deque<std::pair<FrameKey, double>> done_;
};

struct PreprocessConfig
{
  int width{128}, height{128};
  std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
  std::array<float, 3> stddev{0.229F, 0.224F, 0.225F};
  void validate() const;
};
struct ImageView
{
  const uint8_t * data{};
  size_t bytes{}, step{};
  int width{}, height{};
  bool bgr{};
};
struct Box {double cx{}, cy{}, width{}, height{};};
// Direct bilinear stretch into RGB NCHW float32: (pixel / 255 - mean) / stddev.
// No full-image copy or intermediate ROI allocation.
std::vector<float> preprocess(const ImageView &, const Box &, const PreprocessConfig &);
bool normalize(std::vector<float> & feature);

struct GalleryConfig
{
  double threshold{0.8}, new_threshold{0.5}, margin{0.08}, ttl{300.0};
  size_t capacity{16};
  unsigned confirmation_hits{2};
};
struct Match
{
  std::string id;
  float similarity{-1};
  bool confirmed{false};
  std::string status;
};
class Gallery
{
public:
  explicit Gallery(GalleryConfig config);
  Match match(std::vector<float> feature, double stamp, std::set<std::string> & reserved);
  void clear() {entries_.clear(); dimension_ = 0;}  // IDs never reused in a session.
private:
  struct Identity
  {std::string id; std::vector<float> feature; double last; unsigned hits;};
  GalleryConfig config_;
  std::vector<Identity> entries_;
  size_t dimension_{};
  uint64_t next_{1};
};
}  // namespace jetpilot_object_detection::reid
