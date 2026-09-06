#include "jetpilot_object_detection/reid_core.hpp"
#include <limits>

namespace jetpilot_object_detection::reid
{
void PreprocessConfig::validate() const
{
  if (width <= 0 || height <= 0 || width > 2048 || height > 2048) {
    throw std::invalid_argument("ReID input dimensions must be in [1, 2048]");
  }
  for (size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(mean[i]) || !std::isfinite(stddev[i]) || stddev[i] <= 0) {
      throw std::invalid_argument("ReID mean/stddev must be finite with positive stddev");
    }
  }
}

std::vector<float> preprocess(const ImageView & image, const Box & box,
  const PreprocessConfig & config)
{
  config.validate();
  if (!image.data || image.width <= 0 || image.height <= 0 ||
    image.step < static_cast<size_t>(image.width) * 3 ||
    image.step > image.bytes / static_cast<size_t>(image.height))
  {throw std::invalid_argument("invalid RGB/BGR image storage");}
  if (!std::isfinite(box.cx) || !std::isfinite(box.cy) ||
    !std::isfinite(box.width) || !std::isfinite(box.height) ||
    box.width < 2 || box.height < 2)
  {throw std::invalid_argument("invalid ROI");}
  // Reject clipped vehicles: partial appearances can contaminate the identity gallery.
  const double left = box.cx - box.width / 2, top = box.cy - box.height / 2;
  if (left < 0 || top < 0 || left + box.width > image.width ||
    top + box.height > image.height)
  {throw std::invalid_argument("clipped ROI");}
  const size_t plane = static_cast<size_t>(config.width) * config.height;
  std::vector<float> output(plane * 3);
  for (int y = 0; y < config.height; ++y) {
    const double sy = std::clamp(top + (y + 0.5) * box.height / config.height - 0.5,
      0.0, static_cast<double>(image.height - 1));
    const int y0 = static_cast<int>(sy), y1 = std::min(y0 + 1, image.height - 1);
    const double fy = sy - y0;
    for (int x = 0; x < config.width; ++x) {
      const double sx = std::clamp(left + (x + 0.5) * box.width / config.width - 0.5,
        0.0, static_cast<double>(image.width - 1));
      const int x0 = static_cast<int>(sx), x1 = std::min(x0 + 1, image.width - 1);
      const double fx = sx - x0;
      for (size_t c = 0; c < 3; ++c) {
        const auto source_c = image.bgr ? 2 - c : c;
        const auto pixel = [&](int row, int col) {
            return image.data[row * image.step + col * 3 + source_c];
          };
        const double value = (1 - fy) * ((1 - fx) * pixel(y0, x0) + fx * pixel(y0, x1)) +
          fy * ((1 - fx) * pixel(y1, x0) + fx * pixel(y1, x1));
        output[c * plane + y * config.width + x] =
          static_cast<float>((value / 255.0 - config.mean[c]) / config.stddev[c]);
      }
    }
  }
  return output;
}

bool normalize(std::vector<float> & feature)
{
  double sum = 0;
  for (float value : feature) {
    if (!std::isfinite(value)) return false;
    sum += static_cast<double>(value) * value;
  }
  if (sum <= 1e-12 || !std::isfinite(sum)) return false;
  const double scale = 1 / std::sqrt(sum);
  for (auto & value : feature) value = static_cast<float>(value * scale);
  return true;
}

Gallery::Gallery(GalleryConfig config) : config_(config)
{
  if (!std::isfinite(config.threshold) || !std::isfinite(config.new_threshold) ||
    !std::isfinite(config.margin) || !std::isfinite(config.ttl) ||
    config.threshold > 1 || config.new_threshold < -1 ||
    config.new_threshold >= config.threshold || config.margin <= 0 || config.margin > 2 ||
    config.ttl <= 0 || config.capacity == 0 || config.confirmation_hits == 0)
  {throw std::invalid_argument("invalid ReID gallery configuration");}
}

Match Gallery::match(std::vector<float> feature, double stamp, std::set<std::string> & reserved)
{
  if (!std::isfinite(stamp) || !normalize(feature)) return {{}, -1, false, "invalid_embedding"};
  if (dimension_ && dimension_ != feature.size()) return {{}, -1, false, "dimension_mismatch"};
  dimension_ = feature.size();
  entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
    [&](const Identity & identity) {return stamp - identity.last > config_.ttl;}), entries_.end());
  Identity * best = nullptr;
  double score = -2, second = -2;
  for (auto & identity : entries_) {
    double cosine = 0;
    for (size_t i = 0; i < feature.size(); ++i) cosine += feature[i] * identity.feature[i];
    cosine = std::clamp(cosine, -1.0, 1.0);
    if (cosine > score) {second = score; score = cosine; best = &identity;}
    else {second = std::max(second, cosine);}
  }
  if (best && score >= config_.threshold && score - second >= config_.margin) {
    // Do not fall back to another identity when the best identity is already in this frame.
    if (reserved.count(best->id)) return {{}, static_cast<float>(score), false, "identity_conflict"};
    if (stamp <= best->last) return {{}, static_cast<float>(score), false, "stale_observation"};
    for (size_t i = 0; i < feature.size(); ++i) {
      best->feature[i] = 0.9F * best->feature[i] + 0.1F * feature[i];
    }
    normalize(best->feature);
    best->last = stamp;
    if (best->hits < config_.confirmation_hits) ++best->hits;
    reserved.insert(best->id);
    const bool confirmed = best->hits >= config_.confirmation_hits;
    return {best->id, static_cast<float>(score), confirmed, confirmed ? "matched" : "tentative"};
  }
  if (best && score >= config_.new_threshold) {
    return {{}, static_cast<float>(score), false, "ambiguous"};
  }
  if (entries_.size() >= config_.capacity) return {{}, -1, false, "gallery_full"};
  auto id = std::to_string(next_++);
  entries_.push_back({id, std::move(feature), stamp, 1});
  reserved.insert(id);
  return {id, -1, config_.confirmation_hits == 1, "new_identity"};
}
}  // namespace jetpilot_object_detection::reid
