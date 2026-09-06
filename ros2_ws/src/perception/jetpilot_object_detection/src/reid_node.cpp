#include "jetpilot_object_detection/reid_backend.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "jetpilot_msgs/msg/reid_match_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>

namespace jetpilot_object_detection
{
class ReidNode : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using Detections = vision_msgs::msg::Detection2DArray;
  using Buffer = reid::FrameBuffer<Image, Detections>;
  using Output = jetpilot_msgs::msg::ReidMatchArray;
  struct Job {Buffer::Entry entry; uint64_t generation;};
public:
  explicit ReidNode(const rclcpp::NodeOptions & options) : Node("opponent_reid", options)
  {
    const auto bounded_int = [&](const char * name, int initial, int maximum) {
        const auto value = declare_parameter<int>(name, initial);
        if (value < 1 || value > maximum) throw std::invalid_argument(std::string(name) + " out of range");
        return value;
      };
    const auto positive = [&](const char * name, double initial) {
        const auto value = declare_parameter<double>(name, initial);
        if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(std::string(name) + " must be positive");
        return value;
      };
    if (declare_parameter<std::string>("backend", "tensorrt") != "tensorrt") {
      throw std::invalid_argument("only tensorrt backend is implemented; see reid_backend.hpp for adapter contract");
    }
    preprocess_.width = bounded_int("input_width", 128, 2048);
    preprocess_.height = bounded_int("input_height", 128, 2048);
    const auto mean = declare_parameter<std::vector<double>>(
      "mean", std::vector<double>{0.485, 0.456, 0.406});
    const auto stddev = declare_parameter<std::vector<double>>(
      "stddev", std::vector<double>{0.229, 0.224, 0.225});
    if (mean.size() != 3 || stddev.size() != 3) throw std::invalid_argument("mean/stddev must have 3 elements");
    for (size_t i = 0; i < 3; ++i) {
      preprocess_.mean[i] = static_cast<float>(mean[i]);
      preprocess_.stddev[i] = static_cast<float>(stddev[i]);
    }
    preprocess_.validate();
    trt_.width = preprocess_.width;
    trt_.height = preprocess_.height;
    trt_.embedding_size = bounded_int("embedding_size", 256, 16384);
    trt_.engine_path = declare_parameter<std::string>("engine_path", "");
    trt_.input_name = declare_parameter<std::string>("input_name", "images");
    trt_.output_name = declare_parameter<std::string>("output_name", "embeddings");
    trt_.device = declare_parameter<int>("device", 0);
    if (trt_.engine_path.empty() || trt_.input_name.empty() || trt_.output_name.empty() || trt_.device < 0) {
      throw std::invalid_argument("ReID requires engine_path, binding names and nonnegative device");
    }
    source_width_ = bounded_int("source_width", 424, 16384);
    source_height_ = bounded_int("source_height", 240, 16384);
    ttl_ = positive("image_ttl_seconds", 0.5);
    interval_ = positive("reid_interval_seconds", 0.2);
    min_box_ = positive("min_box_pixels", 16.0);
    min_score_ = positive("min_detection_score", 0.5);
    if (min_score_ > 1) throw std::invalid_argument("min_detection_score must be <= 1");
    max_rois_ = bounded_int("max_rois_per_frame", 3, 32);
    queue_capacity_ = bounded_int("pending_frames", 2, 16);
    const auto buffer_frames = bounded_int("buffer_frames", 16, 256);
    max_bytes_ = static_cast<size_t>(bounded_int("buffer_megabytes", 32, 1024)) * 1024 * 1024;
    buffer_ = std::make_unique<Buffer>(buffer_frames, max_bytes_, ttl_);
    reid::GalleryConfig gallery;
    gallery.threshold = declare_parameter<double>("match_threshold", 0.8);
    gallery.new_threshold = declare_parameter<double>("new_identity_threshold", 0.5);
    gallery.margin = declare_parameter<double>("match_margin", 0.08);
    gallery.ttl = positive("gallery_ttl_seconds", 300.0);
    gallery.capacity = bounded_int("gallery_capacity", 16, 128);
    gallery.confirmation_hits = bounded_int("confirmation_hits", 2, 100);
    gallery_ = std::make_unique<reid::Gallery>(gallery);
    class_name_ = declare_parameter<std::string>("class_name", "vehicle");
    std::ostringstream session;
    std::random_device random;
    session << std::hex << random() << random() <<
      std::chrono::system_clock::now().time_since_epoch().count();
    session_ = session.str();
    output_ = create_publisher<Output>("matches", rclcpp::QoS(10).reliable());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
    const auto qos = rclcpp::SensorDataQoS().keep_last(5);
    // ConstSharedPtr retains the original pixels without cloning them.
    images_ = create_subscription<Image>("image", qos,
      [this](Image::ConstSharedPtr image) {
        if (image->width != static_cast<unsigned>(source_width_) ||
          image->height != static_cast<unsigned>(source_height_) || image->data.size() > max_bytes_ ||
          (image->encoding != "rgb8" && image->encoding != "bgr8")) {
          ++invalid_images_; return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        check_clock();
        buffer_->image(key(image->header), image, image->data.size(), steady());
        enqueue();
      });
    detections_ = create_subscription<Detections>("detections", qos,
      [this](Detections::ConstSharedPtr detections) {
        std::lock_guard<std::mutex> lock(mutex_);
        check_clock();
        if (detections->detections.size() > 512) {++dropped_; return;}
        const bool no_targets = std::none_of(detections->detections.begin(), detections->detections.end(),
          [this](const auto & detection) {return eligible(detection);});
        buffer_->detections(key(detections->header), detections, no_targets, steady());
        if (no_targets) {
          Output output; output.header = detections->header; output.session_id = session_;
          output_->publish(output);
        }
        enqueue();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        check_clock();
        buffer_->expire(steady());
        publish_diagnostics();
      });
    // Initialize and destroy CUDA objects on the worker, propagating startup failures.
    std::promise<void> ready;
    auto started = ready.get_future();
    worker_ = std::thread([this, ready = std::move(ready)]() mutable {
        std::unique_ptr<reid::EmbeddingBackend> backend;
        try {backend = reid::make_tensor_rt_backend(trt_); ready.set_value();}
        catch (...) {ready.set_exception(std::current_exception()); return;}
        run(*backend);
      });
    try {started.get();} catch (...) {worker_.join(); throw;}
  }
  ~ReidNode() override
  {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; jobs_.clear();}
    condition_.notify_one();
    if (worker_.joinable()) worker_.join();
  }
private:
  bool eligible(const vision_msgs::msg::Detection2D & detection) const
  {
    return !detection.id.empty() &&
      std::any_of(detection.results.begin(), detection.results.end(), [this](const auto & result) {
        return result.hypothesis.class_id == class_name_ && std::isfinite(result.hypothesis.score) &&
               result.hypothesis.score >= min_score_;
      });
  }
  static double steady()
  {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  static reid::FrameKey key(const std_msgs::msg::Header & header)
  {return {static_cast<int64_t>(header.stamp.sec) * 1000000000LL + header.stamp.nanosec, header.frame_id};}
  void check_clock()  // mutex held; acquisition time ordering is handled on the worker.
  {
    const auto clock = now().nanoseconds();
    if (last_clock_ && clock < *last_clock_) {
      ++generation_;
      buffer_->clear();
      jobs_.clear();
    }
    last_clock_ = clock;
  }
  void enqueue()
  {
    for (auto & entry : buffer_->take_ready(steady())) {
      if (jobs_.size() >= queue_capacity_) {jobs_.pop_front(); ++dropped_;}
      jobs_.push_back({std::move(entry), generation_});
    }
    condition_.notify_one();
  }
  void run(reid::EmbeddingBackend & backend)
  {
    uint64_t active_generation = 0, request_id = 0;
    std::optional<reid::FrameKey> last_frame;
    std::map<std::string, double> sampled;
    // Keep currently visible track ownership even when its embedding is rate limited.
    std::map<std::string, std::string> owners;
    while (true) {
      Job job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&]() {return stopping_ || !jobs_.empty();});
        if (stopping_) return;
        job = std::move(jobs_.front()); jobs_.pop_front();
      }
      if (job.generation != active_generation ||
        (last_frame && last_frame->frame != job.entry.key.frame)) {
        gallery_->clear(); sampled.clear(); owners.clear(); last_frame.reset();
        active_generation = job.generation;
      }
      if (steady() - job.entry.arrival >= ttl_ ||
        (last_frame && job.entry.key.stamp <= last_frame->stamp)) {++dropped_; continue;}
      last_frame = job.entry.key;
      const double stamp = job.entry.key.stamp * 1e-9;
      for (auto it = sampled.begin(); it != sampled.end();) {
        if (stamp - it->second > interval_) it = sampled.erase(it); else ++it;
      }
      Output output;
      output.header = job.entry.detections->header;
      output.session_id = session_;
      std::set<std::string> reserved, seen_tracks;
      std::set<std::string> visible;
      for (const auto & detection : job.entry.detections->detections) {
        if (!detection.id.empty()) visible.insert(detection.id);
      }
      for (auto it = owners.begin(); it != owners.end();) {
        if (!visible.count(it->first)) it = owners.erase(it); else ++it;
      }
      for (const auto & owner : owners) reserved.insert(owner.second);
      auto next_owners = owners;
      // Evaluate into a copy; stale or failed work must not contaminate the gallery.
      auto gallery = *gallery_;
      size_t processed = 0;
      std::vector<reid::InferenceRequest> requests;
      for (const auto & detection : job.entry.detections->detections) {
        if (detection.id.empty() || !seen_tracks.insert(detection.id).second) continue;
        if (!eligible(detection)) continue;
        jetpilot_msgs::msg::ReidMatch match;
        match.track_id = detection.id;
        match.similarity = -1;
        if (sampled.count(detection.id)) {match.status = "rate_limited";}
        else if (processed >= max_rois_) {match.status = "frame_budget";}
        else {
          try {
            const auto & box = detection.bbox;
            if (!std::isfinite(box.center.theta) || std::abs(box.center.theta) > 1e-6 ||
              box.size_x < min_box_ || box.size_y < min_box_)
            {throw std::invalid_argument("ROI too small or rotated");}
            const auto & image = *job.entry.image;
            reid::InferenceRequest request;
            request.request_id = ++request_id;
            request.frame = job.entry.key;
            request.track_id = detection.id;
            request.width = preprocess_.width; request.height = preprocess_.height;
            request.rgb_nchw = reid::preprocess(
              {image.data.data(), image.data.size(), image.step, static_cast<int>(image.width),
                static_cast<int>(image.height), image.encoding == "bgr8"},
              {box.center.position.x, box.center.position.y, box.size_x, box.size_y}, preprocess_);
            requests.push_back(std::move(request));
            ++processed;
            match.status = "pending";
          } catch (const std::exception &) {match.status = "invalid_roi";}
        }
        output.matches.push_back(std::move(match));
      }
      // All ROI tensors now own their pixels; free the original full-frame reference before inference.
      job.entry.image.reset();
      for (const auto & request : requests) {
        auto & match = *std::find_if(output.matches.begin(), output.matches.end(),
          [&](const auto & item) {return item.track_id == request.track_id;});
        try {
          if (steady() - job.entry.arrival >= ttl_) {match.status = "expired"; continue;}
          const auto result = backend.infer(request);
          reid::validate_result(request, result, trt_.embedding_size);
          // A visible track may update its own identity; other visible tracks keep theirs.
          auto available = reserved;
          const auto owner = next_owners.find(request.track_id);
          if (owner != next_owners.end()) available.erase(owner->second);
          auto identity = gallery.match(result.embedding, stamp, available);
          if (!identity.id.empty()) {
            reserved.insert(identity.id);
            next_owners[request.track_id] = identity.id;
          }
          match.opponent_id = identity.id.empty() ? "" : session_ + ":" + identity.id;
          match.similarity = identity.similarity;
          match.confirmed = identity.confirmed;
          match.status = identity.status;
        } catch (const std::exception & error) {
          ++errors_;
          match.status = "inference_error";
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ReID: %s", error.what());
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_ || job.generation != generation_ || steady() - job.entry.arrival >= ttl_) {
          ++dropped_; continue;
        }
        *gallery_ = std::move(gallery);
        owners = std::move(next_owners);
        for (const auto & request : requests) sampled[request.track_id] = stamp;
        while (sampled.size() > 512) {
          sampled.erase(std::min_element(sampled.begin(), sampled.end(),
            [](const auto & a, const auto & b) {return a.second < b.second;}));
        }
        output_->publish(output);
        ++published_;
      }
    }
  }
  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "JetPilot ReID"; status.hardware_id = "TensorRT";
    status.level = errors_ || invalid_images_ ? 1 : 0;
    status.message = "ReID worker active; counters are cumulative";
    const auto add = [&](const char * name, size_t value) {
        diagnostic_msgs::msg::KeyValue item; item.key = name; item.value = std::to_string(value);
        status.values.push_back(std::move(item));
      };
    add("buffered_frames", buffer_->size()); add("pending_frames", jobs_.size());
    add("dropped_jobs", dropped_); add("invalid_images", invalid_images_);
    add("inference_errors", errors_); add("published_frames", published_);
    message.status.push_back(std::move(status)); diagnostics_->publish(message);
  }
  reid::PreprocessConfig preprocess_;
  reid::TensorRtConfig trt_;
  std::unique_ptr<Buffer> buffer_;
  std::unique_ptr<reid::Gallery> gallery_;
  std::string session_, class_name_;
  int source_width_{}, source_height_{};
  double ttl_{}, interval_{}, min_box_{}, min_score_{};
  size_t max_rois_{}, queue_capacity_{}, max_bytes_{};
  std::mutex mutex_;
  std::condition_variable condition_;
  bool stopping_{false};
  uint64_t generation_{};
  std::optional<int64_t> last_clock_;
  std::deque<Job> jobs_;
  std::atomic<size_t> dropped_{0}, errors_{0}, invalid_images_{0}, published_{0};
  rclcpp::Publisher<Output>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<Image>::SharedPtr images_;
  rclcpp::Subscription<Detections>::SharedPtr detections_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread worker_;
};
}  // namespace jetpilot_object_detection
RCLCPP_COMPONENTS_REGISTER_NODE(jetpilot_object_detection::ReidNode)
