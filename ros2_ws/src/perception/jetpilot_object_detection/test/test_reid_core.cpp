#include "jetpilot_object_detection/reid_core.hpp"
#include "jetpilot_object_detection/reid_backend.hpp"
#include <iostream>
#include <limits>

using namespace jetpilot_object_detection::reid;
void require(bool condition, const char * message)
{if (!condition) throw std::runtime_error(message);}
template<class F> void rejects(F action)
{
  bool threw = false;
  try {action();} catch (const std::invalid_argument &) {threw = true;}
  require(threw, "expected invalid_argument");
}

void buffer_tests()
{
  FrameBuffer<int, int> buffer(2, 8, 0.5);
  auto image = std::make_shared<const int>(42);
  const std::weak_ptr<const int> weak = image;
  const auto detections = std::make_shared<const int>(1);
  buffer.image({1, "camera"}, image, 4, 0);
  image.reset();
  require(!weak.expired(), "buffer must retain original image");
  buffer.detections({1, "camera"}, detections, false, 0.1);
  auto ready = buffer.take_ready(0.1);
  require(ready.size() == 1 && *ready[0].image == 42, "pair must match");
  ready.clear();
  require(weak.expired(), "processed image must be released");
  buffer.image({1, "camera"}, std::make_shared<const int>(0), 4, 0.2);
  require(buffer.size() == 0, "duplicate completed frame must be ignored");
  buffer.detections({2, "camera"}, detections, true, 0.2);
  buffer.image({2, "camera"}, std::make_shared<const int>(0), 4, 0.3);
  require(buffer.size() == 0, "empty detections arriving first must discard later image");
  buffer.detections({3, "camera"}, detections, false, 0.2);
  buffer.image({3, "other"}, std::make_shared<const int>(0), 4, 0.3);
  require(buffer.take_ready(0.3).empty(), "same stamp different frame must not match");
  buffer.image({3, "camera"}, std::make_shared<const int>(0), 4, 0.3);
  require(buffer.take_ready(0.3).size() == 1, "detection-first pairing");
  buffer.expire(1);
  require(buffer.size() == 0, "missing results expire");
  buffer.image({4, "camera"}, std::make_shared<const int>(0), 8, 1);
  buffer.image({5, "camera"}, std::make_shared<const int>(0), 8, 1.1);
  require(buffer.size() == 1, "byte limit must evict oldest image");
  buffer.image({6, "camera"}, std::make_shared<const int>(0), 9, 1.1);
  require(buffer.size() == 1, "oversized image must not be retained");
  buffer.detections({5, "camera"}, detections, true, 1.2);
  require(buffer.size() == 0, "empty result releases image");
  buffer.clear();
  buffer.image({5, "camera"}, std::make_shared<const int>(0), 4, 2);
  require(buffer.size() == 1, "clock reset clears tombstones");
}

void preprocess_tests()
{
  // Two rows with non-pixel padding; RGB planes should preserve all four corners.
  std::vector<uint8_t> pixels{255,0,0, 0,255,0, 99,99, 0,0,255, 255,255,255, 99,99};
  ImageView view{pixels.data(), pixels.size(), 8, 2, 2, false};
  PreprocessConfig config{2, 2, {0,0,0}, {1,1,1}};
  const auto tensor = preprocess(view, {1,1,2,2}, config);
  require(tensor == std::vector<float>({1,0,0,1, 0,1,0,1, 0,0,1,1}), "RGB NCHW/stride");
  view.bgr = true;
  auto bgr = preprocess(view, {1,1,2,2}, config);
  require(bgr[0] == 0 && bgr[2] == 1 && bgr[8] == 1, "BGR conversion");
  config.width = config.height = 1;
  auto resized = preprocess(view, {1,1,2,2}, config);
  require(resized == std::vector<float>({0.5F,0.5F,0.5F}), "bilinear average");
  config.mean = {0.5F,0.5F,0.5F};
  config.stddev = {0.5F,0.5F,0.5F};
  require(preprocess(view, {1,1,2,2}, config)[0] == 0, "normalization");
  rejects([&]() {preprocess(view, {0,0,2,2}, config);});
  rejects([&]() {preprocess(view, {1,1,std::nan(""),2}, config);});
  view.bytes = 7;
  rejects([&]() {preprocess(view, {1,1,2,2}, config);});
  config.stddev[0] = 0;
  rejects([&]() {config.validate();});
}

void gallery_tests()
{
  Gallery gallery({0.8, 0.5, 0.08, 10, 3, 2});
  std::set<std::string> reserved;
  auto first = gallery.match({1,0,0}, 1, reserved);
  require(!first.id.empty() && !first.confirmed, "first observation tentative");
  auto conflict = gallery.match({1,0,0}, 1, reserved);
  require(conflict.id.empty() && conflict.status == "identity_conflict", "no duplicate ID per frame");
  reserved.clear();
  auto recovered = gallery.match({2,0,0}, 2, reserved);
  require(recovered.id == first.id && recovered.confirmed, "normalize and recover same identity");
  auto other = gallery.match({0,1,0}, 2, reserved);
  require(other.id != first.id && !other.id.empty(), "distinct identity");
  reserved.clear();
  auto ambiguous = gallery.match({1,1,0}, 3, reserved);
  require(ambiguous.id.empty() && ambiguous.status == "ambiguous", "ambiguous must not create ID");
  require(gallery.match({0,0,0}, 3, reserved).status == "invalid_embedding", "zero embedding rejected");
  require(gallery.match({1,std::nanf(""),0}, 3, reserved).status == "invalid_embedding", "NaN rejected");
  require(gallery.match({1,0}, 3, reserved).status == "dimension_mismatch", "wrong dimension rejected");
  require(gallery.match({1,0,0}, 1, reserved).status == "stale_observation", "old embedding cannot update gallery");
  auto third = gallery.match({0,0,1}, 3, reserved);
  require(!third.id.empty(), "third identity");
  require(gallery.match({-1,0,0}, 4, reserved).status == "gallery_full", "bounded gallery");
  reserved.clear();
  auto expired = gallery.match({1,0,0}, 20, reserved);
  require(expired.id != first.id, "expired gallery identity not reused");
  gallery.clear(); reserved.clear();
  require(gallery.match({1,0,0}, 0, reserved).id != expired.id, "clock reset never reuses IDs");
}

void backend_contract_tests()
{
  const InferenceRequest request{7, {123, "camera"}, "track_a", 1, 1, {1,0,0}};
  InferenceResult result{7, {123, "camera"}, "track_a", {1,0}};
  validate_result(request, result, 2);
  result.request_id = 8;
  rejects([&]() {validate_result(request, result, 2);});
  result.request_id = 7; result.frame.stamp = 124;
  rejects([&]() {validate_result(request, result, 2);});
  result.frame.stamp = 123; result.frame.frame = "other";
  rejects([&]() {validate_result(request, result, 2);});
  result.frame.frame = "camera"; result.track_id = "track_b";
  rejects([&]() {validate_result(request, result, 2);});
  result.track_id = "track_a";
  rejects([&]() {validate_result(request, result, 3);});
}

int main()
{
  try {buffer_tests(); preprocess_tests(); gallery_tests(); backend_contract_tests();}
  catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
  std::cout << "ReID buffer, preprocessing and gallery tests passed\n";
}
