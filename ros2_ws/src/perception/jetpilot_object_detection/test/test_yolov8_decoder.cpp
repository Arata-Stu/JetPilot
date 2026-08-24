#include <gtest/gtest.h>

#include <vector>

#include "jetpilot_object_detection/yolov8_decoder.hpp"

namespace jetpilot_object_detection
{

TEST(YoloV8Decoder, Infers224CandidateCount)
{
  EXPECT_EQ(infer_candidate_count(6U * 1029U, 2U), 1029U);
  EXPECT_THROW(infer_candidate_count(6175U, 2U), std::invalid_argument);
}

TEST(YoloV8Decoder, DecodesChannelMajorAndAppliesClassAwareNms)
{
  DecoderConfig config;
  config.num_classes = 2U;
  config.network_width = 224;
  config.network_height = 224;
  config.source_width = 224;
  config.source_height = 224;
  config.resize_mode = ResizeMode::kStretch;
  config.confidence_threshold = 0.3F;
  config.nms_threshold = 0.5F;

  // Three candidates, channels [cx, cy, w, h, vehicle, barrier].
  const std::vector<float> values{
    100.0F, 102.0F, 100.0F,
    100.0F, 102.0F, 100.0F,
    40.0F, 40.0F, 40.0F,
    40.0F, 40.0F, 40.0F,
    0.90F, 0.80F, 0.05F,
    0.05F, 0.10F, 0.85F,
  };
  const auto result = decode_yolov8(values.data(), values.size(), config);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0].class_id, 0);
  EXPECT_EQ(result[1].class_id, 1);
}

TEST(YoloV8Decoder, ReversesLetterboxPadding)
{
  DecoderConfig config;
  config.num_classes = 2U;
  config.network_width = 224;
  config.network_height = 224;
  config.source_width = 424;
  config.source_height = 240;
  config.resize_mode = ResizeMode::kLetterbox;
  config.confidence_threshold = 0.3F;

  // A box spanning the resized source region: x=[0,224], y=[48.6,175.4].
  const std::vector<float> values{112.0F, 112.0F, 224.0F, 126.792F, 0.9F, 0.1F};
  const auto result = decode_yolov8(values.data(), values.size(), config);
  ASSERT_EQ(result.size(), 1U);
  EXPECT_NEAR(result[0].x_min, 0.0F, 0.1F);
  EXPECT_NEAR(result[0].y_min, 0.0F, 0.2F);
  EXPECT_NEAR(result[0].x_max, 424.0F, 0.1F);
  EXPECT_NEAR(result[0].y_max, 240.0F, 0.2F);
}

}  // namespace jetpilot_object_detection
