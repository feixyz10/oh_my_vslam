#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "common/common.h"
#include "oh_my_vslam/core/feature.h"

namespace oh_my_vslam {

class FeatureTracker {
 public:
  explicit FeatureTracker(int max_feature_num = 100, bool verbose = false)
      : max_feature_num_(max_feature_num), verbose_(verbose) {
    gftt_ = cv::GFTTDetector::create(max_feature_num, 0.01, 20);
  };

  // Track feature points on right image from left image
  size_t Track(const StereoFrame::Ptr &frame);

  // Track feature points on current image from last image
  size_t Track(const Frame::ConstPtr &frame_last, const Frame::Ptr &frame_curr);

 private:
  void DetectFeature(const Frame::Ptr &frame);

  void LKOpticalFlow(const cv::Mat &img1, const cv::Mat &img2,
                     const std::vector<cv::Point2f> &kps1,
                     std::vector<cv::Point2f> *kps2,
                     std::vector<uchar> *status);

  int max_feature_num_ = 100;
  bool verbose_ = false;
  cv::Ptr<cv::GFTTDetector> gftt_;
  DISALLOW_COPY_AND_ASSIGN(FeatureTracker);
};

}  // namespace oh_my_vslam