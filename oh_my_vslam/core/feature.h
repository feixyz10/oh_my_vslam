#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "common/common.h"
#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/map_point.h"

namespace oh_my_vslam {

struct Feature {
  using Ptr = std::shared_ptr<Feature>;
  using ConstPtr = std::shared_ptr<const Feature>;

  explicit Feature(const cv::Point2f &pt, const Frame::Ptr &frame = nullptr,
                   const MapPoint::Ptr &map_point = nullptr)
      : pt(pt), frame(frame), map_point(map_point) {}

  cv::Point2f pt;
  std::weak_ptr<Frame> frame;
  std::weak_ptr<MapPoint> map_point;
  bool is_outlier = false;
};

class FeatureExtractor {
 public:
  explicit FeatureExtractor(const YAML::Node &config) : config_(config) {
    gftt_ = cv::GFTTDetector::create(config_["max_feature_number"].as<int>(),
                                     0.01, 20);
  }

  void Process(const StereoFrame::Ptr &frame) {
    DetectFeatures(frame);
    LKTrackFeature(frame);
  }

 protected:
  void DetectFeatures(const Frame::Ptr &frame) {
    // cv::Mat mask(frame->img().size(), CV_8UC1, 255);
    // for (const auto &feat : frame->features()) {
    //   cv::rectangle(mask, feat->pt - cv::Point2f(10, 10),
    //                 feat->pt + cv::Point2f(10, 10), 0, CV_FILLED);
    // }
    std::vector<cv::KeyPoint> keypoints;
    // gftt_->detect(frame->img(), keypoints, mask);
    gftt_->detect(frame->img(), keypoints);
    for (auto &kp : keypoints) {
      frame->features().emplace_back(new Feature(kp.pt, frame));
    }
  }

  // use LK flow to estimate points in the right image
  void LKTrackFeature(const StereoFrame::Ptr &frame) {
    std::vector<cv::Point2f> kps_lft, kps_rgt;
    // initial guess
    for (auto &feat : frame->features()) {
      kps_lft.push_back(feat->pt);
      auto mp = feat->map_point.lock();
      if (mp) {
        double disp = frame->Disparity(mp->position());
        kps_rgt.emplace_back(feat->pt.x - disp, feat->pt.y);
      } else {
        kps_rgt.push_back(feat->pt);
      }
    }
    // KLT
    std::vector<uchar> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(
        frame->img(), frame->img_rgt(), kps_lft, kps_rgt, status, error,
        cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);
    // postprocess
    size_t num_tracked = 0;
    for (size_t i = 0; i < status.size(); ++i) {
      if (status[i]) {
        Feature::Ptr feat(new Feature(kps_rgt[i], frame));
        frame->features_rgt().push_back(feat);
        ++num_tracked;
      } else {
        frame->features_rgt().push_back(nullptr);
      }
    }
    AINFO << "Frame " << frame->id() << ": " << num_tracked
          << " features tracked.";
  }

  YAML::Node config_;
  cv::Ptr<cv::GFTTDetector> gftt_;
};

}  // namespace oh_my_vslam