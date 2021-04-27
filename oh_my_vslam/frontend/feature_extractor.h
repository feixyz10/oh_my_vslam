#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "common/common.h"
#include "oh_my_vslam/core/feature.h"

namespace oh_my_vslam {

class FeatureExtractor {
 public:
  // explicit FeatureExtractor(const YAML::Node &config) : config_(config) {
  //   gftt_ = cv::GFTTDetector::create(config_["max_feature_number"].as<int>(),
  //                                    0.01, 20);
  // }
  explicit FeatureExtractor(int max_feature_number) {
    gftt_ = cv::GFTTDetector::create(max_feature_number, 0.01, 20);
  }

  size_t Process(const StereoFrame::Ptr &frame) {
    DetectFeatures(frame);
    return LKTrackFeature(frame);
  }

  size_t Process(const Frame::ConstPtr &frame_last,
                 const Frame::Ptr &frame_curr) {
    return LKTrackFeature(frame_last, frame_curr);
  }

 protected:
  void DetectFeatures(const Frame::Ptr &frame) {
    cv::Mat mask(frame->img().size(), CV_8UC1, 255);
    for (const auto &feat : frame->features()) {
      cv::rectangle(mask, feat->pt - cv::Point2d(10, 10),
                    feat->pt + cv::Point2d(10, 10), 0, CV_FILLED);
    }
    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(frame->img(), keypoints, mask);
    for (auto &kp : keypoints) {
      frame->features().emplace_back(new Feature(kp.pt, frame));
    }
    AINFO << "Frame " << frame->id() << ": " << keypoints.size()
          << " features detected.";
  }

  // use LK flow to track feature points in the right image
  size_t LKTrackFeature(const StereoFrame::Ptr &frame) {
    std::vector<cv::Point2f> kps_lft, kps_rgt;
    StereoCamera::ConstPtr cam = frame->stereo_camera();
    // initial guess
    for (auto &feat : frame->features()) {
      kps_lft.push_back(feat->pt);
      auto mp = feat->map_point.lock();
      if (mp) {
        double disp = cam->Disparity(mp->position());
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
      if (status[i] && std::abs(kps_rgt[i].y - kps_lft[i].y) <= 2.0) {
        Feature::Ptr feat(new Feature(kps_rgt[i], frame));
        feat->is_on_left_img = false;
        frame->features_rgt().push_back(feat);
        ++num_tracked;
      } else {
        frame->features_rgt().push_back(nullptr);
      }
    }
    AINFO << "Frame " << frame->id() << ": " << num_tracked
          << " features tracked in right image.";
    return num_tracked;
  }

  // use LK flow to track feature points in the curr image
  size_t LKTrackFeature(const Frame::ConstPtr &frame_last,
                        const Frame::Ptr &frame_curr) {
    std::vector<cv::Point2f> kps_last, kps_curr;
    // initial guess
    const auto &pose_w2c = frame_curr->pose_w2c();
    for (auto &feat : frame_last->features()) {
      kps_last.push_back(feat->pt);
      if (feat->map_point.lock()) {
        Eigen::Vector2d px = frame_curr->camera()->Project(
            feat->map_point.lock()->position(), pose_w2c);
        kps_curr.emplace_back(px.x(), px.y());
      } else {
        kps_curr.push_back(feat->pt);
      }
    }
    // KLT
    std::vector<uchar> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(
        frame_last->img(), frame_curr->img(), kps_last, kps_curr, status, error,
        cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);
    // postprocess
    size_t num_tracked = 0;
    for (size_t i = 0; i < status.size(); ++i) {
      if (status[i]) {
        Feature::Ptr feat(new Feature(kps_curr[i], frame_curr));
        feat->is_on_left_img = false;
        feat->map_point = frame_last->features()[i]->map_point;
        frame_curr->features().push_back(feat);
        ++num_tracked;
      }
    }
    AINFO << "Frame " << frame_curr->id() << ": " << num_tracked
          << " features tracked from last frame.";
    return num_tracked;
  }

  //   YAML::Node config_;
  cv::Ptr<cv::GFTTDetector> gftt_;
};

}  // namespace oh_my_vslam