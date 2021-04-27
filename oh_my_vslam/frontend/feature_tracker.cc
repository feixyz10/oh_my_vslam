#include "oh_my_vslam/frontend/feature_tracker.h"

namespace oh_my_vslam {

size_t FeatureTracker::Track(const StereoFrame::Ptr &frame) {
  DetectFeature(frame);
  std::vector<cv::Point2f> kps_lft, kps_rgt;
  StereoCamera::ConstPtr cam = frame->stereo_camera();
  // initial guess
  for (auto &feat : frame->features()) {
    kps_lft.push_back(feat->pt);
    auto mp = feat->map_point.lock();
    if (mp) {
      float disp = static_cast<float>(cam->Disparity(mp->position()));
      kps_rgt.emplace_back(feat->pt.x - disp, feat->pt.y);
    } else {
      kps_rgt.push_back(feat->pt);
    }
  }
  // LK
  std::vector<uchar> status;
  LKOpticalFlow(frame->img(), frame->img_rgt(), kps_lft, &kps_rgt, &status);
  // postprocess
  size_t num_tracked = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] && std::abs(kps_rgt[i].y - kps_lft[i].y) <= 2.0f) {
      Feature::Ptr feat(new Feature(kps_rgt[i], frame));
      feat->is_on_left_img = false;
      frame->features_rgt().push_back(feat);
      ++num_tracked;
    } else {
      frame->features_rgt().push_back(nullptr);
    }
  }
  AINFO_IF(verbose_) << "Frame " << frame->id() << ": " << num_tracked
                     << " features tracked on right image from left image.";
  return num_tracked;
}

size_t FeatureTracker::Track(const Frame::ConstPtr &frame_last,
                             const Frame::Ptr &frame_curr) {
  std::vector<cv::Point2f> kps_last, kps_curr;
  // initial guess
  const auto &pose_w2c = frame_curr->pose_w2c();
  for (auto &feat : frame_last->features()) {
    kps_last.push_back(feat->pt);
    if (feat->map_point.lock()) {
      Eigen::Vector2d px = frame_curr->camera()->Project(
          feat->map_point.lock()->position(), pose_w2c);
      kps_curr.emplace_back(static_cast<float>(px.x()),
                            static_cast<float>(px.y()));
    } else {
      kps_curr.push_back(feat->pt);
    }
  }
  // LK
  std::vector<uchar> status;
  LKOpticalFlow(frame_last->img(), frame_curr->img(), kps_last, &kps_curr,
                &status);
  // post process
  size_t num_tracked = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      Feature::Ptr feat(new Feature(kps_curr[i], frame_curr));
      feat->map_point = frame_last->features()[i]->map_point;
      frame_curr->features().push_back(feat);
      ++num_tracked;
    }
  }
  AINFO << "Frame " << frame_curr->id() << ": " << num_tracked
        << " features tracked on current frame from last image.";
  return num_tracked;
}

void FeatureTracker::DetectFeature(const Frame::Ptr &frame) {
  cv::Mat mask(frame->img().size(), CV_8UC1, 255);
  for (const auto &feat : frame->features()) {
    cv::rectangle(mask, feat->pt - cv::Point2f(10, 10),
                  feat->pt + cv::Point2f(10, 10), 0, CV_FILLED);
  }
  bool empty = frame->features().empty();
  std::vector<cv::KeyPoint> keypoints;
  gftt_->detect(frame->img(), keypoints, mask);
  for (auto &kp : keypoints) {
    frame->features().emplace_back(new Feature(kp.pt, frame));
  }
  AINFO_IF(verbose_) << "Frame " << frame->id() << ": " << keypoints.size()
                     << " features detected" << (empty ? " (new)" : "");
}

void FeatureTracker::LKOpticalFlow(const cv::Mat &img1, const cv::Mat &img2,
                                   const std::vector<cv::Point2f> &kps1,
                                   std::vector<cv::Point2f> *kps2,
                                   std::vector<uchar> *status) {
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(
      img1, img2, kps1, *kps2, *status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);
}

}  // namespace oh_my_vslam