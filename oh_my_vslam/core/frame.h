#pragma once

#include <memory>
#include <opencv2/core/core.hpp>

#include "common/common.h"
#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/camera.h"

namespace oh_my_vslam {

struct Feature;

class Frame {
 public:
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

 public:
  Frame(const double timestamp, const cv::Mat &img, const Camera::Ptr &camera,
        const common::Pose3d &pose_c2w = {})
      : timestamp_(timestamp),
        img_(img),
        camera_(camera),
        pose_c2w_(pose_c2w),
        pose_w2c_(pose_c2w.Inv()) {
    ACHECK(camera != nullptr);
    static size_t static_id = 0;
    id_ = static_id++;
  }

  size_t id() const {
    return id_;
  }

  double timestamp() const {
    return timestamp_;
  }

  const cv::Mat &img() const {
    return img_;
  }

  Camera::ConstPtr camera() const {
    return camera_;
  }

  const common::Pose3d &pose_c2w() const {
    return pose_c2w_;
  }

  const common::Pose3d &pose_w2c() const {
    return pose_w2c_;
  }

  void SetPose(const common::Pose3d &pose_c2w) {
    pose_c2w_ = pose_c2w;
    pose_w2c_ = pose_c2w.Inv();
  }

  void SetKeyFrame() {
    static size_t static_keyframe_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = static_keyframe_id++;
  }

  const std::vector<std::shared_ptr<Feature>> &features() const {
    return features_;
  }

  std::vector<std::shared_ptr<Feature>> &features() {
    return features_;
  }

 protected:
  size_t id_ = 0;
  double timestamp_ = -1.0;
  bool is_keyframe_ = false;
  size_t keyframe_id_ = 0;
  cv::Mat img_;
  Camera::Ptr camera_{nullptr};
  common::Pose3d pose_c2w_{}, pose_w2c_{};
  std::vector<std::shared_ptr<Feature>> features_;

  DISALLOW_COPY_AND_ASSIGN(Frame);
};

using MonoFrame = Frame;

class RGBDFrame : public Frame {
 public:
  using Ptr = std::shared_ptr<RGBDFrame>;
  using ConstPtr = std::shared_ptr<const RGBDFrame>;

 public:
  RGBDFrame(const double timestamp, const cv::Mat &img, const cv::Mat &depth,
            const Camera::Ptr &camera = nullptr,
            const common::Pose3d &pose_c2w = {})
      : Frame(timestamp, img, camera, pose_c2w), depth_(depth) {}

  const cv::Mat &depth() const {
    return depth_;
  }

 protected:
  cv::Mat depth_;
  DISALLOW_COPY_AND_ASSIGN(RGBDFrame);
};

class StereoFrame : public Frame {
 public:
  using Ptr = std::shared_ptr<StereoFrame>;
  using ConstPtr = std::shared_ptr<const StereoFrame>;

 public:
  StereoFrame(const double timestamp, const cv::Mat &img_lft,
              const cv::Mat &img_rgt, const StereoCamera::Ptr &camera = nullptr,
              const common::Pose3d &pose_c2w = {})
      : Frame(timestamp, img_lft, camera, pose_c2w), img_rgt_(img_rgt) {}

  const cv::Mat &img_rgt() const {
    return img_rgt_;
  }

  const std::vector<std::shared_ptr<Feature>> &features_rgt() const {
    return features_rgt_;
  }

  std::vector<std::shared_ptr<Feature>> &features_rgt() {
    return features_rgt_;
  }

  StereoCamera::ConstPtr stereo_camera() const {
    return std::static_pointer_cast<StereoCamera>(camera_);
  }

 protected:
  cv::Mat img_rgt_;  // right image
  std::vector<std::shared_ptr<Feature>> features_rgt_;
  DISALLOW_COPY_AND_ASSIGN(StereoFrame);
};

}  // namespace oh_my_vslam