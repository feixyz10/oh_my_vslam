#pragma once

#include <memory>
#include <opencv2/core/core.hpp>

#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/camera.h"

namespace oh_my_vslam {

enum class FrameType { UNKNOWN = 0, MONO, STEREO, RGBD };

class Frame {
 public:
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

 public:
  Frame() = default;

  Frame(const size_t id, const double timestamp, const cv::Mat &img = {},
        const Camera::ConstPtr &camera = nullptr,
        const common::Pose3d &pose_c2w = {})
      : id_(id),
        timestamp_(timestamp),
        img_(img),
        camera_(camera),
        pose_c2w_(pose_c2w),
        pose_w2c_(pose_c2w.Inv()) {}

  virtual ~Frame() = default;

  size_t id() const {
    return id_;
  }

  double timestamp() const {
    return timestamp_;
  }

  cv::Mat img() const {
    return img_;
  }

  Camera::ConstPtr camera() const {
    return camera_;
  }

  common::Pose3d pose() const {
    return pose_c2w_;
  }

  void SetPose(const common::Pose3d &pose_c2w) {
    pose_c2w_ = pose_c2w;
    pose_w2c_ = pose_c2w.Inv();
  }

  // check if a point in world coordinate is seen by the frame
  virtual bool IsInFrame(const Eigen::Vector3d &pt_w) const;

  virtual FrameType frame_type() const {
    return FrameType::UNKNOWN;
  }

 protected:
  size_t id_ = 0;
  double timestamp_ = -1.0;
  cv::Mat img_;
  Camera::ConstPtr camera_{nullptr};
  common::Pose3d pose_c2w_{}, pose_w2c_{};
};

class MonoFrame : public Frame {
 public:
  using Ptr = std::shared_ptr<MonoFrame>;
  using ConstPtr = std::shared_ptr<const MonoFrame>;

 public:
  MonoFrame() = default;

  MonoFrame(const size_t id, const double timestamp, const cv::Mat &img = {},
            const Camera::ConstPtr &camera = nullptr,
            const common::Pose3d &pose_c2w = {})
      : Frame(id, timestamp, img, camera, pose_c2w){};

  FrameType frame_type() const override {
    return FrameType::MONO;
  }
};

class RGBDFrame : public Frame {
 public:
  using Ptr = std::shared_ptr<RGBDFrame>;
  using ConstPtr = std::shared_ptr<const RGBDFrame>;

 public:
  RGBDFrame() = default;

  RGBDFrame(const size_t id, const double timestamp, const cv::Mat &img = {},
            const cv::Mat &depth = {}, const Camera::ConstPtr &camera = nullptr,
            const common::Pose3d &pose_c2w = {})
      : Frame(id, timestamp, img, camera, pose_c2w), depth_(depth) {}

  FrameType frame_type() const override {
    return FrameType::RGBD;
  }

 protected:
  cv::Mat depth_;
};

class StereoFrame : public Frame {
 public:
  using Ptr = std::shared_ptr<StereoFrame>;
  using ConstPtr = std::shared_ptr<const StereoFrame>;

 public:
  StereoFrame() = default;

  StereoFrame(const size_t id, const double timestamp, const cv::Mat &img = {},
              const cv::Mat &img2 = {},
              const Camera::ConstPtr &camera = nullptr,
              const Camera::ConstPtr &camera2 = nullptr,
              const common::Pose3d &pose_c2w = {},
              const common::Pose3d &pose_rel = {})
      : Frame(id, timestamp, img, camera, pose_c2w),
        img2_(img2),
        camera2_(camera2),
        pose_rel_(pose_rel) {}

  FrameType frame_type() const override {
    return FrameType::STEREO;
  }

  bool IsInFrame(const Eigen::Vector3d &pt_w) const override;

 protected:
  cv::Mat img2_;                       // right image
  Camera::ConstPtr camera2_{nullptr};  // right camera
  common::Pose3d pose_rel_{};  // relative pose of right camera to left camera
};

}  // namespace oh_my_vslam