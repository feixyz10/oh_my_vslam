#pragma once

#include <memory>
#include <opencv2/core/core.hpp>

#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/camera.h"

namespace oh_my_vslam {

class Frame {
 public:
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

 public:
  Frame() = default;

  Frame(const size_t id, const double timestamp, const cv::Mat &img = {},
        const Camera::ConstPtr &camera = nullptr,
        const common::Pose3d &pose_c2w = {});

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

 protected:
  size_t id_ = 0;
  double timestamp_ = -1.0;
  cv::Mat img_;
  Camera::ConstPtr camera_{nullptr};
  common::Pose3d pose_c2w_{}, pose_w2c_{};
};

}  // namespace oh_my_vslam