#include "oh_my_vslam/core/frame.h"

namespace oh_my_vslam {

Frame::Frame(const size_t id, const double timestamp, const cv::Mat &img,
             const Camera::ConstPtr &camera, const common::Pose3d &pose_c2w)
    : id_(id),
      timestamp_(timestamp),
      img_(img),
      camera_(camera),
      pose_c2w_(pose_c2w),
      pose_w2c_(pose_c2w.Inv()) {}

bool Frame::IsInFrame(const Eigen::Vector3d &pt_w) const {
  Eigen::Vector3d pt_c = pose_w2c_ * pt_w;
  if (pt_c.z() <= 0.0) return false;
  Eigen::Vector2d pix = camera_->Project(pt_c);
  double u = pix.x(), v = pix.y();
  double w = img_.cols, h = img_.rows;
  return u >= 0 && u < w && v >= 0 && v < h;
}

}  // namespace oh_my_vslam