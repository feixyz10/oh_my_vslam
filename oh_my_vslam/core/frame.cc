#include "oh_my_vslam/core/frame.h"

namespace oh_my_vslam {

bool Frame::IsInFrame(const Eigen::Vector3d &pt_w) const {
  Eigen::Vector3d pt_c = pose_w2c_ * pt_w;
  if (pt_c.z() <= 0.0) return false;
  Eigen::Vector2d pix = camera_->Project(pt_c);
  double u = pix.x(), v = pix.y();
  double w = img_.cols, h = img_.rows;
  return u >= 0 && u < w && v >= 0 && v < h;
}

bool StereoFrame::IsInFrame(const Eigen::Vector3d &pt_w) const {
  return Frame::IsInFrame(pt_w) && Frame::IsInFrame(pose_rel_.Inv() * pt_w);
}

}  // namespace oh_my_vslam