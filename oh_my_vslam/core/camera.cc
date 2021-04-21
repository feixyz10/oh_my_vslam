#include "oh_my_vslam/core/camera.h"

namespace oh_my_vslam {

namespace {
const double kEps = 1e-9;
const size_t kUndistortIterNum = 16;
}  // namespace

Camera::Camera(double fx, double fy, double cx, double cy, double k1, double k2,
               double k3, double p1, double p2)
    : fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy),
      k1_(k1),
      k2_(k2),
      k3_(k3),
      p1_(p1),
      p2_(p2) {
  is_zero_distortion_ = std::abs(k1_) < kEps && std::abs(k2_) < kEps &&
                        std::abs(k3_) < kEps && std::abs(p1_) < kEps &&
                        std::abs(p2_) < kEps;
};

Camera::Camera(const Eigen::Vector4d &intrinsic_params,
               const Eigen::Matrix<double, 5, 1> &distortion_params)
    : Camera(intrinsic_params(0), intrinsic_params(1), intrinsic_params(2),
             intrinsic_params(3), distortion_params(0), distortion_params(1),
             distortion_params(2), distortion_params(3), distortion_params(4)) {
}

void Camera::SetZeroDistortion() {
  k1_ = k2_ = k3_ = p1_ = p2_ = 0.0;
  is_zero_distortion_ = true;
}

Eigen::Vector2d Camera::Project(const Eigen::Vector3d &pt_w,
                                const common::Pose3d &T_w2c) const {
  Eigen::Vector3d pt_c = T_w2c * pt_w;
  Eigen::Vector2d pt = Distort(Normalize(pt_c));
  return {fx_ * pt.x() + cx_, fy_ * pt.y() + cy_};
}

Eigen::Vector3d Camera::InverseProject(const Eigen::Vector2d &pix,
                                       const double depth,
                                       const common::Pose3d &T_c2w) const {
  Eigen::Vector2d pt = Undistort(Normalize(pix));
  Eigen::Vector3d pt_c = depth * pt.homogeneous();
  return T_c2w * pt_c;
};

Eigen::Vector4d Camera::intrinsic_params() const {
  Eigen::Vector4d params;
  params << fx_, fy_, cx_, cy_;
  return params;
}

Eigen::Matrix3d Camera::intrinsic_mat() const {
  Eigen::Matrix3d mat;
  mat << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
  return mat;
}

Eigen::Matrix<double, 5, 1> Camera::distortion_params() const {
  Eigen::Matrix<double, 5, 1> params;
  params << k1_, k2_, k3_, p1_, p2_;
  return params;
}

std::string Camera::ToString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "[Camera intrinsic: " << fx_ << " " << fy_ << " " << cx_ << " " << cy_
      << " distortion: ";
  if (is_zero_distortion_) {
    oss << "none";
  } else {
    oss << std::setprecision(5) << k1_ << " " << k2_ << " " << k3_ << " " << p1_
        << " " << p2_ << "]";
  }
  return oss.str();
}

Eigen::Vector2d Camera::Normalize(const Eigen::Vector3d &pt) const {
  double x = pt.x(), y = pt.y(), z = pt.z();
  return {x / z, y / z};
}

Eigen::Vector2d Camera::Normalize(const Eigen::Vector2d &pix) const {
  double u = pix.x(), v = pix.y();
  return {(u - cx_) / fx_, (v - cy_) / fy_};
}

Eigen::Vector2d Camera::Distort(const Eigen::Vector2d &pt) const {
  if (is_zero_distortion_) return pt;
  double x = pt.x(), y = pt.y();
  double x2 = x * x, xy = x * y, y2 = y * y;
  double r2 = x2 + y2;
  double r4 = r2 * r2;
  double r6 = r4 * r2;
  double k = 1 + k1_ * r2 + k2_ * r4 + k3_ * r6;
  double x_distorted = k * x + 2 * p1_ * xy + p2_ * (r2 + 2 * x2);
  double y_distorted = k * y + 2 * p2_ * xy + p1_ * (r2 + 2 * y2);
  return {x_distorted, y_distorted};
}

Eigen::Vector2d Camera::Undistort(const Eigen::Vector2d &pt) const {
  if (is_zero_distortion_) return pt;
  double x_distorted = pt.x(), y_distorted = pt.y();
  double x = x_distorted, y = y_distorted;
  for (size_t i = 0; i < kUndistortIterNum; ++i) {
    double x2 = x * x, xy = x * y, y2 = y * y;
    double r2 = x2 + y2;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double k_r = 1.0 / (1 + k1_ * r2 + k2_ * r4 + k3_ * r6);
    x = (x_distorted - 2 * p1_ * xy - p2_ * (r2 + 2 * x2)) * k_r;
    y = (y_distorted - 2 * p2_ * xy - p1_ * (r2 + 2 * y2)) * k_r;
  }
  return {x, y};
}

}  // namespace oh_my_vslam