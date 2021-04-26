#pragma once

#include <memory>

#include "common/geometry/pose3d.h"

namespace oh_my_vslam {

class Camera {
 public:
  using Ptr = std::shared_ptr<Camera>;
  using ConstPtr = std::shared_ptr<const Camera>;

 public:
  Camera(double fx, double fy, double cx, double cy, double k1 = 0.0,
         double k2 = 0.0, double k3 = 0.0, double p1 = 0.0, double p2 = 0.0);

  Camera(const Eigen::Vector4d &intrinsic_params,
         const Eigen::Matrix<double, 5, 1> &distortion_params =
             Eigen::Matrix<double, 5, 1>::Zero());

  ~Camera() = default;

  // Set all distortion parameters to zero
  void SetZeroDistortion();

  // Project a point in world coordinate to the image plane
  Eigen::Vector2d Project(const Eigen::Vector3d &pt_w,
                          const common::Pose3d &pose_w2c = {}) const;

  // Inverse project a pixel to world coordinate, here the `depth`
  // is measured in camera coordinate
  Eigen::Vector3d InverseProject(const Eigen::Vector2d &pix,
                                 const double depth = 1.0,
                                 const common::Pose3d &pose_c2w = {}) const;

  Eigen::Vector4d intrinsic_params() const;

  Eigen::Matrix3d intrinsic_mat() const;

  Eigen::Matrix<double, 5, 1> distortion_params() const;

  bool zero_distortion() const {
    return is_zero_distortion_;
  }

  std::string ToString() const;

 protected:
  // Project a point to the normalized image plane (i.e. depth = 1.0)
  Eigen::Vector2d Normalize(const Eigen::Vector3d &pt) const;

  // Inverse project a pixel to the normalized image plane
  Eigen::Vector2d Normalize(const Eigen::Vector2d &pix) const;

  // Add distortion for a point on normalized image plane
  Eigen::Vector2d Distort(const Eigen::Vector2d &pt) const;

  // Remove distortion for a point on normalized image plane
  Eigen::Vector2d Undistort(const Eigen::Vector2d &pt) const;

  // intrinsic parameters
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
  // distortion parameters
  double k1_{0.0}, k2_{0.0}, k3_{0.0}, p1_{0.0}, p2_{0.0};
  bool is_zero_distortion_ = true;
};

class StereoCamera : public Camera {
 public:
  using Ptr = std::shared_ptr<StereoCamera>;
  using ConstPtr = std::shared_ptr<const StereoCamera>;

 public:
  // All distortion parameters must be zero
  StereoCamera(double fx, double fy, double cx, double cy, double baseline)
      : Camera(fx, fy, cx, cy), baseline_(baseline) {}

  StereoCamera(const Eigen::Vector4d &intrinsic_params, double baseline)
      : Camera(intrinsic_params), baseline_(baseline) {}

  double baseline() const {
    return baseline_;
  }

  Eigen::Vector2d ProjectToLeft(const Eigen::Vector3d &pt_w,
                                const common::Pose3d &pose_w2c = {}) const {
    return Project(pt_w, pose_w2c);
  }

  Eigen::Vector2d ProjectToRight(const Eigen::Vector3d &pt_w,
                                 const common::Pose3d &pose_w2c = {}) const {
    return Project(pt_w, pose_w2c) -
           Eigen::Vector2d(Disparity(pt_w, pose_w2c), 0);
  }

  double Disparity(const Eigen::Vector3d &pt_w,
                   const common::Pose3d &pose_w2c = {}) const {
    Eigen::Vector3d pt_c = pose_w2c * pt_w;
    return baseline_ * fx_ / pt_c.z();
  }

  Eigen::Vector3d Triangulation(const Eigen::Vector2d &pt_lft,
                                const Eigen::Vector2d &pt_rgt,
                                const common::Pose3d &pose_c2w = {}) const {
    double z = baseline_ * fx_ / (std::abs(pt_lft.x() - pt_rgt.x()) + 1e-9);
    return InverseProject(pt_lft, z, pose_c2w);
  }

 protected:
  double baseline_;
};

}  // namespace oh_my_vslam
