#pragma once

#include <ceres/ceres.h>

#include "common/common.h"
#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/feature.h"

namespace oh_my_vslam {
namespace backend {

class ReprojCostFunction {
 public:
  explicit ReprojCostFunction(const Feature &feature) : feature_(feature) {}

  template <typename T>
  bool operator()(const T *const r_quat, const T *const t_vec,
                  const T *const point, T *residual) const;

  template <typename T>
  bool operator()(const T *const point, T *residual) const;

  static double Cost(const common::Pose3d &pose, const Feature &feature);

  static ceres::CostFunction *Create(const Feature &feature,
                                     bool is_opt_pose = true) {
    if (is_opt_pose) {
      return new ceres::AutoDiffCostFunction<ReprojCostFunction, 2, 4, 3, 3>(
          new ReprojCostFunction(feature));
    } else {
      return new ceres::AutoDiffCostFunction<ReprojCostFunction, 2, 3>(
          new ReprojCostFunction(feature));
    }
  }

 private:
  template <typename T>
  void Project(const T *const r_quat, const T *const t_vec,
               const T *const point, T *proj, bool left) const;

  Feature feature_;
  DISALLOW_COPY_AND_ASSIGN(ReprojCostFunction);
};

class Solver {
 public:
  Solver();

  void AddPoseVertex(size_t pose_id, const common::Pose3d &pose);

  void AddPointVertex(size_t point_id, const Eigen::Vector3d &point);

  void AddEdge(const Feature &feature);

  bool Solve(int max_iter_num = 5, bool verbose = false);

  bool GetPose(size_t pose_id, common::Pose3d *pose) const;

  bool GetPoint(size_t point_id, Eigen::Vector3d *point) const;

 private:
  ceres::Problem problem_;

  ceres::LossFunction *loss_function_;

  struct APose {
    Eigen::Quaterniond r_quat;
    Eigen::Vector3d t_vec;
    explicit APose(const common::Pose3d &pose) {
      r_quat = pose.r_quat();
      t_vec = pose.t_vec();
    }
    const common::Pose3d ToPose3d() const { return {r_quat, t_vec}; }
  };

  std::unordered_map<size_t, APose> poses_;
  std::unordered_map<size_t, Eigen::Vector3d> points_;
  std::unordered_map<size_t, bool> poses_added_;
  std::unordered_map<size_t, bool> points_added_;

  DISALLOW_COPY_AND_ASSIGN(Solver);
};

template <typename T>
bool ReprojCostFunction::operator()(const T *const r_quat, const T *const t_vec,
                                    const T *const point, T *residual) const {
  Eigen::Matrix<T, 2, 1> kp = {T(feature_.pt.x), T(feature_.pt.y)};
  T proj[2] = {T(0.0), T(0.0)};
  Project<T>(r_quat, t_vec, point, proj, feature_.is_on_left_img);
  residual[0] = kp[0] - proj[0];
  residual[1] = kp[1] - proj[1];
  return true;
}

template <typename T>
bool ReprojCostFunction::operator()(const T *const point, T *residual) const {
  auto pose = feature_.frame.lock()->pose_w2c();
  Eigen::Quaternion<T> r_quat = pose.r_quat().template cast<T>();
  Eigen::Matrix<T, 3, 1> t_vec = pose.t_vec().template cast<T>();
  Eigen::Matrix<T, 2, 1> kp = {T(feature_.pt.x), T(feature_.pt.y)};
  T *quat = r_quat.coeffs().data();
  T *vec = t_vec.data();
  T proj[2] = {T(0.0), T(0.0)};
  Project(quat, vec, point, proj, feature_.is_on_left_img);
  residual[0] = kp[0] - proj[0];
  residual[1] = kp[1] - proj[1];
  return true;
}

template <typename T>
void ReprojCostFunction::Project(const T *const r_quat, const T *const t_vec,
                                 const T *const point, T *proj,
                                 bool left) const {
  Eigen::Quaternion<T> r = {r_quat[3], r_quat[0], r_quat[1], r_quat[2]};
  Eigen::Matrix<T, 3, 1> t = {t_vec[0], t_vec[1], t_vec[2]};
  Eigen::Matrix<T, 3, 1> mp{point[0], point[1], point[2]};
  auto frame = feature_.frame.lock();
  Eigen::Matrix<T, 3, 3> intrinsic =
      frame->camera()->intrinsic_mat().template cast<T>();
  Eigen::Matrix<T, 3, 1> p = intrinsic * (r * mp + t);
  proj[0] = p[0] / p[2];
  proj[1] = p[1] / p[2];
  if (!left) {
    double baseline =
        std::static_pointer_cast<const StereoCamera>(frame->camera())
            ->baseline();
    proj[0] -= T(baseline);
  }
}

}  // namespace backend
}  // namespace oh_my_vslam