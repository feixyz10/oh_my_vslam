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

  static double Cost(const common::Pose3d &pose, const Feature &feature);

  static ceres::CostFunction *Create(const Feature &feature) {
    return new ceres::AutoDiffCostFunction<ReprojCostFunction, 2, 4, 3, 3>(
        new ReprojCostFunction(feature));
  }

 private:
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

  common::Pose3d GetPose(size_t pose_id) const;

  Eigen::Vector3d GetPoint(size_t point_id) const;

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
  Eigen::Matrix<T, 3, 1> mp{point[0], point[1], point[2]};
  Eigen::Matrix<T, 2, 1> kp = {T(feature_.pt.x), T(feature_.pt.y)};
  Eigen::Matrix<T, 3, 3> intrinsic =
      feature_.frame.lock()->camera()->intrinsic_mat().cast<T>();
  Eigen::Quaternion<T> r(r_quat[3], r_quat[0], r_quat[1], r_quat[2]);
  Eigen::Matrix<T, 3, 1> t(t_vec[0], t_vec[1], t_vec[2]);
  Eigen::Matrix<T, 3, 1> p = intrinsic * (r * mp + t);
  residual[0] = kp[0] - p[0] / p[2];
  residual[1] = kp[1] - p[1] / p[2];
  return true;
}

}  // namespace backend
}  // namespace oh_my_vslam