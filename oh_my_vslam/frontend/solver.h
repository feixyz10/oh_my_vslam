#pragma once

#include <ceres/ceres.h>

#include "common/common.h"
#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/feature.h"

namespace oh_my_vslam {
namespace frontend {

class ReprojCostFunction {
 public:
  explicit ReprojCostFunction(const Feature &feature) : feature_(feature) {}

  template <typename T>
  bool operator()(const T *const r_quat, const T *const t_vec,
                  T *residual) const;

  static double Cost(const common::Pose3d &pose, const Feature &feature);

  static ceres::CostFunction *Create(const Feature &feature) {
    return new ceres::AutoDiffCostFunction<ReprojCostFunction, 2, 4, 3>(
        new ReprojCostFunction(feature));
  }

 private:
  Feature feature_;
  DISALLOW_COPY_AND_ASSIGN(ReprojCostFunction);
};

class Solver {
 public:
  explicit Solver(const common::Pose3d &pose);

  void AddEdge(const Feature &feature);

  bool Solve(int max_iter_num = 5, bool verbose = false,
             common::Pose3d *pose = nullptr);

  common::Pose3d GetPose() const;

 private:
  ceres::Problem problem_;

  ceres::LossFunction *loss_function_;

  // r_quat_: [x, y, z, w], t_vec_: [x, y, z]
  double r_quat_[4], t_vec_[3];

  DISALLOW_COPY_AND_ASSIGN(Solver);
};

template <typename T>
bool ReprojCostFunction::operator()(const T *const r_quat, const T *const t_vec,
                                    T *residual) const {
  Eigen::Matrix<T, 3, 1> mp = feature_.map_point.lock()->position().cast<T>();
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

}  // namespace frontend
}  // namespace oh_my_vslam