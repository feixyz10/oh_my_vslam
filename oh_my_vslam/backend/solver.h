#pragma once

#include <ceres/ceres.h>

#include "common/common.h"
#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/feature.h"

namespace oh_my_vslam {

class ReprojectionCostFunction {
 public:
  explicit ReprojectionCostFunction(const Feature &feature)
      : feature_(feature) {}

  template <typename T>
  bool operator()(const T *const r_quat, const T *const t_vec,
                  T *residual) const;

  static double Cost(const common::Pose3d &pose, const Feature &feature);

  static ceres::CostFunction *Create(const Feature &feature) {
    return new ceres::AutoDiffCostFunction<ReprojectionCostFunction, 2, 4, 3>(
        new ReprojectionCostFunction(feature));
  }

 private:
  Feature feature_;
  DISALLOW_COPY_AND_ASSIGN(ReprojectionCostFunction);
};

class Solver {
 public:
  Solver() = default;

  void AddPoseVertex(size_t pose_id, const common::Pose3d &pose);

  void AddPointVertex(size_t point_id, const Eigen::Vector3d &point);

  void AddEdge(const Feature &feature);

  bool Solve(int max_iter_num = 5, bool verbose = false);

  const common::Pose3d &GetPose(size_t pose_id) const;

  const Eigen::Vector3d &GetPoint(size_t point_id) const;

 private:
  ceres::Problem problem_;

  ceres::LossFunction *loss_function_;

  std::unordered_map<size_t, common::Pose3d> poses_;
  std::unordered_map<size_t, Eigen::Vector3d> points_;
  double r_quat_[4], t_vec_[3];

  DISALLOW_COPY_AND_ASSIGN(Solver)
};

template <typename T>
bool ReprojectionCostFunction::operator()(const T *const r_quat,
                                          const T *const t_vec,
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

}  // namespace oh_my_vslam