#include "oh_my_vslam/backend/solver.h"

namespace oh_my_vslam {
namespace backend {

namespace {
double kHuberLossScale = 0.1;
}

double ReprojCostFunction::Cost(const common::Pose3d &pose,
                                const Feature &feature) {
  Eigen::Vector3d mp = feature.map_point.lock()->position();
  Eigen::Vector2d kp = {feature.pt.x, feature.pt.y};
  auto frame = feature.frame.lock();
  Eigen::Vector2d pr = frame->camera()->Project(mp, frame->pose_c2w());
  return (pr - kp).norm();
}

Solver::Solver() { loss_function_ = new ceres::HuberLoss(kHuberLossScale); }

void Solver::AddPoseVertex(size_t pose_id, const common::Pose3d &pose) {
  poses_.insert({pose_id, APose(pose.Inv())});
  poses_added_.insert({pose_id, false});
}

void Solver::AddPointVertex(size_t point_id, const Eigen::Vector3d &point) {
  points_.insert({point_id, point});
  points_added_.insert({point_id, false});
}

void Solver::AddEdge(const Feature &feature) {
  auto fm = feature.frame.lock();
  auto mp = feature.map_point.lock();
  double *point = points_.at(mp->id()).data();
  double *r_quat = nullptr;
  double *t_vec = nullptr;
  if (poses_.count(fm->keyframe_id())) {
    auto &pose = poses_.at(fm->keyframe_id());
    r_quat = pose.r_quat.coeffs().data();
    t_vec = pose.t_vec.data();
    if (!poses_added_.at(fm->keyframe_id())) {
      problem_.AddParameterBlock(r_quat, 4,
                                 new ceres::EigenQuaternionParameterization());
      problem_.AddParameterBlock(t_vec, 3);
      poses_added_.at(fm->keyframe_id()) = true;
    }
  }
  if (!points_added_.at(mp->id())) {
    problem_.AddParameterBlock(point, 3);
    points_added_.at(mp->id()) = true;
  }
  if (r_quat && t_vec) {
    ceres::CostFunction *cost_function =
        ReprojCostFunction::Create(feature, true);
    problem_.AddResidualBlock(cost_function, loss_function_, r_quat, t_vec,
                              point);
  } else {
    ceres::CostFunction *cost_function =
        ReprojCostFunction::Create(feature, false);
    problem_.AddResidualBlock(cost_function, loss_function_, point);
  }
}

bool Solver::Solve(int max_iter_num, bool verbose) {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = max_iter_num;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);
  AINFO_IF(verbose) << summary.BriefReport();
  return summary.termination_type == ceres::CONVERGENCE;
}

bool Solver::GetPose(size_t pose_id, common::Pose3d *pose) const {
  if (!poses_.count(pose_id)) return false;
  *pose = poses_.at(pose_id).ToPose3d().Inv();
  return true;
}

bool Solver::GetPoint(size_t point_id, Eigen::Vector3d *point) const {
  if (!points_.count(point_id)) return false;
  *point = points_.at(point_id);
  return true;
}

}  // namespace backend
}  // namespace oh_my_vslam