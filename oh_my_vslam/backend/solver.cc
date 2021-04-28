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
  poses_.insert({pose_id, APose(pose)});
  poses_added_.insert({pose_id, false});
}

void Solver::AddPointVertex(size_t point_id, const Eigen::Vector3d &point) {
  points_.insert({point_id, point});
  poses_added_.insert({point_id, false});
}

void Solver::AddEdge(const Feature &feature) {
  auto fm = feature.frame.lock();
  auto mp = feature.map_point.lock();
  auto &pose = poses_.at(fm->id());
  double *r_quat = pose.r_quat.coeffs().data();
  double *t_vec = pose.t_vec.data();
  double *point = points_.at(mp->id()).data();
  if (!poses_added_.at(fm->id())) {
    problem_.AddParameterBlock(r_quat, 4,
                               new ceres::EigenQuaternionParameterization());
    problem_.AddParameterBlock(t_vec, 3);
    poses_added_.at(fm->id()) = true;
  }
  if (!points_added_.at(fm->id())) {
    problem_.AddParameterBlock(point, 3);
    points_added_.at(fm->id()) = true;
  }
  ceres::CostFunction *cost_function = ReprojCostFunction::Create(feature);
  problem_.AddResidualBlock(cost_function, loss_function_, r_quat, t_vec,
                            point);
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

common::Pose3d Solver::GetPose(size_t pose_id) const {
  return poses_.at(pose_id).ToPose3d();
}

Eigen::Vector3d Solver::GetPoint(size_t point_id) const {
  return points_.at(point_id);
}

}  // namespace backend
}  // namespace oh_my_vslam