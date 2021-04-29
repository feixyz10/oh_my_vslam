#include "oh_my_vslam/frontend/solver.h"

namespace oh_my_vslam {
namespace frontend {

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

Solver::Solver(const common::Pose3d &pose) {
  common::Pose3d pose_inv = pose.Inv();
  std::copy_n(pose_inv.r_quat().coeffs().data(), 4, r_quat_);
  std::copy_n(pose_inv.t_vec().data(), 3, t_vec_);
  loss_function_ = new ceres::HuberLoss(kHuberLossScale);
  problem_.AddParameterBlock(r_quat_, 4,
                             new ceres::EigenQuaternionParameterization());
  problem_.AddParameterBlock(t_vec_, 3);
}

bool Solver::Solve(int max_iter_num, bool verbose, common::Pose3d *const pose) {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = max_iter_num;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);
  AINFO_IF(verbose) << "Frontend: " << summary.BriefReport();
  if (pose) *pose = GetPose();
  return summary.termination_type == ceres::CONVERGENCE;
}

void Solver::AddEdge(const Feature &feature) {
  ceres::CostFunction *cost_function = ReprojCostFunction::Create(feature);
  problem_.AddResidualBlock(cost_function, loss_function_, r_quat_, t_vec_);
}

common::Pose3d Solver::GetPose() const {
  return common::Pose3d(r_quat_, t_vec_).Inv();
}

}  // namespace frontend
}  // namespace oh_my_vslam