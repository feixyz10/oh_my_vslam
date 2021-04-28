#include "oh_my_vslam/backend/solver.h"

namespace oh_my_vslam {

namespace {
double kHuberLossScale = 0.1;
}

double ReprojectionCostFunction::Cost(const common::Pose3d &pose,
                                      const Feature &feature) {
  Eigen::Vector3d mp = feature.map_point.lock()->position();
  Eigen::Vector2d kp = {feature.pt.x, feature.pt.y};
  auto frame = feature.frame.lock();
  Eigen::Vector2d pr = frame->camera()->Project(mp, frame->pose_c2w());
  return (pr - kp).norm();
}

void Solver::AddPoseVertex(size_t pose_id, const common::Pose3d &pose) {
  poses_.insert({pose_id, pose});
}

void Solver::AddPointVertex(size_t point_id, const Eigen::Vector3d &point) {
  points_.insert({point_id, point});
}

void Solver::AddEdge(const Feature &feature) {
  ceres::CostFunction *cost_function =
      ReprojectionCostFunction::Create(feature);
  problem_.AddResidualBlock(cost_function, loss_function_, r_quat_, t_vec_);
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

const common::Pose3d &Solver::GetPose(size_t pose_id) const {
  return poses_.at(pose_id);
}

const Eigen::Vector3d &Solver::GetPoint(size_t point_id) const {
  return points_.at(point_id);
}

}  // namespace oh_my_vslam