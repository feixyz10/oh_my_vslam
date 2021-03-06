#include "oh_my_vslam/frontend/vo.h"

#include "oh_my_vslam/frontend/solver.h"

namespace oh_my_vslam {

size_t VO::PnP(const Frame::Ptr &frame, size_t max_iter_num,
               double outlier_std) {
  std::vector<size_t> outlier_indices;
  for (size_t i = 0; i < frame->features().size(); ++i) {
    if (!frame->features()[i]->map_point.lock()) outlier_indices.push_back(i);
  }
  for (size_t iteration = 0; iteration < max_iter_num; ++iteration) {
    if (outlier_indices.size() == frame->features().size()) {
      AERROR << "Frontend PnP: no map point";
      return 0;
    }
    frontend::Solver solver(frame->pose_c2w());
    size_t j = 0;
    for (size_t i = 0; i < frame->features().size(); ++i) {
      if (outlier_indices[j] == i) {
        ++j;
        continue;
      }
      solver.AddEdge(*frame->features()[i]);
    }
    common::Pose3d pose_new;
    bool converge = solver.Solve(5, false, &pose_new);
    if (!converge) {
      AWARN_IF(verbose_) << "Frontend: PnP iteration " << iteration
                         << ": no convergence";
    }
    frame->SetPose(pose_new);
    FindOutliers(frame, outlier_std, &outlier_indices);
  }
  for (auto &i : outlier_indices) {
    frame->features()[i]->map_point.reset();
  }
  size_t num_inlier = frame->features().size() - outlier_indices.size();
  AINFO_IF(verbose_) << "Frontend: PnP outliers/inliers number: "
                     << outlier_indices.size() << "/" << num_inlier;
  return num_inlier;
}

size_t VO::Triangulate(const StereoFrame::Ptr &frame) {
  StereoCamera::ConstPtr cam = frame->stereo_camera();
  int num_mp = 0;
  for (size_t i = 0; i < frame->features().size(); ++i) {
    if (!frame->features_rgt()[i]) continue;
    if (frame->features()[i]->map_point.lock()) {
      frame->features_rgt()[i]->map_point =
          frame->features()[i]->map_point.lock();
      continue;
    }
    auto &p1 = frame->features()[i]->pt;
    auto &p2 = frame->features_rgt()[i]->pt;
    MapPoint::Ptr map_point(new MapPoint);
    Eigen::Vector3d pt =
        cam->Triangulation({p1.x, p1.y}, {p2.x, p2.y}, frame->pose_c2w());
    map_point->SetPosition(pt);
    map_point->AddFeature(frame->features()[i]);
    map_point->AddFeature(frame->features_rgt()[i]);
    frame->features()[i]->map_point = map_point;
    frame->features_rgt()[i]->map_point = map_point;
    Map::Instance()->InsertMapPoint(map_point);
    ++num_mp;
  }
  return num_mp;
}

void VO::FindOutliers(const Frame::ConstPtr &frame, double outlier_std,
                      std::vector<size_t> *outlier_indices) const {
  outlier_indices->clear();
  std::vector<double> dists, dists_sq;
  double dist_sum = 0.0, dist_sq_sum = 0.0;
  int num = 0;
  for (size_t i = 0; i < frame->features().size(); ++i) {
    const auto &feat = frame->features()[i];
    if (!feat->map_point.lock()) {
      dists.push_back(std::numeric_limits<double>::max());
      dists_sq.push_back(std::numeric_limits<double>::max());
      continue;
    }
    double cost = frontend::ReprojCostFunction::Cost(frame->pose_w2c(), *feat);
    double cost_sq = cost * cost;
    dists.push_back(cost);
    dists_sq.push_back(cost_sq);
    dist_sum += cost;
    dist_sq_sum += cost_sq;
    ++num;
  }
  if (num == 0) num = 1;
  double mu = dist_sum / num, avg_dist_sq = dist_sq_sum / num;
  double sigma_sq = avg_dist_sq - mu * mu;
  for (size_t i = 0; i < dists.size(); ++i) {
    double d = dists[i] - mu;
    if (d * d > outlier_std * outlier_std * sigma_sq) {
      outlier_indices->push_back(i);
    }
  }
}

}  // namespace oh_my_vslam