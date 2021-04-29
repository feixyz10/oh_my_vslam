#include "oh_my_vslam/backend/backend.h"

#include "oh_my_vslam/core/map.h"

namespace oh_my_vslam {

void Backend::Process() {
  std::vector<StereoFrame::Ptr> key_frames;
  GetLatestKeyFrames(&key_frames);
  if (key_frames.size() < 3) return;
  backend::Solver solver;
  ConstructGraph(key_frames, &solver);
  bool convergence = solver.Solve();
  if (!convergence) AWARN << "Backend: no convergence";
  common::Pose3d pose_opt;
  for (auto &frame : key_frames) {
    std::ostringstream oss;
    if (solver.GetPose(frame->keyframe_id(), &pose_opt)) {
      frame->SetPose(pose_opt);
    }
    for (auto &feat : frame->features()) {
      auto mp = feat->map_point.lock();
      if (!mp) continue;
      Eigen::Vector3d point_opt;
      if (solver.GetPoint(mp->id(), &point_opt)) {
        mp->SetPosition(point_opt);
      }
    }
  }
  ACHECK(solver.GetPose(key_frames.back()->keyframe_id(), &pose_opt));
  pose_delta_ = pose_opt * key_frames.back()->pose_w2c();
}

void Backend::GetLatestKeyFrames(
    std::vector<StereoFrame::Ptr> *key_frames) const {
  size_t num_kfs = Map::Instance()->key_frames().size();
  size_t win_size = std::min(num_kfs, config_["ba_window_size"].as<size_t>());
  auto it = Map::Instance()->key_frames().rbegin();
  for (size_t i = 0; i < win_size; ++i) {
    key_frames->push_back(std::static_pointer_cast<StereoFrame>(it->second));
    ++it;
  }
  std::reverse(key_frames->begin(), key_frames->end());
}

void Backend::ConstructGraph(const std::vector<StereoFrame::Ptr> &key_frames,
                             backend::Solver *solver) const {
  bool first = true;
  for (auto &frame : key_frames) {
    if (first) {
      first = false;
    } else {
      solver->AddPoseVertex(frame->keyframe_id(), frame->pose_c2w());
    }
    for (auto &feat : frame->features()) {
      auto mp = feat->map_point.lock();
      if (mp) solver->AddPointVertex(mp->id(), mp->position());
    }
  }
  for (auto &frame : key_frames) {
    int i = 0, j = 0;
    for (auto &feat : frame->features()) {
      if (feat->map_point.lock()) {
        solver->AddEdge(*feat);
        ++i;
      }
    }
    for (auto &feat : frame->features_rgt()) {
      if (feat && feat->map_point.lock()) {
        solver->AddEdge(*feat);
        ++j;
      }
    }
  }
}

}  // namespace oh_my_vslam