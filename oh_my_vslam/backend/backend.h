#pragma once

#include <memory>

#include "common/common.h"
#include "oh_my_vslam/backend/solver.h"
#include "oh_my_vslam/core/frame.h"

namespace oh_my_vslam {

class Backend {
 public:
  explicit Backend(const YAML::Node &config) : config_(config){};

  void Process();

  common::Pose3d pose_delta() const { return pose_delta_; }

 private:
  void GetLatestKeyFrames(std::vector<StereoFrame::Ptr> *key_frames) const;

  void ConstructGraph(const std::vector<StereoFrame::Ptr> &key_frames,
                      backend::Solver *solver) const;

  YAML::Node config_;
  common::Pose3d pose_delta_;
  DISALLOW_COPY_AND_ASSIGN(Backend);
};

}  // namespace oh_my_vslam