#pragma once

#include <memory>

#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/camera.h"
#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/map.h"
#include "oh_my_vslam/core/map_point.h"
#include "oh_my_vslam/frontend/feature_tracker.h"
#include "oh_my_vslam/frontend/vo.h"

namespace oh_my_vslam {

enum class FrontendState { INITIALIZING = 0, TRACKING, LOST };

class Frontend {
 public:
  using Ptr = std::shared_ptr<Frontend>;
  using ConstPtr = std::shared_ptr<const Frontend>;

 public:
  Frontend(const StereoCamera::ConstPtr &camera, const YAML::Node &config = {});

  void Process(const StereoFrame::Ptr &frame);

  FrontendState state() const {
    return state_;
  }

 protected:
  void Initialize(const StereoFrame::Ptr &frame);

  void Track(const StereoFrame::Ptr &frame);

  void InsertKeyframe(const StereoFrame::Ptr &frame, bool init = false);

  void Reset();

  FrontendState state_ = FrontendState::INITIALIZING;
  StereoCamera::ConstPtr camera_ = nullptr;
  StereoFrame::Ptr frame_last_{nullptr};
  common::Pose3d pose_delta_;
  std::unique_ptr<FeatureTracker> feature_tracker_;
  std::unique_ptr<VO> vo_;
  YAML::Node config_;
};

}  // namespace oh_my_vslam