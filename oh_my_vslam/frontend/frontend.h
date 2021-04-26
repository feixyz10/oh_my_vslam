#pragma once

#include <memory>

#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/camera.h"
#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/map.h"
#include "oh_my_vslam/core/map_point.h"

namespace oh_my_vslam {

enum class FrontendState {
  INITIALIZING = 0,
  TRACKING_GOOD,
  TRACKING_BAD,
  LOST
};

class Frontend {
 public:
  using Ptr = std::shared_ptr<Frontend>;
  using ConstPtr = std::shared_ptr<const Frontend>;

 public:
  Frontend(const StereoCamera::ConstPtr &camera, const YAML::Node &config)
      : camera_(camera), config_(config){};

  void Process(const StereoFrame::Ptr &frame);

  FrontendState state() const {
    return state_;
  }

 protected:
  bool TrackFrameFeature(const StereoFrame::Ptr &frame);

  bool Initialize(const StereoFrame::Ptr &frame);

  FrontendState state_ = FrontendState::INITIALIZING;
  StereoCamera::ConstPtr camera_ = nullptr;
  StereoFrame::Ptr frame_last_;
  YAML::Node config_;
};

}  // namespace oh_my_vslam