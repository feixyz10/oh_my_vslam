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
  Frontend() = default;

  bool TrackNewFrame(const Frame::Ptr &frame);

 protected:
  FrontendState state_ = FrontendState::INITIALIZING;
  Camera::ConstPtr camera_ = nullptr;
};

}  // namespace oh_my_vslam