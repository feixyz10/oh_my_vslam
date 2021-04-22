#pragma once

#include <memory>

#include "common/common.h"
#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/camera.h"
#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/map.h"
#include "oh_my_vslam/core/map_point.h"

namespace oh_my_vslam {

class VO {
 public:
  using Ptr = std::shared_ptr<VO>;
  using ConstPtr = std::shared_ptr<const VO>;

 public:
  explicit VO(const YAML::Node &config) : config_(config) {}

  void Process(const Frame::Ptr &frame);

 protected:
  YAML::Node config_;
};

}  // namespace oh_my_vslam