#pragma once

#include <memory>
#include <opencv2/core/core.hpp>

#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/frame.h"

namespace oh_my_vslam {

class FeatureExtractor {
 public:
  using Ptr = std::shared_ptr<FeatureExtractor>;
  using ConstPtr = std::shared_ptr<const FeatureExtractor>;

 public:
  FeatureExtractor() = default;

  void Process(Frame *frame);
};
}  // namespace oh_my_vslam