#pragma once

#include <cstddef>

#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/map.h"
#include "oh_my_vslam/frontend/feature_tracker.h"

namespace oh_my_vslam {

class VO {
 public:
  using Ptr = std::shared_ptr<VO>;
  using ConstPtr = std::shared_ptr<const VO>;

 public:
  VO() = default;

  // return: inlier number
  size_t PnP(const Frame::Ptr &frame);

  size_t Triangulate(const StereoFrame::Ptr &frame);

 private:
  void FindOutliers(const Frame::ConstPtr &frame,
                    std::vector<size_t> *outlier_indices) const;

  DISALLOW_COPY_AND_ASSIGN(VO);
};

}  // namespace oh_my_vslam