#pragma once

#include "common/geometry/pose3d.h"
#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/map.h"

namespace oh_my_vslam {

class VO {
 public:
  using Ptr = std::shared_ptr<VO>;
  using ConstPtr = std::shared_ptr<const VO>;

 public:
  VO() = default;

  // return: inlier number
  size_t Process(const StereoFrame::Ptr &frame, bool init = false);

 private:
  size_t Triangulation(const StereoFrame::Ptr &frame);

  void FindOutliers(const Frame::ConstPtr &frame,
                    std::vector<size_t> *outlier_indices) const;

  StereoFrame::Ptr frame_last_;
};

}  // namespace oh_my_vslam