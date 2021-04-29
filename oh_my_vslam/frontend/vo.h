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
  explicit VO(bool verbose = false) : verbose_(verbose){};

  // return: inlier number
  size_t PnP(const Frame::Ptr &frame, size_t max_iter_num = 5,
             double outlier_std = 2.0);

  size_t Triangulate(const StereoFrame::Ptr &frame);

 private:
  void FindOutliers(const Frame::ConstPtr &frame, double outlier_std,
                    std::vector<size_t> *outlier_indices) const;

  bool verbose_ = false;
  DISALLOW_COPY_AND_ASSIGN(VO);
};

}  // namespace oh_my_vslam