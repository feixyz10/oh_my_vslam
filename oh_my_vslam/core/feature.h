#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "common/common.h"
#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/map_point.h"

namespace oh_my_vslam {

struct Feature {
  using Ptr = std::shared_ptr<Feature>;
  using ConstPtr = std::shared_ptr<const Feature>;

  explicit Feature(const cv::Point2d &pt, const Frame::Ptr &frame = nullptr,
                   const MapPoint::Ptr &map_point = nullptr)
      : pt(pt), frame(frame), map_point(map_point) {}

  cv::Point2d pt;
  std::weak_ptr<Frame> frame;
  std::weak_ptr<MapPoint> map_point;
  bool is_on_left_img = true;  // for stereo only
};

}  // namespace oh_my_vslam