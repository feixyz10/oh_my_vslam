#pragma once

#include <memory>
#include <opencv2/core/core.hpp>

#include "common/geometry/pose3d.h"

namespace oh_my_vslam {

class MapPoint {
 public:
  using Ptr = std::shared_ptr<MapPoint>;
  using ConstPtr = std::shared_ptr<const MapPoint>;

 public:
  MapPoint() = default;
  explicit MapPoint(const size_t id,
                    const Eigen::Vector3d &position = Eigen::Vector3d::Zero())
      : id_(id), position_(position){};

  size_t id() const {
    return id_;
  }

  Eigen::Vector3d position() const {
    return position_;
  }

  void SetPosition(const Eigen::Vector3d &position) {
    position_ = position;
  }

 protected:
  size_t id_ = 0;
  Eigen::Vector3d position_{0, 0, 0};  // position in world coorld
};

}  // namespace oh_my_vslam