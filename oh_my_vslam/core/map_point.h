#pragma once

#include <list>
#include <memory>
#include <opencv2/core/core.hpp>

#include "common/geometry/pose3d.h"

namespace oh_my_vslam {

struct Feature;

class MapPoint {
 public:
  using Ptr = std::shared_ptr<MapPoint>;
  using ConstPtr = std::shared_ptr<const MapPoint>;

 public:
  explicit MapPoint(const Eigen::Vector3d &position = {0.0, 0.0, 0.0})
      : position_(position) {
    static size_t static_id = 0;
    id_ = static_id++;
  };

  size_t id() const { return id_; }

  Eigen::Vector3d position() const { return position_; }

  void SetPosition(const Eigen::Vector3d &position) { position_ = position; }

  const auto &features() const { return features_; }

  void AddFeature(const std::shared_ptr<Feature> &feature);

  void RemoveFeature(const std::shared_ptr<Feature> &feature);

 protected:
  size_t id_ = 0;
  Eigen::Vector3d position_{0, 0, 0};  // position in world coorld
  std::list<std::weak_ptr<Feature>> features_;
};

}  // namespace oh_my_vslam