#pragma once

#include <memory>
#include <unordered_map>

#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/map_point.h"

namespace oh_my_vslam {

class Map {
 public:
  using Ptr = std::shared_ptr<Map>;
  using ConstPtr = std::shared_ptr<const Map>;

 public:
  Map() = default;

  void InsertKeyFrame(const Frame::Ptr &key_frame);

  void InsertMapPoint(const MapPoint::Ptr &map_point);

 protected:
  std::unordered_map<size_t, MapPoint::Ptr> map_points_;
  std::unordered_map<size_t, Frame::Ptr> key_frames_;
};

}  // namespace oh_my_vslam