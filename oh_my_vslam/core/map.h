#pragma once

#include <memory>
#include <unordered_map>

#include "common/common.h"
#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/map_point.h"

namespace oh_my_vslam {

class Map {
 public:
  void InsertKeyFrame(const Frame::Ptr &key_frame);

  void RemoveKeyFrame(const size_t id);

  void InsertMapPoint(const MapPoint::Ptr &map_point);

  void RemoveMapPoint(const size_t id);

  const auto &map_points() const {
    return map_points_;
  }

  const auto &key_frames() const {
    return key_frames_;
  }

 protected:
  std::unordered_map<size_t, MapPoint::Ptr> map_points_;
  std::unordered_map<size_t, Frame::Ptr> key_frames_;

  DECLARE_SINGLETON(Map);
};

}  // namespace oh_my_vslam