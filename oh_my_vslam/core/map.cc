#include "oh_my_vslam/core/map.h"

namespace oh_my_vslam {

Map::Map() = default;

void Map::InsertKeyFrame(const Frame::Ptr &key_frame) {
  key_frames_.insert({key_frame->id(), key_frame});
}

void Map::InsertMapPoint(const MapPoint::Ptr &map_point) {
  map_points_.insert({map_point->id(), map_point});
}

void Map::RemoveKeyFrame(const size_t id) {
  key_frames_.erase(id);
}

void Map::RemoveMapPoint(const size_t id) {
  map_points_.erase(id);
}

}  // namespace oh_my_vslam