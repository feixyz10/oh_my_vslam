#include "oh_my_vslam/core/map.h"

namespace oh_my_vslam {

Map::Map() = default;

void Map::InsertKeyFrame(const Frame::Ptr &key_frame) {
  key_frames_.insert({key_frame->id(), key_frame});
}

void Map::InsertMapPoint(const MapPoint::Ptr &map_point) {
  map_points_.insert({map_point->id(), map_point});
}

}  // namespace oh_my_vslam