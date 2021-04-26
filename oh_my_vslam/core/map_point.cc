#include "oh_my_vslam/core/map_point.h"
#include "oh_my_vslam/core/feature.h"

namespace oh_my_vslam {

void MapPoint::AddFeature(const std::shared_ptr<Feature> &feature) {
  features_.push_back(feature);
}

void MapPoint::RemoveFeature(const std::shared_ptr<Feature> &feature) {
  for (auto iter = features_.begin(); iter != features_.end(); ++iter) {
    if (iter->lock() == feature) {
      features_.erase(iter);
      feature->map_point.reset();
    }
  }
}

};  // namespace oh_my_vslam
