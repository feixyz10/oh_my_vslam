#include "oh_my_vslam/core/frame.h"
#include "oh_my_vslam/core/feature.h"
#include "oh_my_vslam/core/map_point.h"

namespace oh_my_vslam {

bool StereoFrame::Triangulation() {
  StereoCamera::Ptr cam = std::static_pointer_cast<StereoCamera>(camera_);
  for (size_t i = 0; i < features_.size(); ++i) {
    if (features_rgt_[i] == nullptr) continue;
    auto &p1 = features_[i]->pt;
    auto &p2 = features_rgt_[i]->pt;
    MapPoint::Ptr map_point(new MapPoint);
    Eigen::Vector3d pt =
        cam->Triangulation({p1.x, p1.y}, {p2.x, p2.y}, pose_c2w_);
    map_point->SetPosition(pt);
    map_point->AddFeature(features_[i]);
    map_point->AddFeature(features_rgt_[i]);
    features_[i]->map_point = map_point;
  }
  return true;
}

}  // namespace oh_my_vslam