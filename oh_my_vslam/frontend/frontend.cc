#include "oh_my_vslam/frontend/frontend.h"

namespace oh_my_vslam {

Frontend::Frontend(const StereoCamera::ConstPtr &camera,
                   const YAML::Node &config)
    : camera_(camera), config_(config) {
  feature_extractor_.reset(new FeatureExtractor(150));
};

void Frontend::Process(const StereoFrame::Ptr &frame) {
  switch (state_) {
    case FrontendState::INITIALIZING:
      Initialize(frame);
      break;
    case FrontendState::TRACKING:
      Track(frame);
      break;
    case FrontendState::LOST:
      Reset();
      break;
  }
}

void Frontend::Initialize(const StereoFrame::Ptr &frame) {
  size_t number_feats = feature_extractor_->Process(frame);
  if (number_feats < 50) {
    AINFO << "Initialization failed";
    return;
  }
  state_ = FrontendState::TRACKING;
  UpdateMap();
  InsertKeyframe(frame);
  AINFO << "Initialization OK";
}

void Frontend::Track(const StereoFrame::Ptr &frame) {
  // size_t number_tracked = feature_extractor_->Process(frame_last_, frame);
  // if (number_tracked < 50) {
  //   state_ = FrontendState::LOST;
  //   AERROR << "Tracking lost";
  //   return;
  // }
  // size_t number_inlier = 60;
  // if (number_inlier < 50) {
  //   state_ = FrontendState::LOST;
  //   AERROR << "Tracking lost";
  //   return;
  // }
  // AINFO << "Tracking OK";
  // if (number_inlier < 80) {
  //   InsertKeyframe(frame);
  // }
}

void Frontend::InsertKeyframe(const StereoFrame::Ptr &frame) {
  feature_extractor_->Process(frame);
}

void Frontend::UpdateMap() const {}

void Frontend::Reset() {}

}  // namespace oh_my_vslam