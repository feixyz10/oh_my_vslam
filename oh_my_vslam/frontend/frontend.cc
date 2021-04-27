#include "oh_my_vslam/frontend/frontend.h"

namespace oh_my_vslam {

Frontend::Frontend(const StereoCamera::ConstPtr &camera,
                   const YAML::Node &config)
    : camera_(camera), config_(config) {
  feature_tracker_.reset(new FeatureTracker(150));
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
  frame_last_ = frame;
}

void Frontend::Initialize(const StereoFrame::Ptr &frame) {
  size_t number_feats = feature_tracker_->Track(frame);
  if (number_feats < 50) {
    AINFO << "Initialization failed";
    return;
  }
  vo_->Process(frame, true);
  state_ = FrontendState::TRACKING;
  InsertKeyframe(frame);
  AINFO << "Initialization OK";
}

void Frontend::Track(const StereoFrame::Ptr &frame) {
  size_t number_tracked = feature_tracker_->Track(frame_last_, frame);
  if (number_tracked < 50) {
    state_ = FrontendState::LOST;
    AERROR << "Tracking lost";
    return;
  }
  size_t number_inlier = vo_->Process(frame);
  if (number_inlier < 50) {
    state_ = FrontendState::LOST;
    AERROR << "Tracking lost";
    return;
  }
  AINFO << "Tracking OK";
  if (number_inlier < 66) {
    InsertKeyframe(frame);
  }
}

void Frontend::InsertKeyframe(const StereoFrame::Ptr &frame) {
  feature_tracker_->Track(frame);
  frame->SetKeyFrame();
}

void Frontend::Reset() {}

}  // namespace oh_my_vslam