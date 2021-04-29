#include "oh_my_vslam/frontend/frontend.h"

namespace oh_my_vslam {

Frontend::Frontend(const YAML::Node &config) : config_(config) {
  feature_tracker_.reset(
      new FeatureTracker(150, config_["verbose"].as<bool>()));
  vo_.reset(new VO(config_["verbose"].as<bool>()));
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
  if (number_feats < config_["min_feature_num_initialization"].as<size_t>()) {
    AWARN << "Frontend: initialization failed";
    return;
  }
  vo_->Triangulate(frame);
  InsertKeyframe(frame, true);
  state_ = FrontendState::TRACKING;
  AINFO << "Frontend: initialization OK";
}

void Frontend::Track(const StereoFrame::Ptr &frame) {
  frame->SetPose(frame_last_->pose_c2w() * pose_delta_);
  size_t number_tracked = feature_tracker_->Track(frame_last_, frame);
  if (number_tracked < config_["feature_num_lost_th"].as<size_t>()) {
    state_ = FrontendState::LOST;
    AERROR << "Frontend: tracking lost";
    return;
  }
  size_t number_inlier = vo_->PnP(frame);
  if (number_inlier < config_["feature_num_lost_th"].as<size_t>()) {
    state_ = FrontendState::LOST;
    AERROR << "Frontend: tracking lost";
    return;
  }
  AINFO << "Frontend: tracking OK";
  if (number_inlier < config_["feature_num_keyframe_th"].as<size_t>()) {
    InsertKeyframe(frame);
  }
  pose_delta_ = frame_last_->pose_w2c() * frame->pose_c2w();
}

void Frontend::InsertKeyframe(const StereoFrame::Ptr &frame, bool init) {
  if (!init) {
    feature_tracker_->Track(frame);
    vo_->Triangulate(frame);
  }
  frame->SetKeyFrame();
  Map::Instance()->InsertKeyFrame(frame);
  AINFO << "Frontend: insert key frame (keyframe_id/frame_id): "
        << frame->keyframe_id() << "/" << frame->id();
}

void Frontend::Reset() {}

}  // namespace oh_my_vslam