#include "oh_my_vslam/slam.h"

namespace oh_my_vslam {

bool OhMyVSlame::InitModule() {
  frontend_.reset(new Frontend(config_["frontend"]));
  backend_.reset(new Backend(config_["backend"]));
  return true;
}

bool OhMyVSlame::Run(double timestamp, const StereoFrame::Ptr &frame) {
  frontend_->Process(frame);
  if (frontend_->state() == FrontendState::LOST) {
    AERROR << "Tracking Lost";
    return false;
  }
  if (frame->is_keyframe()) backend_->Process();
  return true;
}

}  // namespace oh_my_vslam