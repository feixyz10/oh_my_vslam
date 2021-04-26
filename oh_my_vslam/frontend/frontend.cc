#include "oh_my_vslam/frontend/frontend.h"

namespace oh_my_vslam {

void Frontend::Process(const StereoFrame::Ptr &frame) {
  if (state_ == FrontendState::INITIALIZING) {
    Initialize(frame);
  }
}

}  // namespace oh_my_vslam