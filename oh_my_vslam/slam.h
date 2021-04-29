#pragma once

#include "oh_my_vslam/backend/backend.h"
#include "oh_my_vslam/frontend/frontend.h"

namespace oh_my_vslam {

class OhMyVSlame {
 public:
  explicit OhMyVSlame(const YAML::Node &config) : config_(config) {
    InitModule();
  };

  bool Run(double timestamp, const StereoFrame::Ptr &frame);

 private:
  bool InitModule();

  std::unique_ptr<Frontend> frontend_;

  std::unique_ptr<Backend> backend_;

  YAML::Node config_;
  DISALLOW_COPY_AND_ASSIGN(OhMyVSlame);
};

}  // namespace oh_my_vslam