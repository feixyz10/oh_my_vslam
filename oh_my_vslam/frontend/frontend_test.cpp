#include "oh_my_vslam/frontend/frontend.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

#include "common/common.h"

using namespace common;
using namespace oh_my_vslam;

double baseline = 0.53715065;
StereoCamera::Ptr camera{
    new StereoCamera{7.070912000000e+02, 7.070912000000e+02, 6.018873000000e+02,
                     1.831104000000e+02, baseline}};

Eigen::Vector3d t(-0.00145573, -0.01786178, 0.89830408);

int main(int argc, char **argv) {
  std::string im_path = argv[1];
  if (im_path.back() != '/') im_path.push_back('/');

  InitG3Logging();
  AINFO << camera->ToString();
  Frontend frontend(camera);

  std::vector<int> img_ids{600, 601};
  Eigen::Vector3d tn;
  for (auto id : img_ids) {
    std::ostringstream oss;
    oss << im_path << std::setw(6) << std::setfill('0') << id;
    cv::Mat im_lft = cv::imread(im_path + oss.str() + "_left.png");
    cv::Mat im_rgt = cv::imread(im_path + oss.str() + "_right.png");
    StereoFrame::Ptr frame{new StereoFrame{0.0, im_lft, im_rgt, camera}};
    frontend.Process(frame);
    AINFO << "Frame " << frame->id() << ": " << frame->pose_c2w().ToString();
    tn = frame->pose_c2w().t_vec();
  }

  AINFO << "Error: " << (t - tn).norm()
        << ", relative error: " << (t - tn).norm() / t.norm();

  return 0;
}