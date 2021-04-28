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

int main(int argc, char **argv) {
  std::string im_path = argv[1];
  if (im_path.back() != '/') im_path.push_back('/');

  InitG3Logging();
  AINFO << camera->ToString();
  Frontend frontend(camera);

  std::vector<int> img_ids = common::Range(100, 160);
  AINFO << "Image num: " << img_ids.size();

  std::vector<Eigen::Vector3d> tvecs;
  for (auto id : img_ids) {
    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << id;
    cv::Mat im_lft = cv::imread(im_path + "left/" + oss.str() + ".png");
    cv::Mat im_rgt = cv::imread(im_path + "right/" + oss.str() + ".png");
    StereoFrame::Ptr frame{new StereoFrame{0.0, im_lft, im_rgt, camera}};
    frontend.Process(frame);
    if (FrontendState::LOST == frontend.state()) {
      AERROR << "tracking lost";
      break;
    }
    AINFO << "####### Frame " << frame->id() << ": "
          << frame->pose_c2w().ToString();
    tvecs.push_back(frame->pose_c2w().t_vec());
  }

  std::ofstream ofs(im_path + "pose_pr.txt");
  for (auto &t : tvecs) {
    ofs << t.x() << " " << t.y() << " " << t.z() << std::endl;
  }

  return 0;
}