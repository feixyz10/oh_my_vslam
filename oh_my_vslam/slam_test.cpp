#include "oh_my_vslam/slam.h"

#include <iostream>
#include <opencv2/opencv.hpp>

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
  std::string conf_path = argv[2];
  if (conf_path.back() != '/') conf_path.push_back('/');

  InitG3Logging();
  YAMLConfig::Instance()->Init(conf_path + "kitti_seq05.yaml");
  auto config = YAMLConfig::Instance()->config();
  OhMyVSlame slam(config);

  std::vector<int> img_ids = common::Range(100, 160);
  AINFO << "Image num: " << img_ids.size();
  AINFO << "First image path: " << im_path + "left/" << std::setw(6)
        << std::setfill('0') << img_ids[0] << ".png";

  std::unordered_map<size_t, Eigen::Vector4d> tvecs;
  for (auto id : img_ids) {
    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << id;
    cv::Mat im_lft = cv::imread(im_path + "left/" + oss.str() + ".png");
    cv::Mat im_rgt = cv::imread(im_path + "right/" + oss.str() + ".png");
    StereoFrame::Ptr frame{new StereoFrame{0.0, im_lft, im_rgt, camera}};
    AUSER << "############## frame " << frame->id() << " ##############";
    if (!slam.Run(0.0, frame)) break;
    AINFO << frame->pose_c2w().ToString();
    tvecs.insert({frame->id(), frame->pose_c2w().t_vec().homogeneous()});
    if (!frame->is_keyframe()) {
      tvecs.at(frame->id())[3] = 0;
    }
  }

  for (auto &[i, frame] : Map::Instance()->key_frames()) {
    tvecs.at(frame->id()) = frame->pose_c2w().t_vec().homogeneous();
  }

  std::ofstream ofs(im_path + "pose_slam_pr.txt");
  for (auto &[i, t] : tvecs) {
    ofs << t.x() << " " << t.y() << " " << t.z() << " " << t.w() << std::endl;
  }

  return 0;
}