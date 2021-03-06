#include "oh_my_vslam/frontend/vo.h"

#include <iostream>
#include <opencv2/opencv.hpp>

#include "common/common.h"
#include "oh_my_vslam/frontend/feature_tracker.h"

using namespace common;
using namespace oh_my_vslam;

double baseline = 0.53715065;
StereoCamera::Ptr camera{
    new StereoCamera{7.070912000000e+02, 7.070912000000e+02, 6.018873000000e+02,
                     1.831104000000e+02, baseline}};

Eigen::Vector3d t{-1.33360599e-02, -2.52272549e-02, 8.14812743e-01};

int main(int argc, char **argv) {
  InitG3Logging();
  std::string im_path = argv[1];
  if (im_path.back() != '/') im_path.push_back('/');
  cv::Mat im1_lft = cv::imread(im_path + "left/000100.png");
  cv::Mat im1_rgt = cv::imread(im_path + "right/000100.png");
  cv::Mat im2_lft = cv::imread(im_path + "left/000101.png");
  cv::Mat im2_rgt = cv::imread(im_path + "right/000101.png");

  StereoFrame::Ptr frame1{new StereoFrame{0.0, im1_lft, im1_rgt, camera}};
  StereoFrame::Ptr frame2{new StereoFrame{0.0, im2_lft, im2_rgt, camera}};
  AINFO << frame1->camera()->ToString();

  FeatureTracker extractor(150, true);
  extractor.Track(frame1);
  VO vo;
  vo.Triangulate(frame1);

  extractor.Track(frame1, frame2);
  vo.PnP(frame2);
  AINFO << frame2->pose_c2w().ToString();
  AINFO << "tvec_gt: " << t[0] << " " << t[1] << " " << t[2];
  AINFO << "tvec error: " << (t - frame2->pose_c2w().t_vec()).norm()
        << ", relative error: "
        << (t - frame2->pose_c2w().t_vec()).norm() / t.norm();

  return 0;
}