#include <iostream>
#include <opencv2/opencv.hpp>

#include "common/common.h"
#include "oh_my_vslam/frontend/feature_extractor.h"

using namespace common;
using namespace oh_my_vslam;

Camera::Ptr camera{new Camera{7.070912000000e+02, 7.070912000000e+02,
                              6.018873000000e+02, 1.831104000000e+02}};
double baseline = 0.53715065;

int main(int argc, char **argv) {
  InitG3Logging();
  std::string im_path = argv[1];
  if (im_path.back() != '/') im_path.push_back('/');
  cv::Mat im1_lft = cv::imread(im_path + "000600_left.png");
  cv::Mat im1_rgt = cv::imread(im_path + "000600_right.png");
  cv::Mat im2_lft = cv::imread(im_path + "000601_left.png");
  cv::Mat im2_rgt = cv::imread(im_path + "000601_right.png");

  AINFO << "Image shape: " << im1_lft.rows << " x " << im1_lft.cols << " x "
        << im1_lft.channels() << ": " << im1_lft.type();

  StereoFrame::Ptr frame1{
      new StereoFrame{0.0, im1_lft, im1_rgt, camera, baseline}};
  StereoFrame::Ptr frame2{
      new StereoFrame{0.0, im2_lft, im2_rgt, camera, baseline}};

  FeatureExtractor extractor(100);
  extractor.Process(frame1);
  extractor.Process(frame1, frame2);
  //   extractor.Process(frame2);

  for (auto &feat : frame1->features()) {
    cv::circle(im1_lft,
               {static_cast<int>(feat->pt.x), static_cast<int>(feat->pt.y)}, 5,
               {0, 255, 0});
  }
  for (auto &feat : frame1->features_rgt()) {
    if (feat == nullptr) continue;
    cv::circle(im1_rgt,
               {static_cast<int>(feat->pt.x), static_cast<int>(feat->pt.y)}, 5,
               {0, 255, 0});
  }
  for (auto &feat : frame2->features()) {
    cv::circle(im2_lft,
               {static_cast<int>(feat->pt.x), static_cast<int>(feat->pt.y)}, 5,
               {0, 255, 0});
  }
  for (auto &feat : frame2->features_rgt()) {
    if (feat == nullptr) continue;
    cv::circle(im2_rgt,
               {static_cast<int>(feat->pt.x), static_cast<int>(feat->pt.y)}, 5,
               {0, 255, 0});
  }

  int cols = im1_lft.cols, rows = im1_lft.rows;
  cv::Mat show(rows * 2, cols * 2, im1_lft.type());
  AINFO << "Image shape: " << show.rows << " x " << show.cols << " x "
        << show.channels() << ": " << show.type();

  im1_lft.copyTo(show(cv::Rect(0, 0, cols, rows)));
  im1_rgt.copyTo(show(cv::Rect(cols, 0, cols, rows)));
  im2_lft.copyTo(show(cv::Rect(0, rows, cols, rows)));
  im2_rgt.copyTo(show(cv::Rect(cols, rows, cols, rows)));

  size_t num_valid_matching = 0;
  for (size_t i = 0; i < frame1->features().size(); ++i) {
    const auto &feat1 = frame1->features()[i];
    const auto &feat2 = frame1->features_rgt().at(i);
    if (feat2 == nullptr) continue;
    cv::Scalar color{0, 0, 255};
    if (std::abs(feat2->pt.y - feat1->pt.y) <= 1.5) {
      color = {0, 255, 0};
      num_valid_matching++;
    }
    cv::line(
        show, {static_cast<int>(feat1->pt.x), static_cast<int>(feat1->pt.y)},
        {static_cast<int>(feat2->pt.x) + cols, static_cast<int>(feat2->pt.y)},
        color);
  }
  AINFO << "Num of valid matching: " << num_valid_matching;

  cv::imshow("", show);
  cv::waitKey(0);

  return 0;
}