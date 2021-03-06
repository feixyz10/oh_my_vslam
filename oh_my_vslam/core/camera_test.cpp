#include <gtest/gtest.h>

#include "oh_my_vslam/core/camera.h"

namespace oh_my_vslam {

namespace {
double fx = 2160.1510951272617, fy = 2177.5748633739654,
       cx = 947.98781106777142, cy = 609.03121854702931;
Camera cam1{fx, fy, cx, cy};
double k1 = -0.21656942109264732, k2 = 0.20199919403908162,
       k3 = 0.00013614389269339059, p1 = -0.0001971061449042489, p2 = 0.0;
Camera cam2{fx, fy, cx, cy, k1, k2, k3, p1, p2};

double x = 10, y = 10, z = 14;
}  // namespace

TEST(CameraTest, ZeroDistortion) {
  EXPECT_TRUE(cam1.zero_distortion());
  EXPECT_TRUE(!cam2.zero_distortion());
  Camera cam4 = cam2;
  cam4.SetZeroDistortion();
  EXPECT_TRUE(cam4.zero_distortion());
}

TEST(CameraTest, CameraWithoutDistortion) {
  Eigen::Vector3d pt(x, y, z);
  Eigen::Vector2d px2 = cam1.Project(pt);
  Eigen::Vector3d pt_recover = cam1.InverseProject(px2, z);
  EXPECT_NEAR(pt.x(), pt_recover.x(), 1e-6);
  EXPECT_NEAR(pt.y(), pt_recover.y(), 1e-6);
  EXPECT_NEAR(pt.z(), pt_recover.z(), 1e-6);
}

TEST(CameraTest, CameraWithDistortion) {
  Eigen::Vector3d pt(x, y, z);
  Eigen::Vector2d px3 = cam2.Project(pt);
  Eigen::Vector3d pt_recover = cam2.InverseProject(px3, z);
  EXPECT_NEAR(pt.x(), pt_recover.x(), 1e-6);
  EXPECT_NEAR(pt.y(), pt_recover.y(), 1e-6);
  EXPECT_NEAR(pt.z(), pt_recover.z(), 1e-6);
}

}  // namespace oh_my_vslam