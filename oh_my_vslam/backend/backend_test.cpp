#include "oh_my_vslam/backend/backend.h"

#include <iostream>
#include <sstream>
#include <vector>

using namespace common;
using namespace oh_my_vslam;

double baseline = 0.53715065;
StereoCamera::Ptr camera{
    new StereoCamera{7.070912000000e+02, 7.070912000000e+02, 6.018873000000e+02,
                     1.831104000000e+02, baseline}};

int main(int argc, char **argv) {
  Eigen::Quaterniond a(1, 2, 3, 4);
  double *va = a.coeffs().data();
  std::cout << va[0] << " " << va[1] << " " << va[2] << " " << va[3]
            << std::endl;
  Eigen::Quaterniond b(a.coeffs().data());
  double *vb = b.coeffs().data();
  std::cout << vb[0] << " " << vb[1] << " " << vb[2] << " " << vb[3]
            << std::endl;

  return 0;
}