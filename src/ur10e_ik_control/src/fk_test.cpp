#include "ur10e_ik_control/ur10e_kinematics.hpp"
#include <cstdio>

int main() {
  ur10e_ik::JointArray q{0, 0, 0, 0, 0, 0};
  Eigen::Matrix4d T = ur10e_ik::forward(q);
  std::printf("position (zeros): %.5f %.5f %.5f\n", T(0,3), T(1,3), T(2,3));
  return 0;
}