#include "ur10e_ik_control/ur10e_kinematics.hpp"
#include <cstdio>
int main() {
  ur10e_ik::JointArray q{0, 0, 0, 0, 0, 0};
  auto T = ur10e_ik::forwardAll(q);                       // mảng 6 ma trận
  std::printf("tool  (zeros): %.5f %.5f %.5f\n", T[5](0,3), T[5](1,3), T[5](2,3));
  std::printf("elbow (zeros): %.5f %.5f %.5f\n", T[2](0,3), T[2](1,3), T[2](2,3));
  return 0;
}