#include "ur10e_ik_control/collision_model.hpp"
#include <cstdio>
using namespace ur10e_ik;

static Eigen::Matrix4d poseDown(double x, double y, double z) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(1,1) = -1; T(2,2) = -1;            // xoay 180° quanh x: mũi chúc xuống
  T(0,3) = x; T(1,3) = y; T(2,3) = z;
  return T;
}

int main() {
  CollisionChecker cc;
  std::printf("home          -> va cham? %d (mong 0)\n", cc.isColliding({0,0,0,0,0,0}));
  auto sols = inverse(poseDown(0.5, 0.2, 0.25));          // tâm vật cản
  if (!sols.empty())
    std::printf("trong vat can -> va cham? %d (mong 1)\n", cc.isColliding(sols[0]));
  else
    std::printf("IK vo nghiem tai vat can\n");
  return 0;
}