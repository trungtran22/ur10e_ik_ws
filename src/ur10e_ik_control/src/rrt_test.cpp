#include "ur10e_ik_control/rrt_planner.hpp"
#include <cstdio>
using namespace ur10e_ik;

static Eigen::Matrix4d poseDown(double x,double y,double z){
  Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
  T(1,1)=-1; T(2,2)=-1; T(0,3)=x; T(1,3)=y; T(2,3)=z; return T;
}

int main(){
  CollisionChecker cc;
  RRTPlanner rrt(cc);
  JointArray start{0,0,0,0,0,0};

  // goal: ngay tren dinh vat can (hop cao toi z=0.5) -> chon nghiem IK khong va cham
  auto sols = inverse(poseDown(0.5, 0.2, 0.8));
  JointArray goal{}; bool found=false;
  for (auto& s : sols) if (!cc.isColliding(s)) { goal=s; found=true; break; }
  if(!found){ std::printf("Khong co goal config sach!\n"); return 1; }

  std::printf("start va cham? %d | goal va cham? %d\n",
              cc.isColliding(start), cc.isColliding(goal));

  auto path = rrt.plan(start, goal);
  if (path.empty()){ std::printf("RRT THAT BAI\n"); return 1; }

  std::printf("RRT THANH CONG: %zu waypoint\n", path.size());
  bool ok=true; for (auto& q : path) if (cc.isColliding(q)) ok=false;
  std::printf("Moi waypoint sach? %d (mong 1)\n", ok);
  return 0;
}