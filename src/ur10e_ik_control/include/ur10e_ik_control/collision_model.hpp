#pragma once
#include "ur10e_ik_control/ur10e_kinematics.hpp"
#include <fcl/fcl.h>
#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <algorithm>

namespace ur10e_ik {

// Dựng một capsule (bán kính r) nối p1->p2 trong hệ gốc
inline fcl::CollisionObjectd makeCapsule(const Eigen::Vector3d& p1,
                                         const Eigen::Vector3d& p2, double r) {
  Eigen::Vector3d d = p2 - p1;
  double len = d.norm();
  auto shape = std::make_shared<fcl::Capsuled>(r, std::max(len, 1e-6));
  fcl::Transform3d tf = fcl::Transform3d::Identity();
  tf.translation() = 0.5 * (p1 + p2);
  if (len > 1e-9)
    tf.linear() = Eigen::Quaterniond::FromTwoVectors(
        Eigen::Vector3d::UnitZ(), d / len).toRotationMatrix();
  return fcl::CollisionObjectd(shape, tf);
}

class CollisionChecker {
public:
  CollisionChecker() {
    // Vật cản: trụ hộp 0.15 x 0.15 x 0.5 ở (0.5, 0.2, 0.25)
    auto obs = std::make_shared<fcl::Boxd>(0.15, 0.15, 0.5);
    fcl::Transform3d ot = fcl::Transform3d::Identity();
    ot.translation() = fcl::Vector3d(0.5, 0.2, 0.25);
    obstacle_ = std::make_shared<fcl::CollisionObjectd>(obs, ot);

    // Sàn: hộp lớn, mặt trên ở z = -0.05
    auto flr = std::make_shared<fcl::Boxd>(10.0, 10.0, 0.1);
    fcl::Transform3d ft = fcl::Transform3d::Identity();
    ft.translation() = fcl::Vector3d(0.0, 0.0, -0.10);
    floor_ = std::make_shared<fcl::CollisionObjectd>(flr, ft);
  }

  bool isColliding(const JointArray& q) const {
    auto frames = forwardAll(q);
    std::array<Eigen::Vector3d, 7> pts;
    pts[0] = Eigen::Vector3d::Zero();                     // base
    for (int i = 0; i < 6; ++i) pts[i + 1] = frames[i].block<3,1>(0,3);

    const double r = 0.06;                                // bán kính link ~6cm
    for (int i = 1; i < 6; ++i) {                         // bỏ đoạn base (i=0)
      auto cap = makeCapsule(pts[i], pts[i + 1], r);
      if (hit(cap, *obstacle_)) return true;
      if (hit(cap, *floor_))    return true;
    }
    return false;
  }

private:
  static bool hit(fcl::CollisionObjectd& a, fcl::CollisionObjectd& b) {
    fcl::CollisionRequestd req; fcl::CollisionResultd res;
    fcl::collide(&a, &b, req, res);
    return res.isCollision();
  }
  std::shared_ptr<fcl::CollisionObjectd> obstacle_, floor_;
};

} // namespace ur10e_ik