#include <fcl/fcl.h>
#include <memory>
#include <cstdio>

int main() {
  auto shape = std::make_shared<fcl::Boxd>(1.0, 1.0, 1.0);   // hộp 1x1x1

  fcl::CollisionObjectd a(shape, fcl::Transform3d::Identity());

  fcl::Transform3d tf = fcl::Transform3d::Identity();
  tf.translation() = fcl::Vector3d(0.5, 0.0, 0.0);           // chồng lên a
  fcl::CollisionObjectd b(shape, tf);

  fcl::CollisionRequestd req;
  fcl::CollisionResultd res;
  fcl::collide(&a, &b, req, res);
  std::printf("cach 0.5m -> va cham? %d\n", res.isCollision());   // mong: 1

  tf.translation() = fcl::Vector3d(2.0, 0.0, 0.0);           // tách xa
  fcl::CollisionObjectd c(shape, tf);
  fcl::CollisionResultd res2;
  fcl::collide(&a, &c, req, res2);
  std::printf("cach 2.0m -> va cham? %d\n", res2.isCollision()); // mong: 0
  return 0;
}