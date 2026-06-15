#include "ur10e_ik_control/ur10e_kinematics.hpp"
#include <cmath>

namespace ur10e_ik{
Eigen::Matrix4d dhTransform (double theta, double d, double a, double alpha){
    Eigen::Matrix4d A;
    A << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
         0, sin(alpha), cos(alpha), d,
         0,   0,   0,   1;
    return A;
}

Eigen::Matrix4d forward(const JointArray & q){
    const double H = M_PI /2.0;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T = T*dhTransform(q[0], DH::d1, 0.0, H);
    T = T*dhTransform(q[1], 0.0, DH::a2, 0.0);
    T = T*dhTransform(q[2], 0.0, DH::a3, 0.0);
    T = T*dhTransform(q[3], DH::d4, 0.0, H);
    T = T*dhTransform(q[4], DH::d5, 0.0, -H);
    T = T*dhTransform(q[5], DH::d6, 0.0, 0.0);

    return T;
}

std::array<Eigen::Matrix4d, 6> forwardAll(const JointArray& q) {
    const double H = M_PI / 2.0;
    std::array<Eigen::Matrix4d, 6> out;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T = T * dhTransform(q[0], DH::d1, 0.0,    H);   out[0] = T;
    T = T * dhTransform(q[1], 0.0,    DH::a2, 0.0);  out[1] = T;
    T = T * dhTransform(q[2], 0.0,    DH::a3, 0.0);  out[2] = T;
    T = T * dhTransform(q[3], DH::d4, 0.0,    H);   out[3] = T;
    T = T * dhTransform(q[4], DH::d5, 0.0,   -H);   out[4] = T;
    T = T * dhTransform(q[5], DH::d6, 0.0,    0.0);  out[5] = T;
    return out;
}

static double clampUnit(double x) { return x > 1.0 ? 1.0 : (x < -1.0 ? -1.0 : x); }

static double wrapToPi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}
/*
        | nx  ox  ax  px |    
T  =    | ny  oy  ay  py |     
        | nz  oz  az  pz |     
        | 0   0   0   1  |
*/
std::vector<JointArray> inverse(const Eigen::Matrix4d & T) {
  std::vector<JointArray> sols;
  const double H = M_PI / 2.0;

  // mở pose: các cột của R là trục của công cụ trong hệ gốc
  const double nx = T(0,0), ny = T(1,0);   // cột 0
  const double ox = T(0,1), oy = T(1,1);   // cột 1
  const double ax = T(0,2), ay = T(1,2);   // cột 2 = trục tiếp cận z6
  const double px = T(0,3), py = T(1,3);   // vị trí p

  // --- theta1 (mẫu): 2 nhánh vai trái/phải ---
  const double p05x = px - DH::d6 * ax;
  const double p05y = py - DH::d6 * ay;
  const double R = std::hypot(p05x, p05y);
  if (R < std::fabs(DH::d4)) return sols;          // tâm cổ tay trong trụ d4 -> ngoài tầm
  const double phi = std::atan2(p05y, p05x);
  const double psi = std::acos(clampUnit(DH::d4 / R));
  const double theta1[2] = { phi + psi + H, phi - psi + H };

  for (int i = 0; i < 2; ++i) {
    const double q1 = theta1[i];
    const double s1 = std::sin(q1), c1 = std::cos(q1);

    // --- theta5 
    const double cos_theta5 = clampUnit((px*s1 - py*c1 - DH::d4) / DH::d6);    
    double theta5[2] = { +acos(cos_theta5), -acos(cos_theta5) };

    for (int j = 0; j < 2; ++j) {
      const double q5 = theta5[j];
      const double s5 = std::sin(q5);
      double q6;

      // --- theta6
      if (std::fabs(s5) < 1e-7){
        q6 = 0.0;
      }            // kỳ dị cổ tay
      else{
        q6 = atan2( (-ox*s1 + oy*c1)/s5 , (nx*s1 - ny*c1)/s5 );
      } 

      // --- bóc T14 (BẠN VIẾT) ---
      Eigen::Matrix4d A1 = dhTransform(q1, DH::d1, 0.0, H);
      Eigen::Matrix4d A5 = dhTransform(q5, DH::d5, 0.0, -H);
      Eigen::Matrix4d A6 = dhTransform(q6, DH::d6, 0.0, 0.0);
      Eigen::Matrix4d T14 = A1.inverse() * T * (A5*A6).inverse();
      const double xp = T14(0,3), yp = T14(1,3), theta234 = atan2(T14(1,0), T14(0,0));

      // theta3
      const double cos_theta3 = clampUnit((xp*xp + yp*yp - DH::a2*DH::a2- DH::a3*DH::a3) / (2*DH::a2*DH::a3));  
      double theta3[2] = { +acos(cos_theta3), -acos(cos_theta3) };

      for (int k = 0; k < 2; ++k) {
        const double q3 = theta3[k];
        const double s3 = std::sin(q3);

        // theta2 
        const double q2 = atan2(yp, xp) - atan2(DH::a3*s3, DH::a2 + DH::a3*cos(q3));
        // theta4
        const double q4 = theta234 - q2 - q3;

        sols.push_back({ wrapToPi(q1), wrapToPi(q2), wrapToPi(q3),
                         wrapToPi(q4), wrapToPi(q5), wrapToPi(q6) });
      }
    }
  }
  return sols;
} 
}