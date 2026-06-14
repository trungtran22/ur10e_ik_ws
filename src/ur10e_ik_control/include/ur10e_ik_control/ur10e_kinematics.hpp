#pragma once
#include <array>
#include <vector>
#include <Eigen/Dense>

namespace ur10e_ik {

// Tham số DH chính thức của UR10e (mét, chuẩn DH).
struct DH {
  static constexpr double d1 =  0.1807;
  static constexpr double a2 = -0.6127;
  static constexpr double a3 = -0.57155;
  static constexpr double d4 =  0.17415;
  static constexpr double d5 =  0.11985;
  static constexpr double d6 =  0.11655;
};

using JointArray = std::array<double, 6>;   // {theta1..theta6} rad

Eigen::Matrix4d forward(const JointArray & q);          // FK
std::vector<JointArray> inverse(const Eigen::Matrix4d & T);  // IK (lượt sau)

}  // namespace ur10e_ik