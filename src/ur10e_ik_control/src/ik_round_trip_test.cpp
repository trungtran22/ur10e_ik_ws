#include "ur10e_ik_control/ur10e_kinematics.hpp"
#include <cstdio>
#include <random>
#include <cmath>

int main() {
  std::mt19937 gen(42);
  std::uniform_real_distribution<double> d(-M_PI, M_PI);
  const int total = 500;
  int pass = 0; double worst = 0.0;
  for (int t = 0; t < total; ++t) {
    ur10e_ik::JointArray q{d(gen),d(gen),d(gen),d(gen),d(gen),d(gen)};
    Eigen::Matrix4d T = ur10e_ik::forward(q);
    double best = 1e9;
    for (const auto & s : ur10e_ik::inverse(T))
      best = std::min(best, (ur10e_ik::forward(s) - T).cwiseAbs().maxCoeff());
    worst = std::max(worst, best);
    if (best < 1e-6) ++pass;
  }
  std::printf("Round-trip: %d/%d within 1e-6 (worst=%.3e)\n", pass, total, worst);
  return pass == total ? 0 : 1;
}