#pragma once
#include "ur10e_ik_control/collision_model.hpp"
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

namespace ur10e_ik {

struct RRTNode { JointArray q; int parent; };

class RRTPlanner {
public:
  explicit RRTPlanner(const CollisionChecker& cc)
    : cc_(cc), rng_(std::random_device{}()) {}

  // Tra ve duong di {start ... goal}, hoac rong neu that bai
  std::vector<JointArray> plan(const JointArray& start, const JointArray& goal) {
    tree_.clear();
    tree_.push_back({start, -1});

    for (int iter = 0; iter < maxIter_; ++iter) {
      JointArray q_rand = bias() ? goal : sample();
      int near = nearest(q_rand);
      JointArray q_new = steer(tree_[near].q, q_rand);

      if (edgeFree(tree_[near].q, q_new)) {              // (B) chỉ nhận cạnh sạch
            tree_.push_back({q_new, near});
            int idx = (int)tree_.size() - 1;

            if (dist(q_new, goal) < goalTol_ && edgeFree(q_new, goal)) {   // (C) tới đích
                tree_.push_back({goal, idx});
                return tracePath((int)tree_.size() - 1);
            }
        }
    }
    return {}; // that bai
  }

private:
  // ===== BAN VIET =====
  
  double dist(const JointArray& a, const JointArray& b) const {
    // TODO(A1): sqrt( tong wrapToPi(a[i]-b[i])^2 )
    double s = 0.0;
    for (int i =0; i < 6; ++i){
        double d = wrapToPi(a[i]-b[i]);
        s += d*d;
    }

    return std::sqrt(s);
  }
  JointArray steer(const JointArray& from, const JointArray& to) const {
    JointArray diff;
    double d = 0.0;
    for (int i = 0; i < 6; ++i) {
        diff[i] = wrapToPi(to[i] - from[i]);
        d += diff[i] * diff[i];
    }
    d = std::sqrt(d);

    if (d <= step_) return to;                  // đã trong tầm 1 bước -> tới thẳng

    JointArray out;
    for (int i = 0; i < 6; ++i)
        out[i] = wrapToPi(from[i] + step_ * diff[i] / d);   // tiến step_ theo hướng diff
    return out;
    }
  bool edgeFree(const JointArray& a, const JointArray& b) const {
    int n = (int)std::ceil(dist(a, b) / res_);
    for (int k = 1; k <= n; ++k) {
        JointArray q;                                   // MỘT điểm nội suy
        for (int i = 0; i < 6; ++i)
            q[i] = wrapToPi(a[i] + (double)k / n * wrapToPi(b[i] - a[i]));
        if (cc_.isColliding(q)) return false;           // dính BẤT KỲ điểm nào -> loại ngay
    }
    return true;                                        // mọi điểm đều sạch
}

  // ===== CO SAN =====
  JointArray sample() {
    JointArray q; std::uniform_real_distribution<double> d(-M_PI, M_PI);
    for (auto& v : q) v = d(rng_); return q;
  }
  bool bias() { std::uniform_real_distribution<double> d(0,1); return d(rng_) < goalBias_; }
  int nearest(const JointArray& q) const {
    int best = 0; double bd = 1e18;
    for (int i = 0; i < (int)tree_.size(); ++i) {
      double dd = dist(tree_[i].q, q);
      if (dd < bd) { bd = dd; best = i; }
    }
    return best;
  }
  std::vector<JointArray> tracePath(int idx) const {
    std::vector<JointArray> p;
    for (int i = idx; i != -1; i = tree_[i].parent) p.push_back(tree_[i].q);
    std::reverse(p.begin(), p.end()); return p;
  }
  static double wrapToPi(double a) {
    while (a >  M_PI) a -= 2*M_PI;
    while (a <= -M_PI) a += 2*M_PI;
    return a;
  }

  const CollisionChecker& cc_;
  std::mt19937 rng_;
  std::vector<RRTNode> tree_;
  double step_     = 0.3;    // buoc steer (rad)
  double res_      = 0.05;   // do phan giai kiem canh
  double goalBias_ = 0.1;    // 10% gieo thang goal
  double goalTol_  = 0.3;    // nguong "du gan" goal
  int    maxIter_  = 20000;
};

} // namespace ur10e_ik