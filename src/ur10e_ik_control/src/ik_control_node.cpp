#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "ur10e_ik_control/rrt_planner.hpp"
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "ur10e_ik_control/ur10e_kinematics.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>

class IkControlNode : public rclcpp::Node {
public:
  IkControlNode() : Node("ur10e_ik_control") {
    arm_joint_names_ = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                        "wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    current_.fill(0.0);

    gripper_open_  = declare_parameter<double>("gripper_open", 0.0);
    gripper_close_ = declare_parameter<double>("gripper_close", 0.8);
    move_time_     = declare_parameter<double>("move_time", 4.0);

    // publishers
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);
    gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_controller/commands", 10);

    // subscribers
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&IkControlNode::onJointState, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10,
      std::bind(&IkControlNode::onTargetPose, this, std::placeholders::_1));

    // service
    gripper_srv_ = create_service<std_srvs::srv::SetBool>(
      "/set_gripper",
      std::bind(&IkControlNode::onSetGripper, this,
                std::placeholders::_1, std::placeholders::_2));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planner_markers", 10);
    marker_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&IkControlNode::publishObstacle, this));
    RCLCPP_INFO(get_logger(), "UR10e IK node ready.");
  }

private:
  // cập nhật tư thế khớp hiện tại từ /joint_states
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), arm_joint_names_[i]);
      if (it != msg->name.end()) {
        size_t idx = std::distance(msg->name.begin(), it);
        if (idx < msg->position.size()) current_[i] = msg->position[idx];
      }
    }
  }
  static double wrapToPi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // STUB — lượt sau bạn điền: dựng T -> inverse() -> chọn nghiệm -> publish quỹ đạo
  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // (1) dựng T 4x4 từ pose: quaternion -> ma trận xoay, position -> cột tịnh tiến
    tf2::Quaternion qt(msg->pose.orientation.x, msg->pose.orientation.y,
                        msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3 m(qt);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) T(r, c) = m[r][c];
    T(0,3) = msg->pose.position.x;
    T(1,3) = msg->pose.position.y;
    T(2,3) = msg->pose.position.z;

    // (2) gọi IK
    const auto sols = ur10e_ik::inverse(T);
    if (sols.empty()) {
        RCLCPP_WARN(get_logger(), "Khong co nghiem IK (pose ngoai tam).");
        return;
    }

    // (3) CHỌN NGHIỆM gần current_ nhất  (BẠN VIẾT)
    //   - duyệt mọi nghiệm s trong sols
    //   - tính "chi phí" = tổng bình phương sai khác từng khớp so với current_
    //     (nhớ wrap hiệu số về (-pi, pi] trước khi bình phương, như wrapToPi)
    //   - giữ nghiệm có chi phí nhỏ nhất -> ur10e_ik::JointArray best
    //   gợi ý khung:
    double bestCost = 1e18;
    ur10e_ik::JointArray goal; bool found = false;
    for (const auto & s : sols) {
      if (cc_.isColliding(s)) continue;                 // bỏ nghiệm đụng vật cản
      double cost = 0;
      for (int i = 0; i < 6; ++i) { double d = wrapToPi(s[i] - current_[i]); cost += d*d; }
      if (cost < bestCost) { bestCost = cost; goal = s; found = true; }
    }
    if (!found) { RCLCPP_WARN(get_logger(), "Moi nghiem IK deu va cham"); return; }

    // RRT lập đường current_ -> goal
    auto path = rrt_.plan(current_, goal);
    if (path.empty()) { RCLCPP_WARN(get_logger(), "RRT khong tim duoc duong"); return; }
    RCLCPP_INFO(get_logger(), "RRT: %zu waypoint", path.size());

    // Path visualize
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "world";  line.header.stamp = now();
    line.ns = "rrt_path";  line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.01;
    line.color.g = 1.0;  line.color.a = 1.0;
    for (const auto & wp : path) {
      Eigen::Vector3d p = ur10e_ik::forwardAll(wp)[5].block<3,1>(0,3);
      geometry_msgs::msg::Point pt; pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
      line.points.push_back(pt);
    }
    visualization_msgs::msg::MarkerArray arr; arr.markers.push_back(line);
    marker_pub_->publish(arr);

    // publish quỹ đạo NHIỀU điểm
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = arm_joint_names_;                // 6 tên khớp như cũ của bạn
    double dt = 0.4;
    for (size_t k = 0; k < path.size(); ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.assign(path[k].begin(), path[k].end());
      pt.time_from_start = rclcpp::Duration::from_seconds((k + 1) * dt);
      traj.points.push_back(pt);
    }
    traj_pub_->publish(traj);
  }

  // mở/đóng gripper: true -> mở (0.0), false -> đóng (0.04)
  void onSetGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    std_msgs::msg::Float64MultiArray cmd;
    double pos = req->data ? gripper_open_ : gripper_close_;
    cmd.data = {pos};               
    gripper_pub_->publish(cmd);
    res->success = true;
    res->message = req->data ? "open gripper" : "close gripper";
    RCLCPP_INFO(get_logger(), "%s (pos=%.3f)", res->message.c_str(), pos);
  }

  void publishObstacle() {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";  m.header.stamp = now();
    m.ns = "obstacle";  m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = 0.5; m.pose.position.y = 0.2; m.pose.position.z = 0.25;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.15; m.scale.y = 0.15; m.scale.z = 0.5;
    m.color.r = 0.9; m.color.g = 0.3; m.color.b = 0.2; m.color.a = 0.8;
    visualization_msgs::msg::MarkerArray arr; arr.markers.push_back(m);
    marker_pub_->publish(arr);
  }

  std::vector<std::string> arm_joint_names_;
  ur10e_ik::JointArray current_;
  ur10e_ik::CollisionChecker cc_;
  ur10e_ik::RRTPlanner rrt_{cc_};
  double gripper_open_, gripper_close_, move_time_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_srv_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IkControlNode>());
  rclcpp::shutdown();
  return 0;
}