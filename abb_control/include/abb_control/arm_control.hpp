#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <abb_interfaces/srv/arm_goal.hpp>

namespace arm_control {

class ArmControl {
protected:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::mutex move_mutex_;
  std::atomic<int> active_srv_count_{0};

public:
  ArmControl(rclcpp::Node::SharedPtr node, const std::string& planning_group);
  ~ArmControl();

  bool MoveToGoal(const geometry_msgs::msg::Pose& goal_pose);

  void MoveToGoalSrv(
      const std::shared_ptr<abb_interfaces::srv::ArmGoal::Request> request,
      std::shared_ptr<abb_interfaces::srv::ArmGoal::Response> response);

  void AddCollisionObjects(
      const std::vector<geometry_msgs::msg::Vector3>& dims,
      const geometry_msgs::msg::PoseArray& obstacle_poses);
};

}  // namespace arm_control
