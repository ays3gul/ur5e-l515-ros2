#include <abb_control/arm_control.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <pthread.h>

namespace arm_control {

ArmControl::ArmControl(rclcpp::Node::SharedPtr node,
                       const std::string& planning_group)
    : node_(node), move_group_(node, planning_group) {}

ArmControl::~ArmControl() {}

bool ArmControl::MoveToGoal(const geometry_msgs::msg::Pose& goal_pose) {
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group_.setPoseTarget(goal_pose);

  bool success = static_cast<bool>(move_group_.plan(my_plan));
  RCLCPP_INFO(node_->get_logger(), "Planning (pose goal) %s",
              success ? "SUCCEEDED" : "FAILED");

  if (success) {
    success = static_cast<bool>(move_group_.execute(my_plan));
    RCLCPP_INFO(node_->get_logger(), "Execution (pose goal) %s",
                success ? "SUCCEEDED" : "FAILED");
  }
  return success;
}

void ArmControl::MoveToGoalSrv(
    const std::shared_ptr<abb_interfaces::srv::ArmGoal::Request> request,
    std::shared_ptr<abb_interfaces::srv::ArmGoal::Response> response) {
  int entry_count = ++active_srv_count_;
  RCLCPP_INFO(node_->get_logger(),
              "MoveToGoalSrv ENTER tid=%lu active=%d",
              pthread_self(), entry_count);
  {
    std::unique_lock<std::mutex> lock(move_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      RCLCPP_WARN(node_->get_logger(),
                  "MoveToGoalSrv BLOCKED waiting for mutex tid=%lu active=%d",
                  pthread_self(), active_srv_count_.load());
      lock.lock();
    }
    RCLCPP_INFO(node_->get_logger(),
                "MoveToGoalSrv GOT MUTEX tid=%lu active=%d",
                pthread_self(), active_srv_count_.load());
    response->success = MoveToGoal(request->goal_pose);
    RCLCPP_INFO(node_->get_logger(),
                "MoveToGoalSrv RELEASE MUTEX tid=%lu success=%d",
                pthread_self(), (int)response->success);
  }
  --active_srv_count_;
}

void ArmControl::AddCollisionObjects(
    const std::vector<geometry_msgs::msg::Vector3>& dims,
    const geometry_msgs::msg::PoseArray& obstacle_poses) {
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  for (size_t i = 0; i < dims.size(); i++) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box" + std::to_string(i);

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dims[i].x;
    primitive.dimensions[1] = dims[i].y;
    primitive.dimensions[2] = dims[i].z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(obstacle_poses.poses[i]);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(collision_object);
  }

  RCLCPP_INFO(node_->get_logger(), "Adding %zu collision objects to planning scene",
              collision_objects.size());
  planning_scene_interface_.applyCollisionObjects(collision_objects);
  rclcpp::sleep_for(std::chrono::seconds(1));
}

}  // namespace arm_control
