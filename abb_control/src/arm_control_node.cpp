#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <abb_interfaces/srv/arm_goal.hpp>

#include <abb_control/arm_control.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("arm_control");

  node->declare_parameter<std::string>("world_frame_id", "world");
  node->declare_parameter<std::string>("arm_name", "manipulator");
  // Flat array: groups of 10 [dx, dy, dz, px, py, pz, qx, qy, qz, qw]
  node->declare_parameter("obstacles", std::vector<double>{});

  std::string world_frame_id =
      node->get_parameter("world_frame_id").as_string();
  std::string arm_name = node->get_parameter("arm_name").as_string();
  auto obstacles_flat = node->get_parameter("obstacles").as_double_array();

  std::vector<geometry_msgs::msg::Vector3> dims;
  geometry_msgs::msg::PoseArray obstacle_poses;
  obstacle_poses.header.frame_id = world_frame_id;

  for (size_t i = 0; i + 9 < obstacles_flat.size(); i += 10) {
    geometry_msgs::msg::Vector3 dim;
    dim.x = obstacles_flat[i + 0];
    dim.y = obstacles_flat[i + 1];
    dim.z = obstacles_flat[i + 2];
    dims.push_back(dim);

    geometry_msgs::msg::Pose pose;
    pose.position.x    = obstacles_flat[i + 3];
    pose.position.y    = obstacles_flat[i + 4];
    pose.position.z    = obstacles_flat[i + 5];
    pose.orientation.x = obstacles_flat[i + 6];
    pose.orientation.y = obstacles_flat[i + 7];
    pose.orientation.z = obstacles_flat[i + 8];
    pose.orientation.w = obstacles_flat[i + 9];
    obstacle_poses.poses.push_back(pose);
  }

  // MoveGroupInterface requires the executor to be spinning during construction
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::thread([&executor]() { executor.spin(); });

  arm_control::ArmControl arm_ctrl(node, arm_name);
  arm_ctrl.AddCollisionObjects(dims, obstacle_poses);

  auto srv = node->create_service<abb_interfaces::srv::ArmGoal>(
      "move_arm_to_pose",
      std::bind(&arm_control::ArmControl::MoveToGoalSrv, &arm_ctrl,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(node->get_logger(),
              "arm_control_node ready — serving move_arm_to_pose");

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
