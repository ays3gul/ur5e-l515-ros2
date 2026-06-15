"""
Full robot stack launch for ABB IRB1200 + L515 (simulation mode).

Starts (in order):
  1. robot_state_publisher     — TF tree from joint states + URDF
  2. controller_manager        — ros2_control with mock_components hardware
  3. joint_state_broadcaster   — publishes /joint_states from hardware
  4. joint_trajectory_controller — accepts FollowJointTrajectory actions
  5. move_group                — MoveIt2 planning server
  6. arm_control_node          — serves move_arm_to_pose ROS2 service
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("abb_l515_moveit_config_ros2")
    abb_control_dir = get_package_share_directory("abb_control")

    moveit_config = (
        MoveItConfigsBuilder("abb_l515", package_name="abb_l515_moveit_config_ros2")
        .robot_description(file_path="config/abb_l515.urdf")
        .robot_description_semantic(file_path="config/abb_l515.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    robot_description_param = moveit_config.robot_description

    # 1. Publish TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # 2. ros2_control controller manager with mock hardware
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description_param,
            os.path.join(pkg_dir, "config", "ros2_controllers.yaml"),
        ],
    )

    # 3. Joint state broadcaster (reads from hardware, publishes /joint_states)
    joint_state_broadcaster = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller", "--set-state", "active",
                     "joint_state_broadcaster"],
                output="screen",
            )
        ],
    )

    # 4. Joint trajectory controller (accepts FollowJointTrajectory from MoveIt)
    joint_trajectory_controller = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller", "--set-state", "active",
                     "joint_trajectory_controller"],
                output="screen",
            )
        ],
    )

    # 5. MoveIt2 move_group server
    # moveit_controllers.yaml and sensors_3d.yaml must be loaded as dicts,
    # NOT passed as --params-file, because they don't use ros__parameters format
    with open(os.path.join(pkg_dir, "config", "moveit_controllers.yaml")) as f:
        moveit_controllers = yaml.safe_load(f)

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_controllers,
        ],
    )

    # 6. Arm control service node (serves move_arm_to_pose)
    arm_control_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="abb_control",
                executable="arm_control_node",
                name="arm_control",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    os.path.join(abb_control_dir, "config", "arm_control.yaml"),
                    os.path.join(abb_control_dir, "config", "obstacles.yaml"),
                ],
            )
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        joint_trajectory_controller,
        move_group,
        arm_control_node,
    ])
