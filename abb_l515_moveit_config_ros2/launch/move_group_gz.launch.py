"""
Gz Harmonic simulation launch for ABB IRB1200 + L515.

Starts (in order):
  1. gz sim          — physics + rendering + sensor simulation
  2. spawn_entity    — loads abb_l515_gz.urdf into Gz Sim world
  3. robot_state_publisher — TF tree from joint states
  4. ros_gz_bridge   — clock + camera topics Gz→ROS
  5. controller spawners — joint_state_broadcaster, joint_trajectory_controller
  6. move_group      — MoveIt2 planning server (use_sim_time=true)
  7. arm_control_node — serves move_arm_to_pose (use_sim_time=true)

PREREQUISITE: run this launch file first, then run_ros2_gz.sh in a second terminal.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("abb_l515_moveit_config_ros2")
    abb_control_dir = get_package_share_directory("abb_control")

    world_file = os.path.join(pkg_dir, "worlds", "bunny_gz.sdf")
    gz_urdf   = os.path.join(pkg_dir, "config", "abb_l515_gz.urdf")

    # Read URDF string for spawn_entity and robot_state_publisher
    with open(gz_urdf, "r") as f:
        robot_description_str = f.read()

    use_sim_time = {"use_sim_time": True}

    moveit_config = (
        MoveItConfigsBuilder("abb_l515", package_name="abb_l515_moveit_config_ros2")
        .robot_description(file_path="config/abb_l515_gz.urdf")
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

    # 1. Start Gz Sim with the bunny world (headless by default; remove -s for GUI)
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
    )

    # 2. Spawn the robot URDF into the running Gz Sim world
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", "abb_l515",
                    "-string", robot_description_str,
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.0",
                ],
                output="screen",
            )
        ],
    )

    # 3. Publish TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_str},
            use_sim_time,
        ],
    )

    # 4. Bridge Gz topics → ROS.
    #    Camera sensor publishes: camera/image, camera/depth_image,
    #    camera/points, camera/camera_info  (Gz topic prefix = sensor <topic> tag)
    #    /clock is bridged so all nodes can use sim time.
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/camera/image",       "/camera/color/image_rect_color"),
            ("/camera/depth_image", "/camera/aligned_depth_to_color/image_raw"),
            ("/camera/camera_info", "/camera/aligned_depth_to_color/camera_info"),
            ("/camera/points",      "/camera/depth_registered/points"),
        ],
        parameters=[use_sim_time],
        output="screen",
    )

    # 5. Controller spawners (gz_ros2_control plugin creates the controller_manager;
    #    these nodes wait for it, then activate each controller)
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    joint_trajectory_controller_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    # 6. MoveIt2 move_group (sim time, no separate ros2_control_node needed)
    with open(os.path.join(pkg_dir, "config", "moveit_controllers.yaml")) as f:
        moveit_controllers = yaml.safe_load(f)

    move_group = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_controllers,
                    use_sim_time,
                ],
            )
        ],
    )

    # 7. Arm control service node (serves move_arm_to_pose)
    arm_control_node = TimerAction(
        period=12.0,
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
                    use_sim_time,
                ],
            )
        ],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        move_group,
        arm_control_node,
    ])
