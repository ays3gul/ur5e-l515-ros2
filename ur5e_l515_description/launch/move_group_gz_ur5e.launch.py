"""
Gz Harmonic simulation launch for UR5e + D455 (RH-NBV thesis).

Starts (in order):
  1. gz sim          - physics + rendering + sensors (custom world with Sensors plugin)
  2. robot_state_publisher - TF tree from joint states
  3. spawn_entity    - loads ur5e_l515 URDF into Gz Sim world
  4. ros_gz_bridge   - clock + camera topics Gz->ROS
  5. controller spawners - joint_state_broadcaster, scaled_joint_trajectory_controller
  6. move_group      - MoveIt2 planning server (use_sim_time=true)
  7. arm_control_node - serves move_arm_to_pose (use_sim_time=true)

PREREQUISITE: run this launch file first, then run_ros2_gz.sh in a second terminal.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("ur5e_l515_description")

    world_file = os.path.join(pkg_dir, "worlds", "ur5e_world.sdf")

    mappings = {
        "name": "ur",
        "ur_type": "ur5e",
        "tf_prefix": "",
        "safety_limits": "true",
        "safety_pos_margin": "0.15",
        "safety_k_position": "20",
        "simulation_controllers": os.path.join(
            get_package_share_directory("ur_simulation_gz"), "config", "ur_controllers.yaml"
        ),
    }

    use_sim_time = {"use_sim_time": True}

    moveit_config = (
        MoveItConfigsBuilder("ur5e_l515", package_name="ur5e_l515_description")
        .robot_description(file_path="urdf/ur5e_l515.urdf.xacro", mappings=mappings)
        .robot_description_semantic(file_path="config/ur5e_l515.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    robot_description_str = moveit_config.robot_description["robot_description"]

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, use_sim_time],
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "ur", "-topic", "robot_description",
                           "-x", "0.0", "-y", "0.0", "-z", "0.0"],
                output="screen",
            )
        ],
    )

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

    jtc_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["scaled_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    with open(os.path.join(pkg_dir, "config", "moveit_controllers.yaml")) as f:
        moveit_controllers = yaml.safe_load(f)

    move_group = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), moveit_controllers, use_sim_time],
            )
        ],
    )

    abb_control_dir = get_package_share_directory("abb_control")
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
                    os.path.join(pkg_dir, "config", "arm_control.yaml"),
                    os.path.join(pkg_dir, "config", "obstacles.yaml"),
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
        jtc_spawner,
        move_group,
        arm_control_node,
    ])
