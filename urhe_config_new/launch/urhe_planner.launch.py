from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(
        "ur5e_he", package_name="urhe_config_new"
    ).to_moveit_configs()

    # Trajectory execution overrides
    trajectory_execution_params = {
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        "trajectory_execution.allowed_goal_duration_margin": 15.0,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    # UR driver launch file path
    ur_driver_launch = os.path.join(
        get_package_share_directory("ur_robot_driver"),
        "launch",
        "ur_control.launch.py"
    )

    return LaunchDescription([

        # ── UR driver (NO RViz) ──────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_driver_launch),
            launch_arguments={
                "ur_type": "ur5e",
                "robot_ip": "192.168.1.100",
                "launch_rviz": "false",
            }.items(),
        ),

        # ── move_group ───────────────────────────────────────────────
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.trajectory_execution,
                moveit_config.planning_scene_monitor,
                moveit_config.joint_limits,
                {"use_sim_time": False},
                trajectory_execution_params,
            ],
        ),

        # ── RViz ─────────────────────────────────────────────────────
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d", str(moveit_config.package_path / "config/moveit.rviz")
            ],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": False},
            ],
        ),

        # ── Hand-E bridge ────────────────────────────────────────────
        Node(
            package="hande_bridge",
            executable="robotiq_hande_bridge",
            output="screen",
            parameters=[{"gripper_ip": "192.168.1.100"}],
        ),

    ])