# urhe_config_new/launch/moveit_only.launch.py
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur5e_he", package_name="urhe_config_new"
    ).to_moveit_configs()

    # Trajectory execution overrides — passed to move_group only.
    # Disables the 0.5s timeout so MoveIt waits for the TCP gripper to finish.
    trajectory_execution_params = {
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        "trajectory_execution.allowed_goal_duration_margin": 15.0,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    return LaunchDescription([

        # ── move_group ────────────────────────────────────────────────────
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
                trajectory_execution_params,   # ← overrides go here only
            ],
        ),

        # ── RViz ─────────────────────────────────────────────────────────
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
                # NOTE: trajectory_execution_params intentionally NOT here
            ],
        ),
    ])