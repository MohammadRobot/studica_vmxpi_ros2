# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Gazebo Sim mapping wrapper (SLAM + unified bringup)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _runtime_mode_expression(use_gz_sim, use_hardware):
    # Keep mode resolution in one place so wrappers stay easy to read.
    return PythonExpression(
        [
            "'gz_sim' if ('",
            use_gz_sim,
            "').lower() in ['true','1','yes','on'] else "
            "('hardware' if ('",
            use_hardware,
            "').lower() in ['true','1','yes','on'] else 'mock')",
        ]
    )


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 from robot launch.",
        ),
        DeclareLaunchArgument(
            "robot_profile",
            default_value="training_4wd",
            description="Robot profile under config/profiles.",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Use Titan hardware instead of mock system.",
        ),
        DeclareLaunchArgument(
            "use_gz_sim",
            default_value="true",
            description="Start Gazebo Sim simulation.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/gz/worlds", "diff_drive_world.sdf"]
            ),
            description="Absolute path to Gazebo Sim world file.",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="default",
            description="World name to spawn into.",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
            description="Initial robot spawn x (meters).",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Initial robot spawn y (meters).",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.10",
            description="Initial robot spawn z (meters).",
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Initial robot spawn yaw (radians).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=LaunchConfiguration("use_gz_sim"),
            description="Use simulation time (defaults to use_gz_sim).",
        ),
        DeclareLaunchArgument(
            "use_ground_truth_odom_tf",
            default_value="false",
            description=(
                "In gz_sim, source /odom and /tf from Gazebo ground-truth odometry topics. "
                "Set false to use controller odometry TF."
            ),
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="false",
            description="Launch joystick teleop from studica_ros2_control.",
        ),
        DeclareLaunchArgument(
            "joystick_cmd_vel_topic",
            default_value="",
            description="Joystick command velocity output topic (empty = auto from drive profile).",
        ),
        DeclareLaunchArgument(
            "joystick_publish_stamped",
            default_value="true",
            description="Publish TwistStamped joystick commands.",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "config", "slam_toolbox_mapper_params.yaml"]
            ),
            description="SLAM Toolbox parameter file.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    robot_profile = LaunchConfiguration("robot_profile")
    use_hardware = LaunchConfiguration("use_hardware")
    use_gz_sim = LaunchConfiguration("use_gz_sim")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ground_truth_odom_tf = LaunchConfiguration("use_ground_truth_odom_tf")
    use_joystick = LaunchConfiguration("use_joystick")
    joystick_cmd_vel_topic = LaunchConfiguration("joystick_cmd_vel_topic")
    joystick_publish_stamped = LaunchConfiguration("joystick_publish_stamped")
    slam_params_file = LaunchConfiguration("slam_params_file")
    mode = _runtime_mode_expression(use_gz_sim, use_hardware)

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("studica_vmxpi_ros2"), "launch", "bringup.launch.py"])
        ),
        launch_arguments={
            "mode": mode,
            "gui": gui,
            "world": world,
            "world_name": world_name,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
            "use_sim_time": use_sim_time,
            "use_ground_truth_odom_tf": use_ground_truth_odom_tf,
            "use_joystick": use_joystick,
            "joystick_cmd_vel_topic": joystick_cmd_vel_topic,
            "joystick_publish_stamped": joystick_publish_stamped,
            "robot_profile": robot_profile,
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [robot, slam])
