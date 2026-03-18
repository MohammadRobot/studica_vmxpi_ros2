# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Legacy Gazebo Classic mapping wrapper (SLAM + base bringup)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


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
            "use_gazebo_classic",
            default_value="true",
            description="Start Gazebo Classic simulation.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/gazebo/worlds", "diff_drive_world.world"]
            ),
            description="Absolute path to Gazebo world file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=LaunchConfiguration("use_gazebo_classic"),
            description="Use simulation time (defaults to use_gazebo_classic).",
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="false",
            description="Launch joystick teleop from studica_ros2_control.",
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
    use_gazebo_classic = LaunchConfiguration("use_gazebo_classic")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joystick = LaunchConfiguration("use_joystick")
    slam_params_file = LaunchConfiguration("slam_params_file")
    mode = PythonExpression(
        [
            "'gazebo_classic' if ('",
            use_gazebo_classic,
            "').lower() in ['true','1','yes','on'] else "
            "('hardware' if ('",
            use_hardware,
            "').lower() in ['true','1','yes','on'] else 'mock')",
        ]
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments={
            "mode": mode,
            "gui": gui,
            "world": world,
            "use_sim_time": use_sim_time,
            "use_joystick": use_joystick,
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
