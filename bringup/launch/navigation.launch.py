# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Nav2 in local simulation or against a separately launched VMXPi."""

from launch import LaunchDescription
import sys
from pathlib import Path

from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from _classroom_launch import deferred_include  # noqa: E402
from _nav2_perception import Nav2PointCloudParams  # noqa: E402


def generate_launch_description():
    autostart = LaunchConfiguration("autostart")
    gui = LaunchConfiguration("gui")
    gz_headless = LaunchConfiguration("gz_headless")
    hardware_max_angular_speed = LaunchConfiguration(
        "hardware_max_angular_speed"
    )
    hardware_max_linear_speed = LaunchConfiguration("hardware_max_linear_speed")
    mode = LaunchConfiguration("mode")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_point_cloud = LaunchConfiguration("use_point_cloud")
    use_joystick = LaunchConfiguration("use_joystick")
    use_sim_time = PythonExpression(
        ["'true' if '", mode, "' == 'gz_sim' else 'false'"]
    )
    hardware_mode = PythonExpression(
        ["'true' if '", mode, "' == 'hardware' else 'false'"]
    )
    simulation_map_file = PathJoinSubstitution(
        [FindPackageShare("studica_vmxpi_ros2"), "maps", "office_map.yaml"]
    )
    hardware_map_file = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "studica_ws",
            "project_maps",
            "real_robot_map.yaml",
        ]
    )
    default_map_file = PythonExpression(
        [
            "'",
            hardware_map_file,
            "' if '",
            mode,
            "' == 'hardware' else '",
            simulation_map_file,
            "'",
        ]
    )
    simulation_point_cloud = IfCondition(
        PythonExpression(
            ["'", mode, "' == 'gz_sim' and '", use_point_cloud, "' == 'true'"]
        )
    )
    robot_profile_file = PathJoinSubstitution(
        [
            FindPackageShare("studica_vmxpi_ros2"),
            "config",
            "profiles",
            "class_4wd",
            "robot_profile.yaml",
        ]
    )

    arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="gz_sim",
            choices=["gz_sim", "hardware"],
            description=(
                "Use gz_sim for the local office world or hardware for the PC-side "
                "Nav2 client of a separately launched VMXPi."
            ),
        ),
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2."),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Run Gazebo without its graphical window.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map_file,
            description=(
                "Map YAML used by AMCL; hardware defaults to "
                "~/studica_ws/project_maps/real_robot_map.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "params", "nav2_params.yaml"]
            ),
            description="Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate Nav2 lifecycle nodes.",
        ),
        DeclareLaunchArgument(
            "hardware_max_linear_speed",
            default_value="0.20",
            description=(
                "Maximum physical Nav2 linear speed in m/s; validated range is "
                "greater than 0 through 0.30."
            ),
        ),
        DeclareLaunchArgument(
            "hardware_max_angular_speed",
            default_value="0.35",
            description=(
                "Maximum physical Nav2 angular speed in rad/s; validated range is "
                "greater than 0 through 0.60."
            ),
        ),
        DeclareLaunchArgument(
            "use_point_cloud",
            default_value=PythonExpression(
                ["'false' if '", mode, "' == 'hardware' else 'true'"]
            ),
            description=(
                "Use filtered depth points to mark and raw depth rays to clear "
                "the Nav2 local costmap; disabled by default on hardware."
            ),
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="false",
            description=(
                "Start simulation joystick teleop; ignored in hardware mode and "
                "keep false while Nav2 controls the robot."
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("studica_vmxpi_ros2"),
                    "description",
                    "robot",
                    "rviz",
                    "nav2_navigation.rviz",
                ]
            ),
            description="RViz configuration used by hardware mode.",
        ),
    ]

    robot = deferred_include(
        "studica_vmxpi_ros2",
        "bringup.launch.py",
        {
            "mode": "gz_sim",
            "robot_profile": "class_4wd",
            "world": "office_map",
            "gui": gui,
            "rviz_config_file": PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description", "robot", "rviz", "nav2_navigation.rviz"]
            ),
            "gz_headless": gz_headless,
            "use_ground_truth_odom_tf": "false",
            "use_lidar": "true",
            "sim_enable_camera": use_point_cloud,
            "sim_camera_width": "640",
            "sim_camera_height": "480",
            "sim_camera_update_rate": "10.0",
            "sim_lidar_samples": "180",
            "sim_lidar_update_rate": "10.0",
            "sim_lidar_visualize": "false",
            "sim_imu_update_rate": "50.0",
            "use_monitoring": "true",
            "use_foxglove": "false",
            "use_joystick": use_joystick,
        },
    )
    simulation = GroupAction(
        actions=[robot],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'gz_sim'"])),
    )

    hardware_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file, "-f", "map"],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(gui),
    )
    hardware_client = GroupAction(
        actions=[hardware_rviz],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'hardware'"])),
    )

    point_cloud = Node(
        package="studica_vmxpi_ros2",
        executable="depth_to_pointcloud.py",
        name="depth_to_pointcloud",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "stride": 4,
                "min_depth_m": 0.1,
                "max_depth_m": 10.0,
            }
        ],
        condition=simulation_point_cloud,
    )

    point_cloud_filter = Node(
        package="studica_vmxpi_ros2",
        executable="point_cloud_filter.py",
        name="point_cloud_filter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/camera/depth/points",
                "output_topic": "/camera/depth/points_filtered",
                "target_frame": "base_link",
                "min_height_m": 0.04,
                "max_height_m": 1.50,
                "min_forward_m": 0.15,
                "max_forward_m": 3.00,
                "max_lateral_m": 2.00,
                # Five-millimetre self-filter padding around the measured
                # 0.350 x 0.385 m outer envelope.
                "robot_min_x_m": -0.18,
                "robot_max_x_m": 0.18,
                "robot_half_width_m": 0.1975,
                "robot_max_height_m": 0.35,
            }
        ],
        condition=simulation_point_cloud,
    )

    nav2 = deferred_include(
        "nav2_bringup",
        "bringup_launch.py",
        {
            "slam": "False",
            "map": map_file,
            "params_file": Nav2PointCloudParams(
                source_file=nav2_params_file,
                enabled=use_point_cloud,
                profile_file=robot_profile_file,
                hardware_mode=hardware_mode,
                hardware_max_linear_speed=hardware_max_linear_speed,
                hardware_max_angular_speed=hardware_max_angular_speed,
            ),
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_composition": "False",
        },
    )

    return LaunchDescription(
        arguments + [simulation, hardware_client, point_cloud, point_cloud_filter, nav2]
    )
