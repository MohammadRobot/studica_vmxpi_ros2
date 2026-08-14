# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Supervised stack_4wd hardware launch with conservative sensor settings."""

from launch import LaunchDescription
import sys
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from _classroom_launch import deferred_include  # noqa: E402


def generate_launch_description():
    robot_profile = LaunchConfiguration("robot_profile")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    use_camera_color = LaunchConfiguration("use_camera_color")
    use_colored_depth_cloud = LaunchConfiguration("use_colored_depth_cloud")
    use_point_cloud = LaunchConfiguration("use_point_cloud")
    use_point_cloud_filter = LaunchConfiguration("use_point_cloud_filter")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_joystick = LaunchConfiguration("use_joystick")
    use_imu_odometry = LaunchConfiguration("use_imu_odometry")
    hardware_control_rate_hz = LaunchConfiguration("hardware_control_rate_hz")

    arguments = [
        DeclareLaunchArgument(
            "robot_profile",
            default_value="stack_4wd",
            description="Physical robot profile under config/profiles.",
        ),
        DeclareLaunchArgument(
            "use_lidar",
            default_value="true",
            description="Start the YDLidar driver.",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Start the Orbbec camera with the low-load depth profile.",
        ),
        DeclareLaunchArgument(
            "use_camera_color",
            default_value="false",
            description="Also start 640x480 color at 15 Hz; disabled for thermal headroom.",
        ),
        DeclareLaunchArgument(
            "use_colored_depth_cloud",
            default_value="false",
            description=(
                "Start registered 320x240 color and depth at 15 Hz for the "
                "RViz DepthCloud display."
            ),
        ),
        DeclareLaunchArgument(
            "use_point_cloud",
            default_value="false",
            description="Publish raw 320x240 depth points at 5 Hz.",
        ),
        DeclareLaunchArgument(
            "use_point_cloud_filter",
            default_value="false",
            description=(
                "Also publish floor/body-filtered depth points when "
                "use_point_cloud is true."
            ),
        ),
        DeclareLaunchArgument(
            "use_foxglove",
            default_value="true",
            description="Start the read-only Foxglove bridge.",
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="false",
            description="Start joystick teleop only when the controller is attached to the VMXPi.",
        ),
        DeclareLaunchArgument(
            "hardware_control_rate_hz",
            default_value="25",
            description="ros2_control update and odometry publication rate in hardware mode.",
        ),
        DeclareLaunchArgument(
            "use_imu_odometry",
            default_value="true",
            description="Fuse encoder forward velocity with IMU yaw and publish the hardware odom TF.",
        ),
        DeclareLaunchArgument(
            "foxglove_address",
            default_value="127.0.0.1",
            description="Foxglove bind address; supply a trusted LAN address for remote access.",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove WebSocket port.",
        ),
    ]

    robot = deferred_include(
        "studica_vmxpi_ros2",
        "bringup.launch.py",
        {
            "mode": "hardware",
            "robot_profile": robot_profile,
            "gui": "false",
            "use_lidar": use_lidar,
            "use_camera": PythonExpression(
                [
                    "'",
                    use_camera,
                    "'.lower() == 'true' or '",
                    use_point_cloud,
                    "'.lower() == 'true' or '",
                    use_colored_depth_cloud,
                    "'.lower() == 'true'",
                ]
            ),
            "use_monitoring": "true",
            "use_foxglove": use_foxglove,
            "use_joystick": use_joystick,
            "hardware_control_rate_hz": hardware_control_rate_hz,
            "use_imu_odometry": use_imu_odometry,
            "foxglove_address": foxglove_address,
            "foxglove_port": foxglove_port,
            "orbbec_enable_point_cloud": use_point_cloud,
            "orbbec_enable_color": PythonExpression(
                [
                    "'true' if '",
                    use_colored_depth_cloud,
                    "'.lower() == 'true' else '",
                    use_camera_color,
                    "'",
                ]
            ),
            "orbbec_enable_depth": "true",
            "orbbec_enable_ir": "false",
            "orbbec_depth_registration": use_colored_depth_cloud,
            "orbbec_color_width": PythonExpression(
                ["'320' if '", use_colored_depth_cloud, "'.lower() == 'true' else '640'"]
            ),
            "orbbec_color_height": PythonExpression(
                ["'240' if '", use_colored_depth_cloud, "'.lower() == 'true' else '480'"]
            ),
            "orbbec_color_fps": "15",
            "orbbec_depth_width": "320",
            "orbbec_depth_height": "240",
            "orbbec_depth_fps": PythonExpression(
                ["'15' if '", use_colored_depth_cloud, "'.lower() == 'true' else '5'"]
            ),
        },
    )

    point_cloud_filter = Node(
        package="studica_vmxpi_ros2",
        executable="point_cloud_filter.py",
        name="point_cloud_filter",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "input_topic": "/camera/depth/points",
                "output_topic": "/camera/depth/points_filtered",
                "target_frame": "base_link",
                "stride": 2,
                "min_height_m": 0.06,
                "max_height_m": 1.50,
                "min_forward_m": 0.15,
                "max_forward_m": 3.00,
                "max_lateral_m": 2.00,
                "robot_min_x_m": -0.18,
                "robot_max_x_m": 0.18,
                "robot_half_width_m": 0.1975,
                "robot_max_height_m": 0.35,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_point_cloud,
                    "'.lower() == 'true' and '",
                    use_point_cloud_filter,
                    "'.lower() == 'true'",
                ]
            )
        ),
    )

    return LaunchDescription(arguments + [robot, point_cloud_filter])
