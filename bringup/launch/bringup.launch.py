# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Unified runtime entry point for simulation, hardware, and mock bringup."""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from profile_validation import drive_topics, validate_profile_for_launch


def _declare_arg(name: str, default_value, description: str = ""):
    """Create a launch argument with optional description text."""
    if description:
        return DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description,
        )
    return DeclareLaunchArgument(name, default_value=default_value)


def _runtime_actions(context, *args, **kwargs):
    """Resolve runtime mode and forward normalized arguments to the selected launch."""
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    use_joystick = LaunchConfiguration("use_joystick").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    robot_profile = LaunchConfiguration("robot_profile").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip()
    use_lidar = LaunchConfiguration("use_lidar").perform(context).strip()
    use_camera = LaunchConfiguration("use_camera").perform(context).strip()
    use_ground_truth_odom_tf = LaunchConfiguration("use_ground_truth_odom_tf").perform(context).strip()
    world = LaunchConfiguration("world").perform(context).strip()
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context).strip()
    world_name = LaunchConfiguration("world_name").perform(context)
    gz_headless = LaunchConfiguration("gz_headless").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    spawn_entity_name = LaunchConfiguration("spawn_entity_name").perform(context)
    sim_enable_camera = LaunchConfiguration("sim_enable_camera").perform(context)
    sim_camera_width = LaunchConfiguration("sim_camera_width").perform(context)
    sim_camera_height = LaunchConfiguration("sim_camera_height").perform(context)
    sim_camera_update_rate = LaunchConfiguration("sim_camera_update_rate").perform(context)
    sim_lidar_samples = LaunchConfiguration("sim_lidar_samples").perform(context)
    sim_lidar_update_rate = LaunchConfiguration("sim_lidar_update_rate").perform(context)
    sim_lidar_visualize = LaunchConfiguration("sim_lidar_visualize").perform(context)
    sim_imu_update_rate = LaunchConfiguration("sim_imu_update_rate").perform(context)
    ydlidar_params_file = LaunchConfiguration("ydlidar_params_file").perform(context)
    orbbec_launch_file = LaunchConfiguration("orbbec_launch_file").perform(context)
    orbbec_camera_name = LaunchConfiguration("orbbec_camera_name").perform(context)
    orbbec_serial_number = LaunchConfiguration("orbbec_serial_number").perform(context)
    orbbec_enable_point_cloud = LaunchConfiguration("orbbec_enable_point_cloud").perform(context)
    publish_camera_tf = LaunchConfiguration("publish_camera_tf").perform(context)
    camera_parent_frame = LaunchConfiguration("camera_parent_frame").perform(context)
    camera_child_frame = LaunchConfiguration("camera_child_frame").perform(context)
    camera_tf_x = LaunchConfiguration("camera_tf_x").perform(context)
    camera_tf_y = LaunchConfiguration("camera_tf_y").perform(context)
    camera_tf_z = LaunchConfiguration("camera_tf_z").perform(context)
    camera_tf_qx = LaunchConfiguration("camera_tf_qx").perform(context)
    camera_tf_qy = LaunchConfiguration("camera_tf_qy").perform(context)
    camera_tf_qz = LaunchConfiguration("camera_tf_qz").perform(context)
    camera_tf_qw = LaunchConfiguration("camera_tf_qw").perform(context)
    joystick_cmd_vel_topic = LaunchConfiguration("joystick_cmd_vel_topic").perform(context)
    joystick_publish_stamped = LaunchConfiguration("joystick_publish_stamped").perform(context)

    pkg_share = get_package_share_directory("studica_vmxpi_ros2")
    if not rviz_config_file:
        # Keep a safe default when wrappers don't pass an explicit RViz config.
        rviz_config_file = os.path.join(pkg_share, "description", "robot", "rviz", "robot.rviz")
    (
        _profile_file,
        _controllers_file,
        drive_controller_name,
        drive_controller_type,
        drive_wheel_layout,
    ) = validate_profile_for_launch(pkg_share, robot_profile)
    drive_cmd_topic, drive_odom_topic = drive_topics(
        drive_controller_name, drive_controller_type
    )
    if not world:
        if mode == "gazebo_classic":
            world = os.path.join(pkg_share, "description", "gazebo", "worlds", "diff_drive_world.world")
        else:
            world = os.path.join(pkg_share, "description", "gz", "worlds", "diff_drive_world.sdf")

    if not use_sim_time:
        use_sim_time = "true" if mode == "gz_sim" else "false"
    if not use_lidar:
        use_lidar = "true" if mode == "hardware" else "false"
    if not use_camera:
        use_camera = "true" if mode == "hardware" else "false"

    if mode == "gazebo_classic":
        if drive_wheel_layout not in ("diff", "diff_4wd"):
            raise RuntimeError(
                "gazebo_classic mode currently supports only diff or diff_4wd wheel layouts."
            )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "robot_gazebo_classic.launch.py")
                ),
                launch_arguments={
                    "gui": gui,
                    "use_hardware": "false",
                    "use_gazebo_classic": "true",
                    "use_sim_time": use_sim_time,
                    "world": world,
                    "use_joystick": use_joystick,
                    "robot_profile": robot_profile,
                }.items(),
            )
        ]

    if mode not in ("gz_sim", "hardware", "mock"):
        raise RuntimeError("Invalid mode. Use one of: gz_sim, hardware, mock, gazebo_classic.")

    use_gz_sim = "true" if mode == "gz_sim" else "false"
    use_hardware = "true" if mode == "hardware" else "false"
    robot_launch_args = {
        "gui": gui,
        "use_hardware": use_hardware,
        "use_gz_sim": use_gz_sim,
        "use_sim_time": use_sim_time,
        "use_ground_truth_odom_tf": use_ground_truth_odom_tf,
        "world": world,
        "world_name": world_name,
        "gz_headless": gz_headless,
        "spawn_x": spawn_x,
        "spawn_y": spawn_y,
        "spawn_z": spawn_z,
        "spawn_yaw": spawn_yaw,
        "spawn_entity_name": spawn_entity_name,
        "sim_enable_camera": sim_enable_camera,
        "sim_camera_width": sim_camera_width,
        "sim_camera_height": sim_camera_height,
        "sim_camera_update_rate": sim_camera_update_rate,
        "sim_lidar_samples": sim_lidar_samples,
        "sim_lidar_update_rate": sim_lidar_update_rate,
        "sim_lidar_visualize": sim_lidar_visualize,
        "sim_imu_update_rate": sim_imu_update_rate,
        "use_joystick": use_joystick,
        "use_lidar": use_lidar,
        "ydlidar_params_file": ydlidar_params_file,
        "use_camera": use_camera,
        "orbbec_launch_file": orbbec_launch_file,
        "orbbec_camera_name": orbbec_camera_name,
        "orbbec_serial_number": orbbec_serial_number,
        "orbbec_enable_point_cloud": orbbec_enable_point_cloud,
        "publish_camera_tf": publish_camera_tf,
        "camera_parent_frame": camera_parent_frame,
        "camera_child_frame": camera_child_frame,
        "camera_tf_x": camera_tf_x,
        "camera_tf_y": camera_tf_y,
        "camera_tf_z": camera_tf_z,
        "camera_tf_qx": camera_tf_qx,
        "camera_tf_qy": camera_tf_qy,
        "camera_tf_qz": camera_tf_qz,
        "camera_tf_qw": camera_tf_qw,
        "joystick_cmd_vel_topic": joystick_cmd_vel_topic,
        "joystick_publish_stamped": joystick_publish_stamped,
        "drive_controller_name": drive_controller_name,
        "drive_controller_type": drive_controller_type,
        "drive_cmd_topic": drive_cmd_topic,
        "drive_odom_topic": drive_odom_topic,
        "rviz_config_file": rviz_config_file,
        "robot_profile": robot_profile,
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "robot_gz_sim.launch.py")
            ),
            launch_arguments=robot_launch_args.items(),
            )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            _declare_arg(
                "mode",
                "gz_sim",
                "Runtime mode: gz_sim | hardware | mock | gazebo_classic",
            ),
            _declare_arg(
                "gui",
                "false",
                "Start RViz2 automatically.",
            ),
            _declare_arg(
                "rviz_config_file",
                PathJoinSubstitution(
                    [FindPackageShare("studica_vmxpi_ros2"), "description/robot/rviz", "robot.rviz"]
                ),
                "Absolute path to RViz config file.",
            ),
            _declare_arg(
                "robot_profile",
                "training_4wd",
                "Robot profile under config/profiles.",
            ),
            _declare_arg(
                "use_sim_time",
                "",
                "Leave empty to auto-select by mode.",
            ),
            _declare_arg(
                "use_ground_truth_odom_tf",
                "true",
                "In gz_sim, source /odom and /tf from Gazebo odometry topics.",
            ),
            _declare_arg(
                "world",
                "",
                "World file path. Leave empty to use mode default world.",
            ),
            _declare_arg(
                "world_name",
                "default",
                "World name used by Gazebo Sim service calls.",
            ),
            _declare_arg(
                "gz_headless",
                "false",
                "Run Gazebo Sim server only (no Gazebo GUI client).",
            ),
            _declare_arg(
                "spawn_x",
                "0.0",
                "Initial robot spawn x position (meters).",
            ),
            _declare_arg(
                "spawn_y",
                "0.0",
                "Initial robot spawn y position (meters).",
            ),
            _declare_arg(
                "spawn_z",
                "0.10",
                "Initial robot spawn z position (meters).",
            ),
            _declare_arg(
                "spawn_yaw",
                "0.0",
                "Initial robot spawn yaw (radians).",
            ),
            _declare_arg(
                "spawn_entity_name",
                "robot_system_position",
                "Robot entity name used when spawning into Gazebo Sim.",
            ),
            _declare_arg(
                "sim_enable_camera",
                "true",
                "Enable simulated RGB + depth camera sensors in gz_sim.",
            ),
            _declare_arg(
                "sim_camera_width",
                "640",
                "Sim camera image width in pixels.",
            ),
            _declare_arg(
                "sim_camera_height",
                "480",
                "Sim camera image height in pixels.",
            ),
            _declare_arg(
                "sim_camera_update_rate",
                "30.0",
                "Sim camera update rate (Hz) for color and depth streams.",
            ),
            _declare_arg(
                "sim_lidar_samples",
                "200",
                "Sim lidar horizontal sample count.",
            ),
            _declare_arg(
                "sim_lidar_update_rate",
                "20.0",
                "Sim lidar update rate (Hz).",
            ),
            _declare_arg(
                "sim_lidar_visualize",
                "true",
                "Enable Gazebo visualization for lidar rays.",
            ),
            _declare_arg(
                "sim_imu_update_rate",
                "100.0",
                "Sim IMU update rate (Hz).",
            ),
            _declare_arg(
                "use_joystick",
                "false",
                "Launch joystick teleop from studica_ros2_control.",
            ),
            _declare_arg(
                "use_lidar",
                "",
                "Leave empty to auto-select (true in hardware mode).",
            ),
            _declare_arg(
                "ydlidar_params_file",
                "",
                "Optional YDLIDAR params YAML file (hardware mode only; empty uses ydlidar_ros2_driver/params/Tmini.yaml).",
            ),
            _declare_arg(
                "use_camera",
                "",
                "Leave empty to auto-select (true in hardware mode).",
            ),
            _declare_arg(
                "orbbec_launch_file",
                "gemini_e.launch.py",
                "Orbbec launch file in orbbec_camera/launch (hardware mode only).",
            ),
            _declare_arg(
                "orbbec_camera_name",
                "camera",
                "Orbbec camera_name launch argument.",
            ),
            _declare_arg(
                "orbbec_serial_number",
                "",
                "Optional Orbbec serial number for selecting a specific device.",
            ),
            _declare_arg(
                "orbbec_enable_point_cloud",
                "false",
                "Enable Orbbec point cloud output.",
            ),
            _declare_arg(
                "publish_camera_tf",
                "false",
                "Publish additional static TF from base_link to camera frame.",
            ),
            _declare_arg(
                "camera_parent_frame",
                "base_link",
                "Parent frame for camera static transform.",
            ),
            _declare_arg(
                "camera_child_frame",
                "",
                "Child frame for camera static transform (empty => <orbbec_camera_name>_link).",
            ),
            _declare_arg(
                "camera_tf_x",
                "0.0",
                "Camera static TF translation X (meters).",
            ),
            _declare_arg(
                "camera_tf_y",
                "0.0",
                "Camera static TF translation Y (meters).",
            ),
            _declare_arg(
                "camera_tf_z",
                "0.0",
                "Camera static TF translation Z (meters).",
            ),
            _declare_arg(
                "camera_tf_qx",
                "0.0",
                "Camera static TF quaternion X.",
            ),
            _declare_arg(
                "camera_tf_qy",
                "0.0",
                "Camera static TF quaternion Y.",
            ),
            _declare_arg(
                "camera_tf_qz",
                "0.0",
                "Camera static TF quaternion Z.",
            ),
            _declare_arg(
                "camera_tf_qw",
                "1.0",
                "Camera static TF quaternion W.",
            ),
            _declare_arg(
                "joystick_cmd_vel_topic",
                "",
                "Joystick command velocity output topic (empty = auto from drive profile).",
            ),
            _declare_arg(
                "joystick_publish_stamped",
                "true",
                "Publish TwistStamped joystick commands.",
            ),
            OpaqueFunction(function=_runtime_actions),
        ]
    )
