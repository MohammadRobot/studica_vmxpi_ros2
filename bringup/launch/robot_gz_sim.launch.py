# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Primary Gazebo Sim launch for robot model, control, bridges, and optional tools."""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from _launch_helpers import (  # noqa: E402
    _append_env_path,
    _declare_arg,
    _expr_is_false,
    _expr_is_true,
    _sanitize_ld_library_path_for_rviz,
)
from _launch_gz import (  # noqa: E402
    _maybe_add_gz_sim_controller_spawners,
    _maybe_add_gz_sim_runtime_nodes,
    _maybe_include_gz_sim,
)
from _launch_sensors import (  # noqa: E402
    _maybe_include_camera,
    _maybe_include_gamepad,
    _maybe_include_lidar,
)


def generate_launch_description():
    declared_arguments = [
        _declare_arg(
            "gui",
            "false",
            "Start RViz2 automatically with this launch file.",
        ),
        _declare_arg(
            "rviz_start_delay",
            "10.0",
            "Delay before starting RViz2 (seconds). Lower for faster startup.",
        ),
        _declare_arg(
            "rviz_config_file",
            PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/robot/rviz", "robot.rviz"]
            ),
            "Absolute path to RViz config file.",
        ),
        _declare_arg(
            "use_hardware",
            "false",
            "Use Titan hardware instead of mock system.",
        ),
        _declare_arg("use_gz_sim", "true", "Start Gazebo Sim."),
        _declare_arg(
            "use_sim_time",
            LaunchConfiguration("use_gz_sim"),
            "Use simulation time (defaults to use_gz_sim).",
        ),
        _declare_arg(
            "use_ground_truth_odom_tf",
            "true",
            "In gz_sim, source /odom and /tf from Gazebo ground-truth odometry.",
        ),
        _declare_arg(
            "robot_profile",
            "training_4wd",
            "Robot profile under config/profiles (for example: training_4wd, training_2wd).",
        ),
        _declare_arg(
            "world",
            PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/gz/worlds", "diff_drive_world.sdf"]
            ),
            "Absolute path to Gazebo Sim world file.",
        ),
        _declare_arg("world_name", "default", "World name to spawn into."),
        _declare_arg(
            "gz_version",
            "8",
            "Gazebo Sim major version passed to ros_gz_sim (8 for Harmonic).",
        ),
        _declare_arg(
            "gz_headless",
            "false",
            "Run Gazebo Sim server only (no Gazebo GUI client).",
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
        _declare_arg("imu_gz_topic", "/imu", "Gazebo IMU topic to bridge into ROS /imu."),
        _declare_arg("spawn_x", "0.0", "Initial robot spawn x (meters)."),
        _declare_arg("spawn_y", "0.0", "Initial robot spawn y (meters)."),
        _declare_arg("spawn_z", "0.10", "Initial robot spawn z (meters)."),
        _declare_arg("spawn_yaw", "0.0", "Initial robot spawn yaw (radians)."),
        _declare_arg(
            "spawn_entity_name",
            "robot_system_position",
            "Entity name used when spawning the robot into Gazebo Sim.",
        ),
        _declare_arg(
            "controller_manager",
            "/controller_manager",
            "Preferred controller_manager service root for controller spawners.",
        ),
        _declare_arg(
            "use_joystick",
            "false",
            "Launch joystick teleop from studica_ros2_control.",
        ),
        _declare_arg(
            "use_lidar",
            LaunchConfiguration("use_hardware"),
            "Launch YDLIDAR in real hardware mode (defaults to use_hardware).",
        ),
        _declare_arg(
            "ydlidar_params_file",
            "",
            "Optional YDLIDAR params YAML. When set, this overrides lidar_type.",
        ),
        _declare_arg(
            "lidar_type",
            "tmini",
            "YDLIDAR model preset (example: tmini, x4, g4, gs2, sdm15). Used when ydlidar_params_file is empty.",
        ),
        _declare_arg(
            "lidar_parent_frame",
            "base_link",
            "Parent frame for LiDAR static transform.",
        ),
        _declare_arg(
            "lidar_child_frame",
            "laser_frame",
            "Child frame for LiDAR static transform.",
        ),
        _declare_arg("lidar_tf_x", "0.0", "LiDAR static TF translation X (meters)."),
        _declare_arg("lidar_tf_y", "0.0", "LiDAR static TF translation Y (meters)."),
        _declare_arg("lidar_tf_z", "0.02", "LiDAR static TF translation Z (meters)."),
        _declare_arg("lidar_tf_qx", "0.0", "LiDAR static TF quaternion X."),
        _declare_arg("lidar_tf_qy", "0.0", "LiDAR static TF quaternion Y."),
        _declare_arg(
            "lidar_tf_qz",
            "0.0",
            "LiDAR static TF quaternion Z.",
        ),
        _declare_arg("lidar_tf_qw", "1.0", "LiDAR static TF quaternion W."),
        _declare_arg(
            "use_camera",
            LaunchConfiguration("use_hardware"),
            "Launch Orbbec camera in real hardware mode (defaults to use_hardware).",
        ),
        _declare_arg(
            "orbbec_launch_file",
            "gemini_e.launch.py",
            "Orbbec launch file in orbbec_camera/launch.",
        ),
        _declare_arg(
            "orbbec_camera_name",
            "camera",
            "Orbbec camera_name launch argument (also sets namespace).",
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
            "orbbec_enable_color",
            "",
            "Optional override for Orbbec launch arg enable_color.",
        ),
        _declare_arg(
            "orbbec_enable_depth",
            "",
            "Optional override for Orbbec launch arg enable_depth.",
        ),
        _declare_arg(
            "orbbec_enable_ir",
            "",
            "Optional override for Orbbec launch arg enable_ir.",
        ),
        _declare_arg(
            "orbbec_color_width",
            "",
            "Optional override for Orbbec launch arg color_width.",
        ),
        _declare_arg(
            "orbbec_color_height",
            "",
            "Optional override for Orbbec launch arg color_height.",
        ),
        _declare_arg(
            "orbbec_color_fps",
            "",
            "Optional override for Orbbec launch arg color_fps.",
        ),
        _declare_arg(
            "orbbec_depth_width",
            "",
            "Optional override for Orbbec launch arg depth_width.",
        ),
        _declare_arg(
            "orbbec_depth_height",
            "",
            "Optional override for Orbbec launch arg depth_height.",
        ),
        _declare_arg(
            "orbbec_depth_fps",
            "",
            "Optional override for Orbbec launch arg depth_fps.",
        ),
        _declare_arg(
            "publish_camera_tf",
            "false",
            "Publish additional static TF from base frame to camera frame.",
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
        _declare_arg("camera_tf_x", "0.0", "Camera static TF translation X (meters)."),
        _declare_arg("camera_tf_y", "0.0", "Camera static TF translation Y (meters)."),
        _declare_arg("camera_tf_z", "0.0", "Camera static TF translation Z (meters)."),
        _declare_arg("camera_tf_qx", "0.0", "Camera static TF quaternion X."),
        _declare_arg("camera_tf_qy", "0.0", "Camera static TF quaternion Y."),
        _declare_arg("camera_tf_qz", "0.0", "Camera static TF quaternion Z."),
        _declare_arg("camera_tf_qw", "1.0", "Camera static TF quaternion W."),
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
        _declare_arg(
            "drive_controller_name",
            "robot_base_controller",
            "Primary drive controller name loaded by controller_manager.",
        ),
        _declare_arg(
            "drive_controller_type",
            "diff_drive_controller/DiffDriveController",
            "Primary drive controller plugin type.",
        ),
        _declare_arg(
            "drive_cmd_topic",
            "/robot_base_controller/cmd_vel",
            "Primary drive command topic.",
        ),
        _declare_arg(
            "drive_odom_topic",
            "/robot_base_controller/odom",
            "Primary drive odometry topic.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    use_hardware = LaunchConfiguration("use_hardware")
    use_gz_sim = LaunchConfiguration("use_gz_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    use_ground_truth_odom_tf = LaunchConfiguration("use_ground_truth_odom_tf")
    robot_profile = LaunchConfiguration("robot_profile")
    rviz_start_delay = LaunchConfiguration("rviz_start_delay")
    drive_controller_name = LaunchConfiguration("drive_controller_name")
    drive_controller_type = LaunchConfiguration("drive_controller_type")
    drive_cmd_topic = LaunchConfiguration("drive_cmd_topic")
    drive_odom_topic = LaunchConfiguration("drive_odom_topic")

    pkg_studica_vmxpi_ros2 = get_package_share_directory("studica_vmxpi_ros2")
    install_dir = get_package_prefix("studica_vmxpi_ros2")
    gz_models_path = os.path.join(pkg_studica_vmxpi_ros2, "description", "models")

    _append_env_path("GZ_SIM_RESOURCE_PATH", install_dir + "/share")
    _append_env_path("GZ_SIM_RESOURCE_PATH", gz_models_path)
    _append_env_path("GZ_SIM_SYSTEM_PLUGIN_PATH", install_dir + "/lib")

    print("GZ SIM RESOURCE PATH==" + str(os.environ["GZ_SIM_RESOURCE_PATH"]))
    print("GZ SIM SYSTEM PLUGINS PATH==" + str(os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"]))

    profile_file = PathJoinSubstitution(
        [FindPackageShare("studica_vmxpi_ros2"), "config", "profiles", robot_profile, "robot_profile.yaml"]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("studica_vmxpi_ros2"), "config", "profiles", robot_profile, "robot_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("studica_vmxpi_ros2"), "description/urdf", "robot.urdf.xacro"]),
            " ",
            "use_gz_sim:=", use_gz_sim,
            " ",
            "use_hardware:=", use_hardware,
            " ",
            "gz_version:=", LaunchConfiguration("gz_version"),
            " ",
            "sim_enable_camera:=", LaunchConfiguration("sim_enable_camera"),
            " ",
            "sim_camera_width:=", LaunchConfiguration("sim_camera_width"),
            " ",
            "sim_camera_height:=", LaunchConfiguration("sim_camera_height"),
            " ",
            "sim_camera_update_rate:=", LaunchConfiguration("sim_camera_update_rate"),
            " ",
            "sim_lidar_samples:=", LaunchConfiguration("sim_lidar_samples"),
            " ",
            "sim_lidar_update_rate:=", LaunchConfiguration("sim_lidar_update_rate"),
            " ",
            "sim_lidar_visualize:=", LaunchConfiguration("sim_lidar_visualize"),
            " ",
            "sim_imu_update_rate:=", LaunchConfiguration("sim_imu_update_rate"),
            " ",
            "profile_file:=", profile_file,
            " ",
            "controllers_file:=", robot_controllers,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": use_sim_time_param,
    }

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_env = {}
    sanitized_ld_library_path = _sanitize_ld_library_path_for_rviz()
    if sanitized_ld_library_path:
        rviz_env["LD_LIBRARY_PATH"] = sanitized_ld_library_path

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_gz_sim),
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
        condition=IfCondition(
            PythonExpression(
                _expr_is_true(use_hardware)
                + [" and "]
                + _expr_is_false(use_gz_sim)
            )
        ),
    )

    imu_alias_relay_node = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="imu_topic_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "enable_imu_relay": True,
                "imu_input_topic": "/imu_sensor_broadcaster/imu",
                "imu_output_topic": "/imu",
                "imu_use_odom_fallback": ParameterValue(use_gz_sim, value_type=bool),
                "imu_fallback_odom_topic": drive_odom_topic,
                "imu_fallback_frame_id": "imu_link",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                ["("]
                + _expr_is_true(use_gz_sim)
                + [") or ("]
                + _expr_is_true(use_hardware)
                + [" and "]
                + _expr_is_false(use_gz_sim)
                + [")"]
            )
        ),
    )

    control_api_bridge_node_gt = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="nav2_topic_bridge",
        output="screen",
        parameters=[
                {
                    "use_sim_time": use_sim_time_param,
                    "enable_nav2_bridge": True,
                    "input_cmd_vel_topic": "/cmd_vel",
                    "output_cmd_vel_topic": drive_cmd_topic,
                    "input_odom_topic": "/ground_truth/odom",
                    "output_odom_topic": "/odom",
                    "cmd_vel_frame_id": "base_link",
                }
            ],
        condition=IfCondition(
            PythonExpression(
                _expr_is_true(use_gz_sim)
                + [" and "]
                + _expr_is_true(use_ground_truth_odom_tf)
            )
        ),
    )

    control_api_bridge_node = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="nav2_topic_bridge_fallback",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "enable_nav2_bridge": True,
                "input_cmd_vel_topic": "/cmd_vel",
                "output_cmd_vel_topic": drive_cmd_topic,
                "input_odom_topic": drive_odom_topic,
                "output_odom_topic": "/odom",
                "cmd_vel_frame_id": "base_link",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                ["("]
                + _expr_is_true(use_hardware)
                + [") or ("]
                + _expr_is_true(use_gz_sim)
                + [" and "]
                + _expr_is_false(use_ground_truth_odom_tf)
                + [")"]
            )
        ),
    )

    drive_tf_relay_node_gt = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="ground_truth_tf_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "enable_tf_relay": True,
                "tf_input_topic": "/ground_truth/tf",
                "tf_output_topic": "/tf",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                _expr_is_true(use_gz_sim)
                + [" and "]
                + _expr_is_true(use_ground_truth_odom_tf)
            )
        ),
    )

    drive_tf_relay_node = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="drive_tf_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "enable_tf_relay": True,
                "tf_input_topic": PythonExpression(
                    ["'/' + '", drive_controller_name, "' + '/tf_odometry'"]
                ),
                "tf_output_topic": "/tf",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "(('",
                    drive_controller_type,
                    "').strip() == 'mecanum_drive_controller/MecanumDriveController') and not (",
                ]
                + _expr_is_true(use_gz_sim)
                + [" and "]
                + _expr_is_true(use_ground_truth_odom_tf)
                + [")"]
            )
        ),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[drive_controller_name, "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_gz_sim),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{"use_sim_time": use_sim_time_param}],
        arguments=["-d", rviz_config_file],
        additional_env=rviz_env,
        condition=IfCondition(gui),
    )
    rviz_node_delayed = TimerAction(
        # In Gazebo Sim, wheel/odom TF may appear only after ros2_control + broadcasters are active.
        # Delay RViz to avoid startup TF errors (left_wheel/right_wheel/caster -> odom/base_link).
        period=rviz_start_delay,
        actions=[rviz_node],
    )

    base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param}],
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
        ],
    )

    # Hardware-only control path (no simulation).
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="",
        parameters=[robot_description, robot_controllers, {"use_sim_time": use_sim_time_param}],
        output="screen",
        condition=UnlessCondition(use_gz_sim),
    )

    nodes = [
        SetEnvironmentVariable("LD_LIBRARY_PATH", sanitized_ld_library_path),
        LogInfo(msg=["Robot profile: ", robot_profile]),
        OpaqueFunction(function=_maybe_include_gz_sim),
        OpaqueFunction(function=_maybe_add_gz_sim_runtime_nodes),
        OpaqueFunction(function=_maybe_add_gz_sim_controller_spawners),
        control_node,
        node_robot_state_publisher,
        base_footprint_tf,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        imu_alias_relay_node,
        control_api_bridge_node_gt,
        control_api_bridge_node,
        drive_tf_relay_node_gt,
        drive_tf_relay_node,
        robot_controller_spawner,
        rviz_node_delayed,
        OpaqueFunction(function=_maybe_include_gamepad),
        OpaqueFunction(function=_maybe_include_lidar),
        OpaqueFunction(function=_maybe_include_camera),
    ]

    return LaunchDescription(declared_arguments + nodes)
