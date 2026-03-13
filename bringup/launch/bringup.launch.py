import os
import re
from numbers import Real

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml

SUPPORTED_WHEEL_LAYOUTS = ("diff", "diff_4wd", "mecanum", "omni")
SUPPORTED_DRIVE_CONTROLLER_TYPES = (
    "diff_drive_controller/DiffDriveController",
    "mecanum_drive_controller/MecanumDriveController",
)


def _is_true(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")


def _load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root must be a mapping: {path}")
    return data


def _drive_topics(controller_name: str, controller_type: str):
    if controller_type == "mecanum_drive_controller/MecanumDriveController":
        return f"/{controller_name}/reference", f"/{controller_name}/odometry"
    return f"/{controller_name}/cmd_vel", f"/{controller_name}/odom"


def _validate_profile_schema(profile_name: str):
    if not re.fullmatch(r"[A-Za-z0-9_-]+", profile_name):
        raise RuntimeError(
            f"Invalid robot_profile '{profile_name}'. Use letters, numbers, '_' or '-'."
        )

    pkg_share = get_package_share_directory("studica_vmxpi_ros2")
    profile_dir = os.path.join(pkg_share, "config", "profiles", profile_name)
    profile_file = os.path.join(profile_dir, "robot_profile.yaml")
    controllers_file = os.path.join(profile_dir, "robot_controllers.yaml")

    if not os.path.exists(profile_file):
        raise RuntimeError(f"Missing profile file: {profile_file}")
    if not os.path.exists(controllers_file):
        raise RuntimeError(f"Missing controllers file: {controllers_file}")

    profile = _load_yaml(profile_file)
    controllers = _load_yaml(controllers_file)

    if "xacro" not in profile or not isinstance(profile["xacro"], dict):
        raise RuntimeError(f"{profile_file}: missing 'xacro' mapping.")
    if "hardware" not in profile or not isinstance(profile["hardware"], dict):
        raise RuntimeError(f"{profile_file}: missing 'hardware' mapping.")

    xacro_cfg = profile["xacro"]
    hw_cfg = profile["hardware"]
    drive_cfg_raw = profile.get("drive")
    drive_cfg = drive_cfg_raw if isinstance(drive_cfg_raw, dict) else {}

    if drive_cfg_raw is not None and not isinstance(drive_cfg_raw, dict):
        raise RuntimeError(f"{profile_file}: 'drive' must be a mapping.")

    drive_wheel_layout = str(drive_cfg.get("wheel_layout", "diff")).strip().lower()
    drive_controller_name = str(
        drive_cfg.get("controller_name", "robot_base_controller")
    ).strip()
    drive_controller_type = str(
        drive_cfg.get("controller_type", "diff_drive_controller/DiffDriveController")
    ).strip()

    if drive_wheel_layout not in SUPPORTED_WHEEL_LAYOUTS:
        raise RuntimeError(
            f"{profile_file}: drive.wheel_layout must be one of "
            f"{SUPPORTED_WHEEL_LAYOUTS}."
        )
    if not drive_controller_name:
        raise RuntimeError(f"{profile_file}: drive.controller_name cannot be empty.")
    if drive_controller_type not in SUPPORTED_DRIVE_CONTROLLER_TYPES:
        raise RuntimeError(
            f"{profile_file}: drive.controller_type must be one of "
            f"{SUPPORTED_DRIVE_CONTROLLER_TYPES}."
        )
    if (
        drive_wheel_layout in ("diff", "diff_4wd")
        and drive_controller_type != "diff_drive_controller/DiffDriveController"
    ):
        raise RuntimeError(
            f"{profile_file}: {drive_wheel_layout} layout requires diff_drive_controller."
        )
    if (
        drive_wheel_layout in ("mecanum", "omni")
        and drive_controller_type != "mecanum_drive_controller/MecanumDriveController"
    ):
        raise RuntimeError(
            f"{profile_file}: {drive_wheel_layout} layout requires mecanum_drive_controller."
        )

    required_xacro = (
        "base_mass",
        "base_width",
        "base_length",
        "base_height",
        "wheel_mass",
        "wheel_len",
        "wheel_radius",
        "caster_wheel_mass",
        "caster_wheel_radius",
        "laser_model_z",
        "laser_frame_z",
    )
    required_hw = (
        "can_id",
        "motor_freq",
        "ticks_per_rotation",
        "wheel_radius",
        "speed_scale",
        "max_wheel_angular_velocity_rad_s",
        "left_front_motor",
        "left_rear_motor",
        "right_front_motor",
        "right_rear_motor",
        "invert_left_front_motor",
        "invert_left_rear_motor",
        "invert_right_front_motor",
        "invert_right_rear_motor",
        "invert_left_front_encoder",
        "invert_left_rear_encoder",
        "invert_right_front_encoder",
        "invert_right_rear_encoder",
    )

    for key in required_xacro:
        if key not in xacro_cfg:
            raise RuntimeError(f"{profile_file}: missing xacro key '{key}'.")
        if not isinstance(xacro_cfg[key], Real):
            raise RuntimeError(f"{profile_file}: xacro key '{key}' must be numeric.")

    for key in required_hw:
        if key not in hw_cfg:
            raise RuntimeError(f"{profile_file}: missing hardware key '{key}'.")

    numeric_hw_keys = (
        "can_id",
        "motor_freq",
        "ticks_per_rotation",
        "wheel_radius",
        "speed_scale",
        "max_wheel_angular_velocity_rad_s",
        "left_front_motor",
        "left_rear_motor",
        "right_front_motor",
        "right_rear_motor",
    )
    for key in numeric_hw_keys:
        if not isinstance(hw_cfg[key], Real):
            raise RuntimeError(f"{profile_file}: hardware key '{key}' must be numeric.")

    bool_hw_keys = tuple(k for k in required_hw if k.startswith("invert_"))
    for key in bool_hw_keys:
        if not isinstance(hw_cfg[key], bool):
            raise RuntimeError(f"{profile_file}: hardware key '{key}' must be bool.")

    can_id = int(hw_cfg["can_id"])
    motor_freq = int(hw_cfg["motor_freq"])
    ticks_per_rotation = int(hw_cfg["ticks_per_rotation"])
    speed_scale = float(hw_cfg["speed_scale"])
    wheel_radius = float(hw_cfg["wheel_radius"])
    max_wheel_rad_s = float(hw_cfg["max_wheel_angular_velocity_rad_s"])
    left_front_motor = int(hw_cfg["left_front_motor"])
    left_rear_motor = int(hw_cfg["left_rear_motor"])
    right_front_motor = int(hw_cfg["right_front_motor"])
    right_rear_motor = int(hw_cfg["right_rear_motor"])

    if can_id < 0 or can_id > 255:
        raise RuntimeError(f"{profile_file}: can_id must be in [0, 255].")
    if motor_freq <= 0 or motor_freq > 65535:
        raise RuntimeError(f"{profile_file}: motor_freq must be in [1, 65535].")
    if ticks_per_rotation <= 0:
        raise RuntimeError(f"{profile_file}: ticks_per_rotation must be > 0.")
    if wheel_radius <= 0.0:
        raise RuntimeError(f"{profile_file}: wheel_radius must be > 0.")
    if speed_scale < 0.0:
        raise RuntimeError(f"{profile_file}: speed_scale must be >= 0.")
    if max_wheel_rad_s <= 0.0:
        raise RuntimeError(
            f"{profile_file}: max_wheel_angular_velocity_rad_s must be > 0."
        )

    for motor in (
        left_front_motor,
        left_rear_motor,
        right_front_motor,
        right_rear_motor,
    ):
        if motor < -1 or motor > 3:
            raise RuntimeError(
                f"{profile_file}: motor indices must be -1 or in [0, 3]."
            )

    if left_front_motor < 0 and left_rear_motor < 0:
        raise RuntimeError(
            f"{profile_file}: at least one left motor index must be >= 0."
        )
    if right_front_motor < 0 and right_rear_motor < 0:
        raise RuntimeError(
            f"{profile_file}: at least one right motor index must be >= 0."
        )
    if drive_wheel_layout in ("mecanum", "omni"):
        for motor_name, motor_index in (
            ("left_front_motor", left_front_motor),
            ("left_rear_motor", left_rear_motor),
            ("right_front_motor", right_front_motor),
            ("right_rear_motor", right_rear_motor),
        ):
            if motor_index < 0:
                raise RuntimeError(
                    f"{profile_file}: {motor_name} must be >= 0 for "
                    f"{drive_wheel_layout} layout."
                )

    if "controller_manager" not in controllers:
        raise RuntimeError(
            f"{controllers_file}: missing top-level key 'controller_manager'."
        )
    if drive_controller_name not in controllers:
        raise RuntimeError(
            f"{controllers_file}: missing top-level key '{drive_controller_name}'."
        )

    cm = controllers["controller_manager"]
    if not isinstance(cm, dict) or "ros__parameters" not in cm:
        raise RuntimeError(
            f"{controllers_file}: controller_manager must contain 'ros__parameters'."
        )

    drive_controller = controllers[drive_controller_name]
    if (
        not isinstance(drive_controller, dict)
        or "ros__parameters" not in drive_controller
    ):
        raise RuntimeError(
            f"{controllers_file}: {drive_controller_name} must contain 'ros__parameters'."
        )

    cm_params = cm["ros__parameters"]
    cm_drive_cfg = cm_params.get(drive_controller_name)
    if not isinstance(cm_drive_cfg, dict):
        raise RuntimeError(
            f"{controllers_file}: controller_manager must contain "
            f"{drive_controller_name}.type."
        )
    if cm_drive_cfg.get("type") != drive_controller_type:
        raise RuntimeError(
            f"{controllers_file}: controller_manager {drive_controller_name}.type="
            f"'{cm_drive_cfg.get('type')}' does not match profile drive.controller_type="
            f"'{drive_controller_type}'."
        )

    drive_params = drive_controller["ros__parameters"]
    if drive_controller_type == "diff_drive_controller/DiffDriveController":
        wheel_separation = drive_params.get("wheel_separation")
        wheel_radius_cfg = drive_params.get("wheel_radius")
        if not isinstance(wheel_separation, Real) or float(wheel_separation) <= 0.0:
            raise RuntimeError(
                f"{controllers_file}: {drive_controller_name}.wheel_separation must be > 0."
            )
        if not isinstance(wheel_radius_cfg, Real) or float(wheel_radius_cfg) <= 0.0:
            raise RuntimeError(
                f"{controllers_file}: {drive_controller_name}.wheel_radius must be > 0."
            )

    if drive_controller_type == "mecanum_drive_controller/MecanumDriveController":
        required_joint_params = (
            "front_left_wheel_command_joint_name",
            "front_right_wheel_command_joint_name",
            "rear_right_wheel_command_joint_name",
            "rear_left_wheel_command_joint_name",
        )
        for key in required_joint_params:
            value = drive_params.get(key)
            if not isinstance(value, str) or not value.strip():
                raise RuntimeError(
                    f"{controllers_file}: {drive_controller_name}.{key} must be a non-empty string."
                )
        wheels_radius_cfg = drive_params.get("kinematics.wheels_radius")
        proj_cfg = drive_params.get("kinematics.sum_of_robot_center_projection_on_X_Y_axis")
        if not isinstance(wheels_radius_cfg, Real) or float(wheels_radius_cfg) <= 0.0:
            raise RuntimeError(
                f"{controllers_file}: {drive_controller_name}.kinematics.wheels_radius must be > 0."
            )
        if not isinstance(proj_cfg, Real):
            raise RuntimeError(
                f"{controllers_file}: {drive_controller_name}.kinematics.sum_of_robot_center_projection_on_X_Y_axis must be numeric."
            )

    return (
        profile_file,
        controllers_file,
        drive_controller_name,
        drive_controller_type,
        drive_wheel_layout,
    )


def _runtime_actions(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    use_joystick = LaunchConfiguration("use_joystick").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    robot_profile = LaunchConfiguration("robot_profile").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip()
    use_lidar = LaunchConfiguration("use_lidar").perform(context).strip()
    use_ground_truth_odom_tf = LaunchConfiguration("use_ground_truth_odom_tf").perform(context).strip()
    world = LaunchConfiguration("world").perform(context).strip()

    pkg_share = get_package_share_directory("studica_vmxpi_ros2")
    (
        _profile_file,
        _controllers_file,
        drive_controller_name,
        drive_controller_type,
        drive_wheel_layout,
    ) = _validate_profile_schema(robot_profile)
    drive_cmd_topic, drive_odom_topic = _drive_topics(
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

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "robot_gz_sim.launch.py")
            ),
            launch_arguments={
                "gui": gui,
                "use_hardware": use_hardware,
                "use_gz_sim": use_gz_sim,
                "use_sim_time": use_sim_time,
                "use_ground_truth_odom_tf": use_ground_truth_odom_tf,
                "world": world,
                "world_name": LaunchConfiguration("world_name").perform(context),
                "spawn_x": LaunchConfiguration("spawn_x").perform(context),
                "spawn_y": LaunchConfiguration("spawn_y").perform(context),
                "spawn_z": LaunchConfiguration("spawn_z").perform(context),
                "spawn_yaw": LaunchConfiguration("spawn_yaw").perform(context),
                "spawn_entity_name": LaunchConfiguration("spawn_entity_name").perform(context),
                "use_joystick": use_joystick,
                    "use_lidar": use_lidar,
                    "ydlidar_params_file": LaunchConfiguration("ydlidar_params_file").perform(context),
                    "joystick_cmd_vel_topic": LaunchConfiguration("joystick_cmd_vel_topic").perform(context),
                    "joystick_publish_stamped": LaunchConfiguration("joystick_publish_stamped").perform(context),
                    "drive_controller_name": drive_controller_name,
                    "drive_controller_type": drive_controller_type,
                    "drive_cmd_topic": drive_cmd_topic,
                    "drive_odom_topic": drive_odom_topic,
                    "robot_profile": robot_profile,
                }.items(),
            )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="gz_sim",
                description="Runtime mode: gz_sim | hardware | mock | gazebo_classic",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start RViz2 automatically.",
            ),
            DeclareLaunchArgument(
                "robot_profile",
                default_value="training_4wd",
                description="Robot profile under config/profiles.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="",
                description="Leave empty to auto-select by mode.",
            ),
            DeclareLaunchArgument(
                "use_ground_truth_odom_tf",
                default_value="true",
                description="In gz_sim, source /odom and /tf from Gazebo odometry topics.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="",
                description="World file path. Leave empty to use mode default world.",
            ),
            DeclareLaunchArgument("world_name", default_value="default"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.10"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("spawn_entity_name", default_value="robot_system_position"),
            DeclareLaunchArgument("use_joystick", default_value="false"),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="",
                description="Leave empty to auto-select (true in hardware mode).",
            ),
            DeclareLaunchArgument("ydlidar_params_file", default_value=""),
            DeclareLaunchArgument(
                "joystick_cmd_vel_topic",
                default_value="",
                description="Joystick command velocity output topic (empty = auto from drive profile).",
            ),
            DeclareLaunchArgument("joystick_publish_stamped", default_value="true"),
            OpaqueFunction(function=_runtime_actions),
        ]
    )
