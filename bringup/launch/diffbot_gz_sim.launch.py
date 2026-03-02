import os
import shlex

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _is_true(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")


def _sanitize_ld_library_path_for_rviz() -> str:
    """Drop Snap runtime entries that can break host-installed RViz binaries."""
    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    if not ld_library_path:
        return ""

    filtered_entries = [entry for entry in ld_library_path.split(":") if entry and "/snap/" not in entry]
    return ":".join(filtered_entries)


def _maybe_include_gamepad(context, *args, **kwargs):
    use_joystick = LaunchConfiguration("use_joystick").perform(context)
    if not _is_true(use_joystick):
        return []

    use_sim_time_value = LaunchConfiguration("use_sim_time").perform(context)
    if _is_true(LaunchConfiguration("use_gz_sim").perform(context)):
        # Keep stamped joystick commands valid even if /clock is unavailable.
        use_sim_time_value = "false"

    try:
        studica_pkg = get_package_share_directory("studica_ros2_control")
    except PackageNotFoundError:
        return [LogInfo(msg="studica_ros2_control not found; skipping joystick launch.")]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(studica_pkg, "launch", "gamepad_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time_value,
                "cmd_vel_topic": LaunchConfiguration("joystick_cmd_vel_topic").perform(context),
                "publish_stamped": LaunchConfiguration("joystick_publish_stamped").perform(context),
            }.items(),
        )
    ]


def _maybe_include_lidar(context, *args, **kwargs):
    use_lidar = LaunchConfiguration("use_lidar").perform(context)
    use_hardware = LaunchConfiguration("use_hardware").perform(context)
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    if not _is_true(use_lidar):
        return []
    if not _is_true(use_hardware):
        return [LogInfo(msg="use_lidar enabled but use_hardware is false; skipping LiDAR launch.")]
    if _is_true(use_gz_sim):
        return [LogInfo(msg="use_lidar enabled but use_gz_sim is true; skipping real LiDAR launch.")]

    try:
        studica_pkg = get_package_share_directory("studica_ros2_control")
    except PackageNotFoundError:
        return [LogInfo(msg="studica_ros2_control not found; skipping LiDAR launch.")]

    ydlidar_params_file = LaunchConfiguration("ydlidar_params_file").perform(context).strip()
    if not ydlidar_params_file:
        try:
            ydlidar_pkg = get_package_share_directory("ydlidar_ros2_driver")
            ydlidar_params_file = os.path.join(ydlidar_pkg, "params", "X2.yaml")
        except PackageNotFoundError:
            return [LogInfo(msg="ydlidar_ros2_driver not found; skipping LiDAR launch.")]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(studica_pkg, "launch", "lidar_launch.py")
            ),
            launch_arguments={
                "ydlidar_params_file": ydlidar_params_file,
            }.items(),
        )
    ]


def _maybe_include_gz_sim(context, *args, **kwargs):
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    if not _is_true(use_gz_sim):
        return []

    try:
        ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    except PackageNotFoundError:
        return [LogInfo(msg="ros_gz_sim not found. Install ros-humble-ros-gzharmonic-sim.")]

    world = LaunchConfiguration("world").perform(context).strip()
    if not world:
        return [LogInfo(msg="World path is empty. Pass world:=/absolute/path/to/world.sdf")]

    gz_version = LaunchConfiguration("gz_version").perform(context).strip() or "8"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-r {world}",
                "gz_version": gz_version,
            }.items(),
        )
    ]


def _maybe_add_gz_sim_runtime_nodes(context, *args, **kwargs):
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    if not _is_true(use_gz_sim):
        return []

    actions = []

    try:
        get_package_share_directory("ros_gz_bridge")
    except PackageNotFoundError:
        actions.append(LogInfo(msg="ros_gz_bridge not found. Install ros-humble-ros-gzharmonic-bridge."))
        return actions

    try:
        get_package_share_directory("ros_gz_sim")
    except PackageNotFoundError:
        actions.append(LogInfo(msg="ros_gz_sim not found. Install ros-humble-ros-gzharmonic-sim."))
        return actions

    world_name = LaunchConfiguration("world_name").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    use_hardware = LaunchConfiguration("use_hardware").perform(context)
    gz_version = LaunchConfiguration("gz_version").perform(context).strip() or "8"
    use_sim_time = _is_true(LaunchConfiguration("use_sim_time").perform(context))

    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            ],
            remappings=[("/scan", "/scan_raw")],
            output="screen",
        )
    )

    actions.append(
        Node(
            package="studica_vmxpi_ros2",
            executable="topic_adapter_node",
            name="scan_frame_relay",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "enable_scan_relay": True,
                    "scan_input_topic": "/scan_raw",
                    "scan_output_topic": "/scan",
                    "scan_output_frame_id": "laser_scan_frame",
                }
            ],
        )
    )

    spawn_entity_name = LaunchConfiguration("spawn_entity_name").perform(context).strip() or "diffbot_system_position"

    xacro_file = os.path.join(get_package_share_directory("studica_vmxpi_ros2"), "description", "urdf", "diffbot.urdf.xacro")
    spawn_script = (
        "set -e; "
        "TMP_URDF=/tmp/diffbot_spawn.urdf; "
        f"xacro {shlex.quote(xacro_file)} "
        "use_gazebo_classic:=false "
        "use_gz_sim:=true "
        f"use_hardware:={shlex.quote(use_hardware)} "
        f"gz_version:={shlex.quote(gz_version)} "
        "> \"$TMP_URDF\"; "
        f"REQ=\"sdf_filename: \\\"$TMP_URDF\\\" name: \\\"{spawn_entity_name}\\\" "
        f"pose {{ position {{ x: {spawn_x} y: {spawn_y} z: {spawn_z} }} orientation {{ w: 1 }} }}\"; "
        f"echo \"[spawn-gz-service] world={world_name} yaw={spawn_yaw} entity={spawn_entity_name}\"; "
        f"gz service -s /world/{world_name}/create "
        "--reqtype gz.msgs.EntityFactory "
        "--reptype gz.msgs.Boolean "
        "--timeout 10000 "
        "--req \"$REQ\""
    )

    actions.append(
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=["bash", "-lc", spawn_script],
                    output="screen",
                )
            ],
        )
    )

    return actions


def _maybe_add_gz_sim_controller_spawners(context, *args, **kwargs):
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    if not _is_true(use_gz_sim):
        return []

    configured_cm = LaunchConfiguration("controller_manager").perform(context).strip() or "/controller_manager"
    spawn_entity_name = LaunchConfiguration("spawn_entity_name").perform(context).strip() or "diffbot_system_position"

    # Some Gazebo setups namespace controller_manager under the model name.
    # Try root path first, then common model-scoped fallbacks.
    candidates = []
    for candidate in (
        configured_cm,
        f"/{spawn_entity_name}/controller_manager",
        "/diffdrive_robot/controller_manager",
    ):
        if candidate not in candidates:
            candidates.append(candidate)

    quoted_candidates = " ".join(shlex.quote(candidate) for candidate in candidates)
    spawn_script = (
        "set +e; "
        f"CANDIDATES=({quoted_candidates}); "
        'for cm in "${CANDIDATES[@]}"; do '
        '  echo "[spawner-fallback] trying ${cm}"; '
        '  ros2 run controller_manager spawner joint_state_broadcaster --controller-manager "${cm}" || true; '
        '  ros2 run controller_manager spawner imu_sensor_broadcaster --controller-manager "${cm}" || true; '
        '  ros2 run controller_manager spawner diffbot_base_controller --controller-manager "${cm}" || true; '
        '  if ros2 control list_controllers --controller-manager "${cm}" 2>/dev/null | '
        '     grep -q "diffbot_base_controller"; then '
        "    exit 0; "
        "  fi; "
        "done; "
        'echo "[spawner-fallback] failed. tried: ${CANDIDATES[*]}" >&2; '
        "exit 0"
    )

    return [
        ExecuteProcess(
            cmd=["bash", "-lc", spawn_script],
            output="screen",
        )
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Use Titan hardware instead of mock system.",
        ),
        DeclareLaunchArgument(
            "use_gz_sim",
            default_value="true",
            description="Start Gazebo Sim.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=LaunchConfiguration("use_gz_sim"),
            description="Use simulation time (defaults to use_gz_sim).",
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
            "gz_version",
            default_value="8",
            description="Gazebo Sim major version passed to ros_gz_sim (8 for Harmonic).",
        ),
        DeclareLaunchArgument(
            "imu_gz_topic",
            default_value="/imu",
            description="Gazebo IMU topic to bridge into ROS /imu.",
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
            "spawn_entity_name",
            default_value="diffbot_system_position",
            description="Entity name used when spawning the robot into Gazebo Sim.",
        ),
        DeclareLaunchArgument(
            "controller_manager",
            default_value="/controller_manager",
            description="Preferred controller_manager service root for controller spawners.",
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="false",
            description="Launch joystick teleop from studica_ros2_control.",
        ),
        DeclareLaunchArgument(
            "use_lidar",
            default_value=LaunchConfiguration("use_hardware"),
            description="Launch YDLIDAR in real hardware mode (defaults to use_hardware).",
        ),
        DeclareLaunchArgument(
            "ydlidar_params_file",
            default_value="",
            description="Optional YDLIDAR params YAML. Empty uses ydlidar_ros2_driver/params/X2.yaml.",
        ),
        DeclareLaunchArgument(
            "joystick_cmd_vel_topic",
            default_value="/diffbot_base_controller/cmd_vel",
            description="Joystick command velocity output topic.",
        ),
        DeclareLaunchArgument(
            "joystick_publish_stamped",
            default_value="true",
            description="Publish TwistStamped joystick commands.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    use_hardware = LaunchConfiguration("use_hardware")
    use_gz_sim = LaunchConfiguration("use_gz_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_studica_vmxpi_ros2 = get_package_share_directory("studica_vmxpi_ros2")
    install_dir = get_package_prefix("studica_vmxpi_ros2")
    gz_models_path = os.path.join(pkg_studica_vmxpi_ros2, "description", "models")

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] = (
            os.environ["GZ_SIM_RESOURCE_PATH"] + ":" + install_dir + "/share" + ":" + gz_models_path
        )
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = install_dir + "/share" + ":" + gz_models_path

    if "GZ_SIM_SYSTEM_PLUGIN_PATH" in os.environ:
        os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] + ":" + install_dir + "/lib"
    else:
        os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = install_dir + "/lib"

    print("GZ SIM RESOURCE PATH==" + str(os.environ["GZ_SIM_RESOURCE_PATH"]))
    print("GZ SIM SYSTEM PLUGINS PATH==" + str(os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"]))

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("studica_vmxpi_ros2"), "description/urdf", "diffbot.urdf.xacro"]),
            " ",
            "use_gazebo_classic:=false",
            " ",
            "use_gz_sim:=", use_gz_sim,
            " ",
            "use_hardware:=", use_hardware,
            " ",
            "gz_version:=", LaunchConfiguration("gz_version"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": use_sim_time,
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("studica_vmxpi_ros2"), "description/diffbot/rviz", "diffbot.rviz"]
    )
    robot_controllers = PathJoinSubstitution([FindPackageShare("studica_vmxpi_ros2"), "config", "diffbot_controllers.yaml"])
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
                [
                    "('",
                    use_hardware,
                    "').lower() in ['true','1','yes','on'] and ('",
                    use_gz_sim,
                    "').lower() in ['false','0','no','off']",
                ]
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
                "use_sim_time": use_sim_time,
                "enable_imu_relay": True,
                "imu_input_topic": "/imu_sensor_broadcaster/imu",
                "imu_output_topic": "/imu",
                "imu_use_odom_fallback": ParameterValue(use_gz_sim, value_type=bool),
                "imu_fallback_odom_topic": "/diffbot_base_controller/odom",
                "imu_fallback_frame_id": "imu_link",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "(('",
                    use_gz_sim,
                    "').lower() in ['true','1','yes','on']) or (('",
                    use_hardware,
                    "').lower() in ['true','1','yes','on'] and ('",
                    use_gz_sim,
                    "').lower() in ['false','0','no','off'])",
                ]
            )
        ),
    )

    control_api_bridge_node = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="nav2_topic_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "enable_nav2_bridge": True,
                "input_cmd_vel_topic": "/cmd_vel",
                "output_cmd_vel_topic": "/diffbot_base_controller/cmd_vel",
                "input_odom_topic": "/diffbot_base_controller/odom",
                "output_odom_topic": "/odom",
                "cmd_vel_frame_id": "base_link",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    use_hardware,
                    "').lower() in ['true','1','yes','on'] and ('",
                    use_gz_sim,
                    "').lower() in ['false','0','no','off']",
                ]
            )
        ),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_gz_sim),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_file],
        additional_env=rviz_env,
        condition=IfCondition(gui),
    )

    base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0.10", "0", "0", "0", "base_footprint", "base_link"],
    )

    # Hardware-only control path (no simulation).
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="",
        parameters=[robot_description, robot_controllers, {"use_sim_time": use_sim_time}],
        output="screen",
        condition=UnlessCondition(use_gz_sim),
    )

    nodes = [
        OpaqueFunction(function=_maybe_include_gz_sim),
        OpaqueFunction(function=_maybe_add_gz_sim_runtime_nodes),
        OpaqueFunction(function=_maybe_add_gz_sim_controller_spawners),
        control_node,
        node_robot_state_publisher,
        base_footprint_tf,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        imu_alias_relay_node,
        control_api_bridge_node,
        robot_controller_spawner,
        rviz_node,
        OpaqueFunction(function=_maybe_include_gamepad),
        OpaqueFunction(function=_maybe_include_lidar),
    ]

    return LaunchDescription(declared_arguments + nodes)
