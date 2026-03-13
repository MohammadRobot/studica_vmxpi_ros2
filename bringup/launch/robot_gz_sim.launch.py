import math
import os
import shlex

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _is_true(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")


def _sanitize_ld_library_path_for_rviz() -> str:
    """
    Drop environment-specific runtime entries that can break host ROS binaries.

    Removes:
    - Snap runtime entries (RViz issues on some desktops)
    - Conda/Miniconda paths (common libstdc++ ABI mismatch with ROS Humble)
    """
    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    if not ld_library_path:
        return ""

    conda_roots = []
    conda_prefix = os.environ.get("CONDA_PREFIX", "").strip()
    if conda_prefix:
        conda_roots.append(conda_prefix)
    conda_exe = os.environ.get("CONDA_EXE", "").strip()
    if conda_exe:
        conda_roots.append(os.path.dirname(os.path.dirname(conda_exe)))
    conda_roots = [os.path.realpath(path) for path in conda_roots if path]

    def _is_conda_entry(path: str) -> bool:
        real_path = os.path.realpath(path)
        if any(real_path == root or real_path.startswith(root + os.sep) for root in conda_roots):
            return True
        lowered = real_path.lower()
        return any(
            token in lowered for token in ("/miniconda", "/anaconda", "/mambaforge", "/micromamba")
        )

    filtered_entries = []
    for entry in ld_library_path.split(":"):
        if not entry:
            continue
        if "/snap/" in entry:
            continue
        if _is_conda_entry(entry):
            continue
        filtered_entries.append(entry)
    return ":".join(filtered_entries)


def _profile_assets(profile_name: str):
    pkg_share = get_package_share_directory("studica_vmxpi_ros2")
    profile_dir = os.path.join(pkg_share, "config", "profiles", profile_name)
    profile_file = os.path.join(profile_dir, "robot_profile.yaml")
    controllers_file = os.path.join(profile_dir, "robot_controllers.yaml")
    return profile_file, controllers_file


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

    joystick_cmd_vel_topic = LaunchConfiguration("joystick_cmd_vel_topic").perform(context).strip()
    if not joystick_cmd_vel_topic:
        joystick_cmd_vel_topic = LaunchConfiguration("drive_cmd_topic").perform(context).strip()

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(studica_pkg, "launch", "gamepad_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time_value,
                "cmd_vel_topic": joystick_cmd_vel_topic,
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
        studica_pkg = get_package_share_directory("studica_vmxpi_ros2")
    except PackageNotFoundError:
        return [LogInfo(msg="studica_vmxpi_ros2 not found; skipping LiDAR launch.")]
    try:
        get_package_share_directory("ydlidar_ros2_driver")
    except PackageNotFoundError:
        return [LogInfo(msg="ydlidar_ros2_driver not found; skipping LiDAR launch.")]

    ydlidar_params_file = LaunchConfiguration("ydlidar_params_file").perform(context).strip()
    if not ydlidar_params_file:
        ydlidar_params_file = os.path.join(studica_pkg, "config", "ydlidar_x2_hw.yaml")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(studica_pkg, "launch", "lidar_hw.launch.py")
            ),
            launch_arguments={
                "ydlidar_params_file": ydlidar_params_file,
                "lidar_parent_frame": LaunchConfiguration("lidar_parent_frame").perform(context),
                "lidar_child_frame": LaunchConfiguration("lidar_child_frame").perform(context),
                "lidar_tf_x": LaunchConfiguration("lidar_tf_x").perform(context),
                "lidar_tf_y": LaunchConfiguration("lidar_tf_y").perform(context),
                "lidar_tf_z": LaunchConfiguration("lidar_tf_z").perform(context),
                "lidar_tf_qx": LaunchConfiguration("lidar_tf_qx").perform(context),
                "lidar_tf_qy": LaunchConfiguration("lidar_tf_qy").perform(context),
                "lidar_tf_qz": LaunchConfiguration("lidar_tf_qz").perform(context),
                "lidar_tf_qw": LaunchConfiguration("lidar_tf_qw").perform(context),
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
    use_ground_truth_odom_tf = LaunchConfiguration("use_ground_truth_odom_tf").perform(context)
    robot_profile = LaunchConfiguration("robot_profile").perform(context).strip() or "training_4wd"
    gz_version = LaunchConfiguration("gz_version").perform(context).strip() or "8"
    use_sim_time = _is_true(LaunchConfiguration("use_sim_time").perform(context))
    profile_file, controllers_file = _profile_assets(robot_profile)

    if not os.path.exists(profile_file):
        actions.append(LogInfo(msg=f"Robot profile file not found: {profile_file}"))
        return actions
    if not os.path.exists(controllers_file):
        actions.append(LogInfo(msg=f"Controller file not found: {controllers_file}"))
        return actions

    try:
        spawn_yaw_value = float(spawn_yaw)
    except ValueError:
        spawn_yaw_value = 0.0
        actions.append(LogInfo(msg=f"Invalid spawn_yaw '{spawn_yaw}'. Falling back to 0.0."))
    spawn_qz = math.sin(spawn_yaw_value / 2.0)
    spawn_qw = math.cos(spawn_yaw_value / 2.0)

    bridge_arguments = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ]
    if _is_true(use_ground_truth_odom_tf):
        bridge_arguments.extend(
            [
                "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            ]
        )

    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=bridge_arguments,
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

    spawn_entity_name = LaunchConfiguration("spawn_entity_name").perform(context).strip() or "robot_system_position"

    xacro_file = os.path.join(get_package_share_directory("studica_vmxpi_ros2"), "description", "urdf", "robot.urdf.xacro")
    spawn_script = (
        "set -e; "
        "TMP_URDF=/tmp/robot_spawn.urdf; "
        f"xacro {shlex.quote(xacro_file)} "
        "use_gazebo_classic:=false "
        "use_gz_sim:=true "
        f"use_hardware:={shlex.quote(use_hardware)} "
        f"gz_version:={shlex.quote(gz_version)} "
        f"profile_file:={shlex.quote(profile_file)} "
        f"controllers_file:={shlex.quote(controllers_file)} "
        "> \"$TMP_URDF\"; "
        f"REQ=\"sdf_filename: \\\"$TMP_URDF\\\" name: \\\"{spawn_entity_name}\\\" "
        f"pose {{ position {{ x: {spawn_x} y: {spawn_y} z: {spawn_z} }} orientation {{ z: {spawn_qz} w: {spawn_qw} }} }}\"; "
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
    spawn_entity_name = LaunchConfiguration("spawn_entity_name").perform(context).strip() or "robot_system_position"
    drive_controller_name = (
        LaunchConfiguration("drive_controller_name").perform(context).strip()
        or "robot_base_controller"
    )

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
    quoted_drive_controller_name = shlex.quote(drive_controller_name)
    spawn_script = f"""
set +e
CANDIDATES=({quoted_candidates})
DRIVE_CONTROLLER={quoted_drive_controller_name}

run_spawner() {{
  local name="$1"
  local cm="$2"
  local output
  local rc

  output="$(ros2 run controller_manager spawner "$name" --controller-manager "$cm" --controller-manager-timeout 20 2>&1)"
  rc=$?
  echo "$output"

  if [ "$rc" -eq 0 ]; then
    return 0
  fi

  # If already loaded, treat this as success and continue activation flow.
  if echo "$output" | grep -q "Controller already loaded, skipping load_controller"; then
    return 0
  fi

  return "$rc"
}}

for cm in "${{CANDIDATES[@]}}"; do
  echo "[spawner-fallback] trying ${{cm}}"
  for attempt in $(seq 1 20); do
    ok=1
    run_spawner joint_state_broadcaster "$cm" || ok=0
    run_spawner imu_sensor_broadcaster "$cm" || ok=0
    run_spawner "$DRIVE_CONTROLLER" "$cm" || ok=0
    if [ "$ok" -eq 1 ]; then
      echo "[spawner-fallback] controllers active on ${{cm}}"
      exit 0
    fi
    echo "[spawner-fallback] retry ${{attempt}}/20 for ${{cm}}"
    sleep 1
  done
done

echo "[spawner-fallback] failed. tried: ${{CANDIDATES[*]}}" >&2
exit 0
"""

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
            "rviz_start_delay",
            default_value="10.0",
            description="Delay before starting RViz2 (seconds). Lower for faster startup.",
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
            "use_ground_truth_odom_tf",
            default_value="true",
            description="In gz_sim, source /odom and /tf from Gazebo ground-truth odometry.",
        ),
        DeclareLaunchArgument(
            "robot_profile",
            default_value="training_4wd",
            description="Robot profile under config/profiles (for example: training_4wd, training_2wd).",
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
            default_value="robot_system_position",
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
            description="Optional YDLIDAR params YAML. Empty uses studica_vmxpi_ros2/config/ydlidar_x2_hw.yaml.",
        ),
        DeclareLaunchArgument(
            "lidar_parent_frame",
            default_value="base_link",
            description="Parent frame for LiDAR static transform.",
        ),
        DeclareLaunchArgument(
            "lidar_child_frame",
            default_value="laser_frame",
            description="Child frame for LiDAR static transform.",
        ),
        DeclareLaunchArgument(
            "lidar_tf_x",
            default_value="0.0",
            description="LiDAR static TF translation X (meters).",
        ),
        DeclareLaunchArgument(
            "lidar_tf_y",
            default_value="0.0",
            description="LiDAR static TF translation Y (meters).",
        ),
        DeclareLaunchArgument(
            "lidar_tf_z",
            default_value="0.02",
            description="LiDAR static TF translation Z (meters).",
        ),
        DeclareLaunchArgument(
            "lidar_tf_qx",
            default_value="0.0",
            description="LiDAR static TF quaternion X.",
        ),
        DeclareLaunchArgument(
            "lidar_tf_qy",
            default_value="0.0",
            description="LiDAR static TF quaternion Y.",
        ),
        DeclareLaunchArgument(
            "lidar_tf_qz",
            default_value="1.0",
            description="LiDAR static TF quaternion Z (default rotates hardware LiDAR 180 degrees yaw).",
        ),
        DeclareLaunchArgument(
            "lidar_tf_qw",
            default_value="0.0",
            description="LiDAR static TF quaternion W.",
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
            "drive_controller_name",
            default_value="robot_base_controller",
            description="Primary drive controller name loaded by controller_manager.",
        ),
        DeclareLaunchArgument(
            "drive_controller_type",
            default_value="diff_drive_controller/DiffDriveController",
            description="Primary drive controller plugin type.",
        ),
        DeclareLaunchArgument(
            "drive_cmd_topic",
            default_value="/robot_base_controller/cmd_vel",
            description="Primary drive command topic.",
        ),
        DeclareLaunchArgument(
            "drive_odom_topic",
            default_value="/robot_base_controller/odom",
            description="Primary drive odometry topic.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    use_hardware = LaunchConfiguration("use_hardware")
    use_gz_sim = LaunchConfiguration("use_gz_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
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
            "use_gazebo_classic:=false",
            " ",
            "use_gz_sim:=", use_gz_sim,
            " ",
            "use_hardware:=", use_hardware,
            " ",
            "gz_version:=", LaunchConfiguration("gz_version"),
            " ",
            "profile_file:=", profile_file,
            " ",
            "controllers_file:=", robot_controllers,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": use_sim_time,
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("studica_vmxpi_ros2"), "description/robot/rviz", "robot.rviz"]
    )
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
                "imu_fallback_odom_topic": drive_odom_topic,
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

    control_api_bridge_node_gt = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="nav2_topic_bridge",
        output="screen",
        parameters=[
                {
                    "use_sim_time": use_sim_time,
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
                [
                    "('",
                    use_gz_sim,
                    "').lower() in ['true','1','yes','on'] and ('",
                    use_ground_truth_odom_tf,
                    "').lower() in ['true','1','yes','on']",
                ]
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
                "use_sim_time": use_sim_time,
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
                [
                    "(('",
                    use_hardware,
                    "').lower() in ['true','1','yes','on']) or (('",
                    use_gz_sim,
                    "').lower() in ['true','1','yes','on'] and ('",
                    use_ground_truth_odom_tf,
                    "').lower() in ['false','0','no','off'])",
                ]
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
                "use_sim_time": use_sim_time,
                "enable_tf_relay": True,
                "tf_input_topic": "/ground_truth/tf",
                "tf_output_topic": "/tf",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    use_gz_sim,
                    "').lower() in ['true','1','yes','on'] and ('",
                    use_ground_truth_odom_tf,
                    "').lower() in ['true','1','yes','on']",
                ]
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
                "use_sim_time": use_sim_time,
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
                    "').strip() == 'mecanum_drive_controller/MecanumDriveController') and not (('",
                    use_gz_sim,
                    "').lower() in ['true','1','yes','on'] and ('",
                    use_ground_truth_odom_tf,
                    "').lower() in ['true','1','yes','on'])",
                ]
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
        parameters=[{"use_sim_time": use_sim_time}],
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
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["0", "0", "0.0", "0", "0", "0", "base_footprint", "base_link"],
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
    ]

    return LaunchDescription(declared_arguments + nodes)
