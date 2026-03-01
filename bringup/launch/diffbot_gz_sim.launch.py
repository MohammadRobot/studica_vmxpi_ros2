import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _is_true(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")


def _maybe_include_gamepad(context, *args, **kwargs):
    use_joystick = LaunchConfiguration("use_joystick").perform(context)
    if not _is_true(use_joystick):
        return []

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
                "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
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

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-r {world}",
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
    imu_gz_topic = LaunchConfiguration("imu_gz_topic").perform(context).strip() or "/imu"
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    use_sim_time = _is_true(LaunchConfiguration("use_sim_time").perform(context))

    bridge_remappings = [("/scan", "/scan_raw")]
    if imu_gz_topic != "/imu":
        bridge_remappings.append((imu_gz_topic, "/imu"))

    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                f"{imu_gz_topic}@sensor_msgs/msg/Imu[gz.msgs.IMU",
            ],
            remappings=bridge_remappings,
            output="screen",
        )
    )

    actions.append(
        Node(
            package="studica_vmxpi_ros2",
            executable="scan_frame_relay_node",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_topic": "/scan_raw",
                    "output_topic": "/scan",
                    "output_frame_id": "laser_scan_frame",
                }
            ],
        )
    )

    actions.append(
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    arguments=[
                        "-topic",
                        "robot_description",
                        "-name",
                        "diffbot_system_position",
                        "-world",
                        world_name,
                        "-x",
                        spawn_x,
                        "-y",
                        spawn_y,
                        "-z",
                        spawn_z,
                        "-Y",
                        spawn_yaw,
                    ],
                    output="screen",
                )
            ],
        )
    )

    return actions


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
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_file],
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
        control_node,
        node_robot_state_publisher,
        base_footprint_tf,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
        OpaqueFunction(function=_maybe_include_gamepad),
        OpaqueFunction(function=_maybe_include_lidar),
    ]

    return LaunchDescription(declared_arguments + nodes)
