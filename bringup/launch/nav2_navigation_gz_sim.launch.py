import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _maybe_include_nav2(context, *args, **kwargs):
    map_path = LaunchConfiguration("map").perform(context).strip()
    if not map_path:
        return [LogInfo(msg="Map path is empty. Pass map:=/absolute/path/to/map.yaml to start Nav2.")]
    if not os.path.exists(map_path):
        return [LogInfo(msg=f"Map file not found: {map_path}")]

    try:
        nav2_share = get_package_share_directory("nav2_bringup")
    except PackageNotFoundError:
        return [LogInfo(msg="nav2_bringup not found; install ros-humble-nav2-bringup.")]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
            launch_arguments={
                "slam": "False",
                "use_composition": "False",
                "map": map_path,
                "params_file": (
                    LaunchConfiguration("nav2_params_file").perform(context).strip()
                    or os.path.join(nav2_share, "params", "nav2_params.yaml")
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
                "autostart": LaunchConfiguration("autostart").perform(context),
            }.items(),
        )
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 from robot launch.",
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
            "use_joystick",
            default_value="false",
            description="Launch joystick teleop from studica_ros2_control.",
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
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Absolute path to map yaml file for Nav2 localization.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Autostart Nav2 lifecycle nodes.",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value="",
            description="Path to Nav2 parameter file (leave empty to use nav2_bringup default).",
        ),
    ]

    gui = LaunchConfiguration("gui")
    use_hardware = LaunchConfiguration("use_hardware")
    use_gz_sim = LaunchConfiguration("use_gz_sim")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joystick = LaunchConfiguration("use_joystick")
    joystick_cmd_vel_topic = LaunchConfiguration("joystick_cmd_vel_topic")
    joystick_publish_stamped = LaunchConfiguration("joystick_publish_stamped")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("studica_vmxpi_ros2"), "launch", "diffbot_gz_sim.launch.py"])
        ),
        launch_arguments={
            "gui": gui,
            "use_hardware": use_hardware,
            "use_gz_sim": use_gz_sim,
            "world": world,
            "world_name": world_name,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
            "use_sim_time": use_sim_time,
            "use_joystick": use_joystick,
            "joystick_cmd_vel_topic": joystick_cmd_vel_topic,
            "joystick_publish_stamped": joystick_publish_stamped,
        }.items(),
    )

    bridge = Node(
        package="studica_vmxpi_ros2",
        executable="nav2_topic_bridge_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_cmd_vel_topic": "/cmd_vel",
                "output_cmd_vel_topic": "/diffbot_base_controller/cmd_vel",
                "input_odom_topic": "/diffbot_base_controller/odom",
                "output_odom_topic": "/odom",
                "cmd_vel_frame_id": "base_link",
            }
        ],
    )

    return LaunchDescription(
        declared_arguments + [robot, bridge, OpaqueFunction(function=_maybe_include_nav2)]
    )
