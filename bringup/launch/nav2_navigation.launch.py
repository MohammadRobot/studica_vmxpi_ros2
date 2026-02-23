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
            "use_gazebo_classic",
            default_value="true",
            description="Start Gazebo Classic simulation.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("vmxpi_ros2"), "description/gazebo/worlds", "diff_drive_world.world"]
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
            description="Launch joystick teleop from studica_control.",
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
    use_gazebo_classic = LaunchConfiguration("use_gazebo_classic")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joystick = LaunchConfiguration("use_joystick")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("vmxpi_ros2"), "launch", "diffbot_gazebo_classic.launch.py"]
            )
        ),
        launch_arguments={
            "gui": gui,
            "use_hardware": use_hardware,
            "use_gazebo_classic": use_gazebo_classic,
            "world": world,
            "use_sim_time": use_sim_time,
            "use_joystick": use_joystick,
        }.items(),
    )

    bridge = Node(
        package="vmxpi_ros2",
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

    base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )

    return LaunchDescription(
        declared_arguments + [robot, bridge, base_footprint_tf, OpaqueFunction(function=_maybe_include_nav2)]
    )
