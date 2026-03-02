import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="false",
            description="Launch joystick teleop from studica_ros2_control.",
        ),
        DeclareLaunchArgument(
            "use_lidar",
            default_value="true",
            description="Launch YDLIDAR from studica_ros2_control/lidar_launch.py.",
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joystick = LaunchConfiguration("use_joystick")
    use_lidar = LaunchConfiguration("use_lidar")
    ydlidar_params_file = LaunchConfiguration("ydlidar_params_file")
    joystick_cmd_vel_topic = LaunchConfiguration("joystick_cmd_vel_topic")
    joystick_publish_stamped = LaunchConfiguration("joystick_publish_stamped")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("studica_vmxpi_ros2"), "launch", "diffbot_gz_sim.launch.py"])
        ),
        launch_arguments={
            "gui": gui,
            "use_hardware": "true",
            "use_gz_sim": "false",
            "use_sim_time": use_sim_time,
            "use_joystick": use_joystick,
            "use_lidar": use_lidar,
            "ydlidar_params_file": ydlidar_params_file,
            "joystick_cmd_vel_topic": joystick_cmd_vel_topic,
            "joystick_publish_stamped": joystick_publish_stamped,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [robot, OpaqueFunction(function=_maybe_include_nav2)]
    )
