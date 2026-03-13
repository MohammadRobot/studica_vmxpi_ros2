# Legacy combined bringup: ros2_control (studica_vmxpi_ros2) + optional studica sensors.

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _maybe_include_studica(context, *args, **kwargs):
    use_studica = LaunchConfiguration("use_studica_sensors").perform(context).lower()
    if use_studica not in ("true", "1", "yes"):
        return []

    try:
        pkg_share = get_package_share_directory("studica_ros2_control")
    except PackageNotFoundError:
        return [LogInfo(msg="studica_ros2_control not found; skipping sensors_only launch.")]

    return [
        LogInfo(
            msg="Launching legacy studica_ros2_control sensors_only alongside ros2_control (not recommended for production)."
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "sensors_only.launch.py")
            )
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
            "robot_profile",
            default_value="training_4wd",
            description="Robot profile under config/profiles.",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Use Titan hardware instead of mock system.",
        ),
        DeclareLaunchArgument(
            "use_gazebo_classic",
            default_value="false",
            description="Start Gazebo Classic simulation.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/gazebo/worlds", "diff_drive_world.world"]
            ),
            description="Absolute path to Gazebo world file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=LaunchConfiguration("use_gazebo_classic"),
            description="Use simulation time (defaults to use_gazebo_classic).",
        ),
        DeclareLaunchArgument(
            "use_studica_sensors",
            default_value="false",
            description="Legacy mode: launch studica_ros2_control sensors_only node (disabled by default).",
        ),
    ]

    gui = LaunchConfiguration("gui")
    robot_profile = LaunchConfiguration("robot_profile")
    use_hardware = LaunchConfiguration("use_hardware")
    use_gazebo_classic = LaunchConfiguration("use_gazebo_classic")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    mode = PythonExpression(
        [
            "'gazebo_classic' if ('",
            use_gazebo_classic,
            "').lower() in ['true','1','yes','on'] else "
            "('hardware' if ('",
            use_hardware,
            "').lower() in ['true','1','yes','on'] else 'mock')",
        ]
    )

    vmxpi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments={
            "mode": mode,
            "gui": gui,
            "world": world,
            "use_sim_time": use_sim_time,
            "robot_profile": robot_profile,
        }.items(),
    )

    nodes = [
        LogInfo(
            msg=(
                "Compatibility launch wrapper in use (robot_bringup.launch.py). "
                "Prefer bringup.launch.py with mode:=hardware|mock|gazebo_classic for new workflows."
            )
        ),
        vmxpi_launch,
        OpaqueFunction(function=_maybe_include_studica),
    ]

    return LaunchDescription(declared_arguments + nodes)
