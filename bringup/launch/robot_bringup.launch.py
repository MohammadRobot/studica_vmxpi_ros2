# Combined bringup: ros2_control (vmxpi_ros2) + optional studica sensors.

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _maybe_include_studica(context, *args, **kwargs):
    use_studica = LaunchConfiguration("use_studica_sensors").perform(context).lower()
    if use_studica not in ("true", "1", "yes"):
        return []

    try:
        pkg_share = get_package_share_directory("studica_control")
    except PackageNotFoundError:
        return [LogInfo(msg="studica_control not found; skipping sensors_only launch.")]

    return [
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
            "use_sim_time",
            default_value=LaunchConfiguration("use_gazebo_classic"),
            description="Use simulation time (defaults to use_gazebo_classic).",
        ),
        DeclareLaunchArgument(
            "use_studica_sensors",
            default_value=LaunchConfiguration("use_hardware"),
            description="Launch studica_control sensors_only node (defaults to use_hardware).",
        ),
    ]

    gui = LaunchConfiguration("gui")
    use_hardware = LaunchConfiguration("use_hardware")
    use_gazebo_classic = LaunchConfiguration("use_gazebo_classic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    vmxpi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("vmxpi_ros2"), "bringup/launch", "diffbot_gazebo_classic.launch.py"]
            )
        ),
        launch_arguments={
            "gui": gui,
            "use_hardware": use_hardware,
            "use_gazebo_classic": use_gazebo_classic,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    nodes = [
        vmxpi_launch,
        OpaqueFunction(function=_maybe_include_studica),
    ]

    return LaunchDescription(declared_arguments + nodes)
