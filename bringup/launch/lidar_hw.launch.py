"""
Hardware LiDAR launch owned by studica_vmxpi_ros2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ydlidar_params_file = LaunchConfiguration("ydlidar_params_file")
    lidar_parent_frame = LaunchConfiguration("lidar_parent_frame")
    lidar_child_frame = LaunchConfiguration("lidar_child_frame")
    lidar_tf_x = LaunchConfiguration("lidar_tf_x")
    lidar_tf_y = LaunchConfiguration("lidar_tf_y")
    lidar_tf_z = LaunchConfiguration("lidar_tf_z")
    lidar_tf_qx = LaunchConfiguration("lidar_tf_qx")
    lidar_tf_qy = LaunchConfiguration("lidar_tf_qy")
    lidar_tf_qz = LaunchConfiguration("lidar_tf_qz")
    lidar_tf_qw = LaunchConfiguration("lidar_tf_qw")

    declared_arguments = [
        DeclareLaunchArgument(
            "ydlidar_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "config", "ydlidar_x2_hw.yaml"]
            ),
            description="Path to YDLidar ROS2 params file.",
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
        DeclareLaunchArgument("lidar_tf_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_z", default_value="0.02"),
        DeclareLaunchArgument("lidar_tf_qx", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_qy", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_qz", default_value="1.0"),
        DeclareLaunchArgument("lidar_tf_qw", default_value="0.0"),
    ]

    driver_node = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[ydlidar_params_file],
        namespace="/",
    )

    lidar_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser",
        arguments=[
            lidar_tf_x,
            lidar_tf_y,
            lidar_tf_z,
            lidar_tf_qx,
            lidar_tf_qy,
            lidar_tf_qz,
            lidar_tf_qw,
            lidar_parent_frame,
            lidar_child_frame,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            LogInfo(msg=["Using YDLidar params: ", ydlidar_params_file]),
            LogInfo(
                msg=[
                    "LiDAR TF ",
                    lidar_parent_frame,
                    " -> ",
                    lidar_child_frame,
                    " quat=(",
                    lidar_tf_qx,
                    ",",
                    lidar_tf_qy,
                    ",",
                    lidar_tf_qz,
                    ",",
                    lidar_tf_qw,
                    ")",
                ]
            ),
            driver_node,
            lidar_tf_node,
        ]
    )
