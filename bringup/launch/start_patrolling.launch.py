from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='studica_vmxpi_ros2',
            executable='robot_patrol_node',
            output='screen'),
    ])