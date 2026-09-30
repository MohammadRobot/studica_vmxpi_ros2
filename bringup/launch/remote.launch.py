"""One PC launch for supervised drive, SLAM or navigation against real hardware."""
from pathlib import Path
import sys
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _classroom_launch import deferred_include  # noqa: E402
from launch.substitutions import LaunchConfiguration


def session(context):
    mode = LaunchConfiguration('mode').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    share = Path(get_package_share_directory('studica_vmxpi_ros2'))
    if mode == 'navigation':
        if not Path(map_file).expanduser().is_file():
            raise RuntimeError('Navigation requires an existing map YAML: map:=/absolute/path.yaml')
        return [deferred_include('studica_vmxpi_ros2', 'navigation.launch.py', {
            'mode': 'hardware', 'map': str(Path(map_file).expanduser()),
            'use_point_cloud': 'false', 'hardware_max_linear_speed': '0.10',
            'hardware_max_angular_speed': '0.30',
            'navigation_output_topic': '/cmd_vel/navigation'}),
            Node(package='joy', executable='joy_node', name='joy_node',
                 parameters=[{'device_id': 0, 'deadzone': 0.10, 'autorepeat_rate': 20.0}]),
            Node(package='studica_vmxpi_ros2', executable='navigation_override.py',
                 name='navigation_override') ]
    speeds = {}
    for name, default, ceiling in [
        ('linear_speed', '0.20', 0.30), ('angular_speed', '0.60', 0.90),
        ('turbo_linear_speed', '0.30', 0.30), ('turbo_angular_speed', '0.90', 0.90),
    ]:
        value = float(LaunchConfiguration(name, default=default).perform(context))
        if not math.isfinite(value) or not 0 < value <= ceiling:
            raise RuntimeError(f'{name} must be positive and at most {ceiling}')
        speeds[name] = value
    nodes = [
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'device_id': 0, 'deadzone': 0.10, 'autorepeat_rate': 20.0}]),
        Node(package='studica_vmxpi_ros2', executable='navigation_override.py',
             name='navigation_override', parameters=[dict(speeds, allow_navigation=False)]),
    ]
    if mode == 'slam':
        nodes.append(deferred_include('studica_vmxpi_ros2', 'mapping.launch.py', {
            'mode': 'hardware', 'use_joystick': 'false'}))
    else:
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(share / 'description/robot/rviz/robot.rviz')],
                          parameters=[{'use_sim_time': False}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='drive', choices=['drive', 'slam', 'navigation']),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('linear_speed', default_value='0.20', description='Joystick m/s (maximum 0.30).'),
        DeclareLaunchArgument('angular_speed', default_value='0.60', description='Joystick rad/s (maximum 0.90).'),
        DeclareLaunchArgument('turbo_linear_speed', default_value='0.30', description='L1+R1 m/s (maximum 0.30).'),
        DeclareLaunchArgument('turbo_angular_speed', default_value='0.90', description='L1+R1 rad/s (maximum 0.90).'),
        OpaqueFunction(function=session),
    ])
