"""Launch the complete Lab 5 reference node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start one read-only odometry reporter using simulation time."""
    reporter = Node(
        package="robot_course_examples_solution",
        executable="sensor_reporter_solution",
        name="sensor_reporter_solution",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_name": "class_4wd",
                "report_period_sec": 1.0,
            }
        ],
    )
    return LaunchDescription([reporter])
