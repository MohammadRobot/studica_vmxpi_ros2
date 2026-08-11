"""Reference odometry reporter for Lab 5."""

from typing import Optional, Tuple

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger


def format_summary(robot_name: str, position: Tuple[float, float]) -> str:
    """Format one stable, testable planar odometry summary."""
    return f"{robot_name}: x={position[0]:.3f} m, y={position[1]:.3f} m"


class SensorReporter(Node):
    """Subscribe to odometry and expose it through a topic and service."""

    def __init__(self) -> None:
        super().__init__("sensor_reporter_solution")
        self._latest_xy: Optional[Tuple[float, float]] = None
        self.declare_parameter("robot_name", "class_4wd")
        self.declare_parameter("report_period_sec", 1.0)

        self._summary_publisher = self.create_publisher(String, "/apps/odom_summary", 10)
        self._odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            qos_profile_sensor_data,
        )
        self._report_service = self.create_service(
            Trigger,
            "/apps/report_now",
            self._on_report_now,
        )
        period = float(self.get_parameter("report_period_sec").value)
        if period <= 0.0:
            raise ValueError("report_period_sec must be greater than zero")
        self._timer = self.create_timer(period, self._publish_summary)
        self.get_logger().info("Sensor reporter started; waiting for /odom")

    def _on_odom(self, message: Odometry) -> None:
        """Store the latest planar position from odometry."""
        position = message.pose.pose.position
        self._latest_xy = (position.x, position.y)

    def _build_summary(self) -> Optional[str]:
        if self._latest_xy is None:
            return None
        robot_name = str(self.get_parameter("robot_name").value)
        return format_summary(robot_name, self._latest_xy)

    def _publish_summary(self) -> Optional[str]:
        summary = self._build_summary()
        if summary is None:
            self.get_logger().info("Waiting for the first /odom message")
            return None
        message = String()
        message.data = summary
        self._summary_publisher.publish(message)
        return summary

    def _on_report_now(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Publish and return the latest summary without changing robot state."""
        del request
        summary = self._publish_summary()
        response.success = summary is not None
        response.message = summary if summary is not None else "waiting for /odom"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
