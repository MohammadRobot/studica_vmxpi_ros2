"""TODO starter: summarize odometry without commanding robot motion."""

from typing import Optional, Tuple

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SensorReporter(Node):
    """Observe odometry and publish a student-readable summary."""

    def __init__(self) -> None:
        super().__init__("sensor_reporter")
        self._latest_xy: Optional[Tuple[float, float]] = None

        # TODO 1: Declare a string parameter named "robot_name" with the
        # default value "class_4wd".

        # TODO 2: Create a String publisher on /student/odom_summary.
        self._summary_publisher = None

        # TODO 3: Subscribe to /odom with Odometry, self._on_odom, and
        # self._odom_qos.
        self._odom_qos = qos_profile_sensor_data
        self._odom_subscription = None

        # TODO 4: Create a Trigger service named /student/report_now that uses
        # self._on_report_now.
        self._report_service = None

        self._timer = self.create_timer(1.0, self._publish_summary)

    def _on_odom(self, message: Odometry) -> None:
        """Save the latest planar position."""
        # TODO 5: Read x and y from message.pose.pose.position and store them
        # together in self._latest_xy.
        del message

    def _build_summary(self) -> Optional[str]:
        """Return text for the most recent odometry, or None while waiting."""
        # TODO 6: If no odometry has arrived, return None. Otherwise read the
        # robot_name parameter and format x/y with three decimal places.
        return None

    def _publish_summary(self) -> Optional[str]:
        """Publish a summary when both a publisher and odometry are ready."""
        summary = self._build_summary()
        if summary is None or self._summary_publisher is None:
            self.get_logger().info("Complete the TODOs; waiting for /odom")
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
        """TODO service callback that reports the latest observation."""
        del request
        # TODO 7: Call _publish_summary. Set response.success to whether text
        # was available, and response.message to the text or "waiting for /odom".
        response.success = False
        response.message = "Complete TODO 7"
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
