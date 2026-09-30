#!/usr/bin/env python3
"""Sole /cmd_vel publisher: cancel navigation on L1, never resume an old goal."""
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from studica_robot_platform.manual_override import OverrideGate


class NavigationOverride(Node):
    def __init__(self):
        super().__init__('navigation_override')
        self.allow_navigation = self.declare_parameter('allow_navigation', True).value
        speeds = []
        for name, default, limit in [('linear_speed', .20, .30), ('angular_speed', .60, .90),
                                     ('turbo_linear_speed', .20, .30), ('turbo_angular_speed', .60, .90)]:
            value = float(self.declare_parameter(name, default).value)
            if not math.isfinite(value) or not 0 < value <= limit:
                raise ValueError(f'{name} must be positive and <= {limit}')
            speeds.append(value)
        self.gate = OverrideGate(*speeds)
        self.gate.after = self.stamp()
        self.statuses = {}
        self.cancel_futures = {}
        self.last_cancel = 0.0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.state = self.create_publisher(String, '/robot/control_owner', 1)
        self.create_subscription(Joy, '/joy', self.joy, 1)
        self.create_subscription(Twist, '/cmd_vel/navigation', self.nav, 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cancel_clients = {}
        for action in ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints'):
            self.cancel_clients[action] = self.create_client(CancelGoal, f'/{action}/_action/cancel_goal')
            self.create_subscription(GoalStatusArray, f'/{action}/_action/status',
                                     lambda m, a=action: self.status(a, m), qos)
        self.create_timer(0.05, self.tick)

    def stamp(self):
        return self.get_clock().now().nanoseconds / 1e9

    def cancel(self):
        for name, client in self.cancel_clients.items():
            old = self.cancel_futures.get(name)
            if client.service_is_ready() and (old is None or old.done()):
                self.cancel_futures[name] = client.call_async(CancelGoal.Request())
        self.last_cancel = time.monotonic()

    def joy(self, msg):
        if self.gate.joy(time.monotonic(), self.stamp(), msg.buttons, msg.axes):
            self.pub.publish(Twist())
            self.cancel()

    def nav(self, msg):
        if self.gate.mode == 'NAVIGATION':
            self.gate.navigation(time.monotonic(), msg.linear.x, msg.angular.z)

    def status(self, action, msg):
        if not self.allow_navigation:
            return
        self.statuses[action] = [
            (action + bytes(s.goal_info.goal_id.uuid).hex(),
             s.goal_info.stamp.sec + s.goal_info.stamp.nanosec / 1e9, s.status)
            for s in msg.status_list]
        self.gate.goals([s for group in self.statuses.values() for s in group], self.stamp())

    def tick(self):
        now = time.monotonic()
        before = self.gate.mode
        x, yaw = self.gate.output(now, self.stamp())
        if self.count_publishers('/joy') > 1 or self.count_publishers('/cmd_vel/navigation') > 1:
            self.gate.halt(self.stamp())
            self.gate.released = False
            x, yaw = 0.0, 0.0
        if self.count_publishers('/cmd_vel') != 1:
            self.gate.halt(self.stamp())
            x, yaw = 0.0, 0.0
        if ((self.gate.mode != 'NAVIGATION' and now - self.last_cancel > 0.5)
                or (before == 'MANUAL' and self.gate.mode != before)):
            self.cancel()
        msg = Twist()
        msg.linear.x, msg.angular.z = x, yaw
        self.pub.publish(msg)
        self.state.publish(String(data=self.gate.mode))


def main():
    rclpy.init()
    node = NavigationOverride()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
