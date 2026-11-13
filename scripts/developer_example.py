#!/usr/bin/env python3
"""Read odometry; optionally send one short, explicitly requested velocity pulse."""

import argparse
import math
import time


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--publish-motion', action='store_true')
    parser.add_argument('--speed', type=float, default=0.05)
    parser.add_argument('--duration', type=float, default=1.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or not 0 < args.speed <= 0.05:
        parser.error('speed must be in (0, 0.05] m/s')
    if not math.isfinite(args.duration) or not 0 < args.duration <= 2.0:
        parser.error('duration must be in (0, 2] seconds')
    return args


def main():
    args = arguments()
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = Node('studica_python_example')
    node.create_subscription(Odometry, '/odom', lambda msg: node.get_logger().info(
        f'x={msg.pose.pose.position.x:.3f} m vx={msg.twist.twist.linear.x:.3f} m/s',
        throttle_duration_sec=1.0),
        qos_profile_sensor_data)
    publisher = node.create_publisher(Twist, '/cmd_vel', 1) if args.publish_motion else None
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < 1.0 + args.duration + 0.5:
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = time.monotonic() - started
            if publisher is not None:
                if node.count_publishers('/cmd_vel') != 1:
                    raise RuntimeError('another command publisher is present; stop that session first')
                message = Twist()
                if 1.0 <= elapsed < 1.0 + args.duration:
                    message.linear.x = args.speed
                publisher.publish(message)
    except KeyboardInterrupt:
        pass
    finally:
        if publisher is not None and rclpy.ok():
            publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
