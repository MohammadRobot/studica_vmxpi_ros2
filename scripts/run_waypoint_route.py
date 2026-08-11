#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Preview or explicitly start a Nav2 FollowWaypoints route."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import yaml


def _default_route() -> Path:
    return Path(
        get_package_share_directory("studica_vmxpi_ros2"),
        "config",
        "waypoints",
        "office_loop.yaml",
    )


def _arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--route",
        type=Path,
        default=_default_route(),
        help="Waypoint YAML file (default: bundled office loop)",
    )
    parser.add_argument(
        "--start",
        action="store_true",
        help="Send the route; without this flag the command only previews it",
    )
    parser.add_argument(
        "--server-timeout",
        type=float,
        default=15.0,
        help="Seconds to wait for /follow_waypoints",
    )
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def _load_route(path: Path) -> tuple[str, list[dict[str, float | str]]]:
    route = yaml.safe_load(path.expanduser().read_text(encoding="utf-8"))
    if not isinstance(route, dict):
        raise ValueError("route YAML root must be a mapping")
    frame_id = route.get("frame_id", "map")
    waypoints = route.get("waypoints")
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError("frame_id must be a non-empty string")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("waypoints must be a non-empty list")

    validated = []
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            raise ValueError(f"waypoint {index} must be a mapping")
        try:
            name = str(waypoint.get("name", f"waypoint_{index + 1}"))
            x = float(waypoint["x"])
            y = float(waypoint["y"])
            yaw = float(waypoint.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"waypoint {index} requires numeric x, y, and optional yaw"
            ) from error
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError(f"waypoint {index} contains a non-finite value")
        validated.append({"name": name, "x": x, "y": y, "yaw": yaw})
    return frame_id, validated


def _pose(node: Node, frame_id: str, waypoint: dict[str, float | str]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(waypoint["x"])
    pose.pose.position.y = float(waypoint["y"])
    yaw = float(waypoint["yaw"])
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class WaypointRoute(Node):
    """One explicit FollowWaypoints action client."""

    def __init__(self) -> None:
        super().__init__("studica_waypoint_route")
        self.client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self.current_waypoint = -1

    def feedback(self, message) -> None:
        """Log each waypoint transition once."""
        current = int(message.feedback.current_waypoint)
        if current != self.current_waypoint:
            self.current_waypoint = current
            self.get_logger().info(f"Navigating to waypoint {current + 1}")


def main(argv: list[str] | None = None) -> int:
    command_line = sys.argv if argv is None else argv
    rclpy.init(args=command_line)
    node = WaypointRoute()
    goal_handle = None
    try:
        arguments = _arguments(command_line)
        if arguments.server_timeout <= 0.0:
            raise ValueError("--server-timeout must be greater than zero")
        frame_id, waypoints = _load_route(arguments.route)
        print(f"Route: {arguments.route.expanduser().resolve()}")
        print(f"Frame: {frame_id}")
        for index, waypoint in enumerate(waypoints, start=1):
            print(
                f"  {index}. {waypoint['name']}: x={waypoint['x']:.2f}, "
                f"y={waypoint['y']:.2f}, yaw={waypoint['yaw']:.2f}"
            )

        if not arguments.start:
            print("Preview only; add --start to send this route.")
            return 0

        if not node.client.wait_for_server(timeout_sec=arguments.server_timeout):
            node.get_logger().error("/follow_waypoints is unavailable")
            return 1

        goal = FollowWaypoints.Goal()
        goal.poses = [_pose(node, frame_id, waypoint) for waypoint in waypoints]
        send_future = node.client.send_goal_async(
            goal, feedback_callback=node.feedback
        )
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            node.get_logger().error("Waypoint route was rejected")
            return 1

        node.get_logger().info(f"Route accepted with {len(waypoints)} waypoints")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped_result = result_future.result()
        missed = list(wrapped_result.result.missed_waypoints)
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or missed:
            node.get_logger().error(
                f"Route did not complete successfully; missed waypoints: {missed}"
            )
            return 1
        node.get_logger().info("Waypoint route completed successfully")
        return 0
    except (OSError, ValueError, yaml.YAMLError) as error:
        node.get_logger().error(str(error))
        return 2
    except KeyboardInterrupt:
        if goal_handle is not None:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=3.0)
        node.get_logger().warning("Waypoint route cancelled")
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
