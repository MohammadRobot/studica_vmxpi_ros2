"""Read-only health evidence for the minimal runtime; never authorizes motion."""

import math
import time


class MinimalHealth:
    """Use monotonic receive times, unique publishers and repeated state evidence."""

    def __init__(self):
        self.samples = {}
        self.state_count = 0

    def receive(self, name, value, now):
        previous = self.samples.get(name)
        self.samples[name] = (value, now)
        if name == 'state':
            consecutive = previous is not None and 0 <= now - previous[1] <= 1.5
            self.state_count = (self.state_count + 1 if consecutive else 1) \
                if value == 'READY_DISARMED' else 0

    def evaluate(self, now, publishers):
        for name, budget in [('state', 1.5), ('motors', 0.5), ('odom', 0.5)]:
            if publishers.get(name) != 1:
                return False, f'{name}: requires one publisher'
            if name not in self.samples:
                return False, f'{name}: unavailable'
            age = now - self.samples[name][1]
            if not math.isfinite(age) or not 0 <= age <= budget:
                return False, f'{name}: stale'
        if self.samples['state'][0] != 'READY_DISARMED' or self.state_count < 2:
            return False, 'requires repeated READY_DISARMED state'
        motors = self.samples['motors'][0]
        if motors.fault_latched or not motors.pid_supported or motors.pid_type != 2:
            return False, 'motor fault or unexpected PID mode'
        if len(motors.motors) != 4 or {m.motor_channel for m in motors.motors} != {0, 1, 2, 3}:
            return False, 'requires four unique motor channels'
        for motor in motors.motors:
            if (not motor.encoder_fresh or motor.health_level != 0
                    or not math.isfinite(motor.feedback_age_sec)
                    or not 0 <= motor.feedback_age_sec <= 0.5):
                return False, 'motor feedback is unhealthy'
            for value in (motor.target_velocity_rad_s, motor.measured_velocity_rad_s):
                if not math.isfinite(value) or abs(value) > 0.01:
                    return False, 'motors are not stationary'
        odom = self.samples['odom'][0]
        if odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_link':
            return False, 'unexpected odometry frames'
        values = [odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                  odom.twist.twist.angular.z]
        if any(not math.isfinite(value) or abs(value) > 0.01 for value in values):
            return False, 'odometry is not stationary'
        return True, 'fresh stationary READY_DISARMED evidence; not physical qualification'


def observe(timeout):
    """Subscribe only; intentionally exclude transient history from readiness."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from studica_robot_monitor.msg import MotorTelemetryArray

    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be positive and finite')
    rclpy.init()
    node = Node('studica_minimal_readiness')
    health = MinimalHealth()
    topics = {'state': '/robot/state', 'motors': '/robot_status/motors', 'odom': '/odom'}
    try:
        for name, message_type in [('state', String), ('motors', MotorTelemetryArray),
                                   ('odom', Odometry)]:
            def receive(message, name=name):
                health.receive(name, message.data if name == 'state' else message,
                               time.monotonic())
            node.create_subscription(message_type, topics[name], receive, qos_profile_sensor_data)
        deadline = time.monotonic() + timeout
        reason = 'no observations'
        client = AsyncParameterClient(node, '/safety_supervisor')
        parameter_names = ['hardware_mode', 'allow_software_arm', 'control_source',
                           'input_cmd_vel_topic', 'output_cmd_vel_topic',
                           'command_timeout_sec', 'maintenance_lock_file']
        parameters = None
        parameters_at = 0.0
        future = None
        requested_at = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if future is not None and future.done():
                try:
                    parameters = future.result().values
                except Exception:
                    parameters = None
                # Bound the whole request, not just time since a delayed reply.
                parameters_at = requested_at
                future = None
            if future is None and now - parameters_at > 0.25 and client.services_are_ready():
                future = client.get_parameters(parameter_names)
                requested_at = now
            ready, reason = health.evaluate(
                time.monotonic(), {k: node.count_publishers(v) for k, v in topics.items()})
            if ready:
                expected_types = [1, 1, 4, 4, 4, 3, 4]
                if (parameters is None or now - parameters_at > 0.5
                        or [p.type for p in parameters] != expected_types):
                    reason = 'supervisor parameter evidence unavailable or stale'
                    continue
                if not (parameters[0].bool_value and not parameters[1].bool_value
                        and parameters[2].string_value == 'application'
                        and parameters[3].string_value == '/cmd_vel'
                        and parameters[4].string_value == '/robot_base_controller/cmd_vel'
                        and 0 < parameters[5].double_value <= 0.25
                        and parameters[6].string_value == '/var/lib/studica/activation-in-progress'):
                    reason = 'supervisor runtime contract mismatch'
                    continue
                return True, reason
        return False, reason
    finally:
        node.destroy_node()
        rclpy.shutdown()
