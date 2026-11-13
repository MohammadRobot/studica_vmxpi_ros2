#!/usr/bin/env python3
"""Two actual Gazebo robots; run only in a loopback-only network namespace."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from test_runtime_contract import stop_process


class Probe:
    def __init__(self, domain):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = rclpy.create_node(f'isolation_probe_{domain}', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.domain = domain
        self.latest = {}
        self.counts = {}
        self.markers = set()
        self.commands = []
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, kind, qos in [('/odom', Odometry, qos_profile_sensor_data),
                                 ('/scan', LaserScan, qos_profile_sensor_data),
                                 ('/clock', Clock, qos_profile_sensor_data),
                                 ('/tf', TFMessage, qos_profile_sensor_data),
                                 ('/robot/state', String, durable)]:
            def receive(message, key=topic):
                self.latest[key] = message
                self.counts[key] = self.counts.get(key, 0) + 1
            self.node.create_subscription(kind, topic, receive, qos)
        self.node.create_subscription(String, '/isolation_marker',
                                      lambda m: self.markers.add(m.data), 10)
        self.node.create_subscription(Twist, '/cmd_vel',
                                      lambda m: self.commands.append(m.linear.x), 10)
        self.marker = self.node.create_publisher(String, '/isolation_marker', 10)
        self.command = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.arm = self.node.create_client(Trigger, '/robot/arm')
        self.disarm = self.node.create_client(Trigger, '/robot/disarm')

    def position(self):
        p = self.latest['/odom'].pose.pose.position
        return p.x, p.y

    def ready(self):
        return (all(key in self.latest for key in ['/odom', '/scan', '/clock', '/tf', '/robot/state'])
                and self.latest['/robot/state'].data == 'READY_DISARMED'
                and self.arm.service_is_ready())

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    links = json.loads(subprocess.check_output(['ip', '-j', 'link']))
    if ({link['ifname'] for link in links} != {'lo'}
            or not os.environ.get('STUDICA_PARENT_NETNS')
            or os.readlink('/proc/self/ns/net') == os.environ['STUDICA_PARENT_NETNS']):
        raise RuntimeError('requires a separate network namespace with only loopback')
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    processes, logs, probes = [], [], []
    report = {'passed': False, 'domains': [20, 21], 'phases': []}

    def spin(seconds, moving=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for index, probe in enumerate(probes):
                probe.marker.publish(String(data=str(probe.domain)))
                if index == moving:
                    cmd = Twist()
                    cmd.linear.x = 0.10
                    probe.command.publish(cmd)
                probe.executor.spin_once(timeout_sec=0.01)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('a simulator launch exited; inspect launch logs')
            time.sleep(0.01)

    try:
        for domain in report['domains']:
            subprocess.run([sys.executable, str(root / 'scripts/configure_cyclonedds.py'),
                            'sim', '--name', f'sim_{domain}', '--domain-id', str(domain),
                            '--output-dir', str(args.output)], check=True, capture_output=True)
            env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='0',
                       RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', GZ_VERSION='harmonic',
                       GZ_PARTITION=f'studica_test_{os.getpid()}_{domain}',
                       CYCLONEDDS_URI=(args.output / f'cyclonedds_sim_{domain}.xml').as_uri(),
                       ROS_LOG_DIR=str(args.output / f'ros_{domain}'),
                       LIBGL_ALWAYS_SOFTWARE='1', QT_QPA_PLATFORM='offscreen')
            log = (args.output / f'launch_{domain}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(
                ['ros2', 'launch', 'studica_vmxpi_ros2', 'sim.launch.py',
                 'gui:=false', 'gz_headless:=true', 'use_joystick:=false', 'use_camera:=false'],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            os.environ['CYCLONEDDS_URI'] = env['CYCLONEDDS_URI']
            os.environ['RMW_IMPLEMENTATION'] = env['RMW_IMPLEMENTATION']
            probes.append(Probe(domain))
        deadline = time.monotonic() + 100
        while not all(p.ready() for p in probes):
            if time.monotonic() > deadline:
                raise RuntimeError('both simulations did not reach ready-disarmed: ' + str([
                    (p.domain, p.counts, getattr(p.latest.get('/robot/state'), 'data', None))
                    for p in probes]))
            spin(0.5)
        spin(3)
        print('Both Gazebo robots ready; checking graph ownership', flush=True)
        ownership = {}
        for topic in ['/scan', '/odom', '/clock', '/tf']:
            gids = [set(bytes(e.endpoint_gid).hex() for e in p.node.get_publishers_info_by_topic(topic))
                    for p in probes]
            assert all(gids) and gids[0].isdisjoint(gids[1]), (topic, gids)
            if topic != '/tf':
                assert all(len(g) == 1 for g in gids), (topic, gids)
            ownership[topic] = [sorted(g) for g in gids]
        report['publisher_gids'] = ownership
        futures = [p.arm.call_async(Trigger.Request()) for p in probes]
        deadline = time.monotonic() + 10
        while not all(f.done() for f in futures) and time.monotonic() < deadline:
            spin(0.1)
        assert all(f.done() and f.result().success for f in futures), 'simulation arm failed'
        for moving in [0, 1]:
            before = [p.position() for p in probes]
            counts = [dict(p.counts) for p in probes]
            for p in probes:
                p.commands.clear()
            spin(5, moving=moving)
            delta = [math.dist(before[i], p.position()) for i, p in enumerate(probes)]
            assert delta[moving] > 0.03, ('positive motion control failed', delta)
            assert delta[1 - moving] < 0.02, ('uncommanded robot moved', delta)
            assert len(probes[moving].commands) > 20, 'own command not observed'
            assert probes[1 - moving].commands == [], 'command crossed domain'
            for i, p in enumerate(probes):
                assert p.markers == {str(p.domain)}, ('marker crossover', p.markers)
                for topic in ['/odom', '/scan', '/clock', '/tf']:
                    assert p.counts[topic] > counts[i][topic] + 2, ('stale telemetry', topic)
            report['phases'].append({'moving_domain': probes[moving].domain,
                                     'displacement_m': delta,
                                     'command_counts': [len(p.commands) for p in probes]})
            print(report['phases'][-1], flush=True)
            probes[moving].command.publish(Twist())
            probes[moving].disarm.call_async(Trigger.Request())
            spin(2)
        report['markers_seen'] = [sorted(p.markers) for p in probes]
        report['telemetry_counts'] = [p.counts for p in probes]
        report['passed'] = True
    finally:
        for p in probes:
            p.command.publish(Twist())
            p.disarm.call_async(Trigger.Request())
        for process in processes:
            stop_process(process)
            # Reap any remaining children in the exclusively owned launch group.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for p in probes:
            p.close()
        for log in logs:
            log.close()
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS: two concurrent Gazebo robots isolate telemetry and commands', flush=True)


if __name__ == '__main__':
    main()
