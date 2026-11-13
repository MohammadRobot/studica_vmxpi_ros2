#!/usr/bin/env python3
"""Actual dual SLAM/Nav2 acceptance; isolated loopback network required."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from PIL import Image
import yaml

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from test_two_simulations import Probe
from test_runtime_contract import stop_process


def digest_map(message):
    return hashlib.sha256(bytes((x + 1 for x in message.data))).hexdigest()


def saved_map_digest(path):
    metadata = yaml.safe_load(path.read_text())
    with Image.open(path.parent / metadata['image']) as original:
        image = original.convert('L')
    values = []
    for y in range(image.height - 1, -1, -1):
        for x in range(image.width):
            occupied = (255 - image.getpixel((x, y))) / 255
            values.append(101 if occupied > metadata['occupied_thresh'] else
                          1 if occupied < metadata['free_thresh'] else 0)
    return hashlib.sha256(bytes(values)).hexdigest()


class MappingProbe(Probe):
    def __init__(self, domain, navigation):
        super().__init__(domain)
        durable = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.maps, self.tf_marks, self.map_odom, self.goal_ids = [], set(), [], set()
        self.node.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.node.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.tf_pub = self.node.create_publisher(TFMessage, '/tf', 10)
        self.initial = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.action = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                      self.on_status, durable)
        if navigation:
            self.node.destroy_publisher(self.command)
            self.command = None

    def on_map(self, message):
        self.latest['/map'] = message
        self.maps.append(digest_map(message))

    def on_tf(self, message):
        for transform in message.transforms:
            if transform.child_frame_id.startswith('isolation_'):
                self.tf_marks.add(transform.child_frame_id)
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                self.map_odom.append((transform.header.stamp.sec, transform.header.stamp.nanosec))

    def on_status(self, message):
        self.goal_ids.update(bytes(s.goal_info.goal_id.uuid).hex() for s in message.status_list)

    def mark_tf(self):
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = f'isolation_{self.domain}'
        transform.header.stamp = self.latest['/clock'].clock
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = float(self.domain)
        self.tf_pub.publish(TFMessage(transforms=[transform]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    links = json.loads(subprocess.check_output(['ip', '-j', 'link']))
    if ({x['ifname'] for x in links} != {'lo'} or not os.environ.get('STUDICA_PARENT_NETNS')
            or os.readlink('/proc/self/ns/net') == os.environ['STUDICA_PARENT_NETNS']):
        raise RuntimeError('requires a separate loopback-only network namespace')
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    report = {'passed': False, 'domains': [20, 21]}
    probes, processes, logs, environments = [], [], [], []

    def spin(seconds, moving=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for i, p in enumerate(probes):
                if '/clock' in p.latest:
                    p.mark_tf()
                if i == moving:
                    command = Twist()
                    command.linear.x = 0.10 if i == 0 else -0.10
                    p.command.publish(command)
                # Drain each context without letting high-rate clocks starve services.
                for _ in range(8):
                    p.executor.spin_once(timeout_sec=0.001)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('launch exited; inspect logs')
            time.sleep(0.015)

    def wait(predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError(reason)
            spin(0.1)

    def service(client):
        future = client.call_async(Trigger.Request())
        wait(future.done, 10, 'service timed out')
        assert future.result().success, future.result().message

    def launch(mode):
        for domain in [20, 21]:
            subprocess.run([sys.executable, str(root / 'scripts/configure_cyclonedds.py'),
                            'sim', '--name', f'sim_{domain}', '--domain-id', str(domain),
                            '--output-dir', str(args.output)], check=True, capture_output=True)
            env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='0',
                       RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', GZ_VERSION='harmonic',
                       GZ_PARTITION=f'studica_{os.getpid()}_{mode}_{domain}',
                       CYCLONEDDS_URI=(args.output / f'cyclonedds_sim_{domain}.xml').as_uri(),
                       ROS_LOG_DIR=str(args.output / f'ros_{mode}_{domain}'),
                       LIBGL_ALWAYS_SOFTWARE='1', QT_QPA_PLATFORM='offscreen')
            command = ['ros2', 'launch', 'studica_vmxpi_ros2', f'{mode}.launch.py',
                       'mode:=gz_sim', 'gui:=false', 'gz_headless:=true', 'use_joystick:=false']
            if mode == 'navigation':
                command += ['use_point_cloud:=false', f'map:={args.output}/map_{domain}.yaml']
            stream = (args.output / f'{mode}_{domain}.log').open('w')
            logs.append(stream)
            processes.append(subprocess.Popen(command, env=env, stdout=stream,
                                              stderr=subprocess.STDOUT, start_new_session=True))
            environments.append(env)
            os.environ['CYCLONEDDS_URI'] = env['CYCLONEDDS_URI']
            os.environ['RMW_IMPLEMENTATION'] = env['RMW_IMPLEMENTATION']
            probes.append(MappingProbe(domain, mode == 'navigation'))
        wait(lambda: all(p.ready() and p.maps for p in probes), 110,
             f'{mode}: readiness/map timed out')
        print(f'{mode}: both robots ready, both maps received', flush=True)

    def ownership():
        result = {}
        for topic in ['/map', '/tf', '/clock']:
            gids = [set(bytes(e.endpoint_gid).hex() for e in p.node.get_publishers_info_by_topic(topic))
                    for p in probes]
            assert all(gids) and gids[0].isdisjoint(gids[1]), (topic, gids)
            if topic != '/tf':
                assert all(len(x) == 1 for x in gids), (topic, gids)
            result[topic] = [sorted(x) for x in gids]
        for p in probes:
            assert p.tf_marks == {f'isolation_{p.domain}'}, p.tf_marks
            assert len(set(p.map_odom[-100:])) > 2, 'map->odom not advancing'
        return result

    def stop():
        for p in probes:
            if p.command is not None:
                p.command.publish(Twist())
            if p.disarm.service_is_ready():
                p.disarm.call_async(Trigger.Request())
        for process in processes:
            stop_process(process)
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for p in probes:
            p.close()
        for stream in logs:
            stream.close()
        probes.clear()
        processes.clear()
        logs.clear()
        environments.clear()

    try:
        launch('mapping')
        for i, p in enumerate(probes):
            service(p.arm)
            before = [q.position() for q in probes]
            spin(5, moving=i)
            p.command.publish(Twist())
            service(p.disarm)
            spin(2)
            delta = [math.dist(before[j], q.position()) for j, q in enumerate(probes)]
            assert delta[i] > 0.03 and delta[1-i] < 0.02, delta
        spin(3)
        report['slam_publishers'] = ownership()
        report['slam_map_hashes'] = [p.maps[-1] for p in probes]
        report['slam_map_updates'] = [len(p.maps) for p in probes]
        assert all(len(p.maps) > 3 for p in probes)
        assert probes[0].maps[-1] != probes[1].maps[-1], 'maps not distinguishable'
        for i, p in enumerate(probes):
            with (args.output / f'save_{p.domain}.log').open('w') as stream:
                saver = subprocess.Popen(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                     '-f', str(args.output / f'map_{p.domain}'), '--ros-args',
                     '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=10.0'],
                    env=environments[i], stdout=stream, stderr=subprocess.STDOUT)
                try:
                    wait(lambda: saver.poll() is not None, 20, 'map save timeout')
                    assert saver.returncode == 0, 'map save failed'
                finally:
                    if saver.poll() is None:
                        saver.kill()
                        saver.wait()
            assert (args.output / f'map_{p.domain}.yaml').is_file()
        report['slam_passed'] = True
        print('SLAM: distinct maps saved; map/TF ownership isolated', flush=True)
        stop()
        launch('navigation')
        for p in probes:
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = p.latest['/clock'].clock
            pose.pose.pose.orientation.w = 1.0
            pose.pose.covariance[0] = pose.pose.covariance[7] = 0.05
            pose.pose.covariance[35] = 0.05
            p.initial.publish(pose)
        wait(lambda: all(p.map_odom and p.action.server_is_ready() for p in probes),
             30, 'AMCL/action readiness timed out')
        spin(3)
        report['navigation_publishers'] = ownership()
        hashes = [p.maps[-1] for p in probes]
        assert hashes[0] != hashes[1], 'navigation maps not distinguishable'
        report['navigation_map_hashes'] = hashes
        expected = [saved_map_digest(args.output / f'map_{p.domain}.yaml') for p in probes]
        assert hashes == expected, 'navigation loaded the wrong saved map'
        report['saved_map_grid_hashes'] = expected
        for p in probes:
            service(p.arm)
        goals, handles, results = [], [], []
        before = [p.position() for p in probes]
        for i, p in enumerate(probes):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = p.latest['/clock'].clock
            goal.pose.pose.position.x = 0.45 if i == 0 else -0.45
            goal.pose.pose.orientation.w = 1.0
            goals.append(p.action.send_goal_async(goal))
        wait(lambda: all(f.done() for f in goals), 15, 'goal acceptance timeout')
        for f in goals:
            assert f.result().accepted, 'goal rejected'
            handles.append(f.result())
            results.append(f.result().get_result_async())
        ids = [bytes(h.goal_id.uuid).hex() for h in handles]
        print('Navigation: two distinct goals accepted concurrently', flush=True)
        wait(lambda: all(f.done() for f in results), 90, 'navigation goals timed out')
        spin(2)
        statuses = [f.result().status for f in results]
        assert statuses == [4, 4], ('goals did not both succeed', statuses)
        for i, p in enumerate(probes):
            assert p.goal_ids == {ids[i]}, ('goal crossover', p.goal_ids, ids)
            assert set(p.maps) == {hashes[i]}, 'navigation map changed/crossed'
            assert math.dist(before[i], p.position()) > 0.1, 'goal caused no motion'
        report['final_navigation_publishers'] = ownership()
        report['tf_markers_seen'] = [sorted(p.tf_marks) for p in probes]
        report['goal_ids_seen'] = [sorted(p.goal_ids) for p in probes]
        report['goal_ids'] = ids
        report['goal_statuses'] = statuses
        report['final_odom_xy'] = [p.position() for p in probes]
        report['passed'] = True
        print('PASS: concurrent SLAM and navigation isolate maps, TF and goals', flush=True)
    finally:
        stop()
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
