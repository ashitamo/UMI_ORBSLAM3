#!/usr/bin/env python3
"""Interactively validate RGB-D topics and record a consistently named ROS 2 bag."""

import argparse
from collections import defaultdict
from datetime import datetime
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
import yaml

from umi_dataset_tools.project_paths import data_root
from umi_dataset_tools.qos import reliable_image_qos


INFRA_TOPIC = '/camera/camera/infra1/image_rect_raw'
DEPTH_TOPIC = '/camera/camera/depth/image_rect_raw'
DEPTH_COMPRESSED_TOPIC = '/camera/camera/depth/image_rect_raw/compressedDepth'
COLOR_TOPIC = '/camera/camera/color/image_raw/compressed'
IMU_TOPIC = '/imu/data'
COLOR_INFO_TOPIC = '/camera/camera/color/camera_info'
DEPTH_INFO_TOPIC = '/camera/camera/depth/camera_info'
INFRA_INFO_TOPIC = '/camera/camera/infra1/camera_info'
DEPTH_TO_TRACKING_TOPIC = '/camera/camera/extrinsics/depth_to_infra1'
DEPTH_TO_COLOR_TOPIC = '/camera/camera/extrinsics/depth_to_color'
TF_TOPIC = '/tf'
TF_STATIC_TOPIC = '/tf_static'

IMAGE_TOPICS = (INFRA_TOPIC, DEPTH_TOPIC, COLOR_TOPIC)
STATIC_TOPICS = (DEPTH_TO_TRACKING_TOPIC, DEPTH_TO_COLOR_TOPIC, TF_STATIC_TOPIC)
REQUIRED_TOPICS = (
    *IMAGE_TOPICS,
    IMU_TOPIC,
    COLOR_INFO_TOPIC,
    DEPTH_INFO_TOPIC,
    INFRA_INFO_TOPIC,
    DEPTH_TO_TRACKING_TOPIC,
    DEPTH_TO_COLOR_TOPIC,
)
WARNING_TOPICS = (TF_TOPIC, TF_STATIC_TOPIC)
RECORD_TOPICS = (*REQUIRED_TOPICS, *WARNING_TOPICS)


def configure_depth_topic(depth_topic, sensor='rgbd-inertial'):
    global DEPTH_TOPIC, IMAGE_TOPICS, REQUIRED_TOPICS, RECORD_TOPICS
    DEPTH_TOPIC = depth_topic
    IMAGE_TOPICS = (INFRA_TOPIC, DEPTH_TOPIC, COLOR_TOPIC)
    REQUIRED_TOPICS = (
        *IMAGE_TOPICS,
        *((IMU_TOPIC,) if sensor == 'rgbd-inertial' else ()),
        COLOR_INFO_TOPIC,
        DEPTH_INFO_TOPIC,
        INFRA_INFO_TOPIC,
        DEPTH_TO_TRACKING_TOPIC,
        DEPTH_TO_COLOR_TOPIC,
    )
    RECORD_TOPICS = (*REQUIRED_TOPICS, *WARNING_TOPICS)


def format_bytes(size):
    value = float(size)
    for unit in ('B', 'KiB', 'MiB', 'GiB', 'TiB'):
        if value < 1024.0 or unit == 'TiB':
            return f'{value:.2f} {unit}'
        value /= 1024.0


def next_bag_name(output_root, prefix):
    pattern = re.compile(rf'^{re.escape(prefix)}_(\d+)$')
    indices = []
    if output_root.is_dir():
        for path in output_root.iterdir():
            match = pattern.match(path.name)
            if match:
                indices.append(int(match.group(1)))
    return f'{prefix}_{max(indices, default=0) + 1:02d}'


def stamp_ns(message):
    header = getattr(message, 'header', None)
    if header is None:
        return None
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def transient_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class TopicHealthMonitor(Node):
    def __init__(self, topic_types):
        super().__init__('umi_dataset_topic_health_monitor')
        self.counts = defaultdict(int)
        self.first_arrival = {}
        self.last_arrival = {}
        self.stamps = {topic: set() for topic in IMAGE_TOPICS}
        self._topic_subscriptions = []
        for topic in RECORD_TOPICS:
            type_names = topic_types.get(topic)
            if not type_names:
                continue
            message_type = get_message(type_names[0])
            if topic in STATIC_TOPICS:
                qos = transient_qos()
            elif topic in IMAGE_TOPICS:
                qos = reliable_image_qos()
            else:
                qos = qos_profile_sensor_data
            subscription = self.create_subscription(
                message_type,
                topic,
                lambda message, selected_topic=topic: self.callback(
                    selected_topic, message
                ),
                qos,
            )
            self._topic_subscriptions.append(subscription)

    def callback(self, topic, message):
        now = time.monotonic()
        self.counts[topic] += 1
        self.first_arrival.setdefault(topic, now)
        self.last_arrival[topic] = now
        if topic in self.stamps:
            timestamp = stamp_ns(message)
            if timestamp is not None:
                self.stamps[topic].add(timestamp)

    def rate(self, topic):
        count = self.counts[topic]
        duration = self.last_arrival.get(topic, 0.0) - self.first_arrival.get(
            topic, 0.0
        )
        return (count - 1) / duration if count > 1 and duration > 0.0 else 0.0


def discover_topic_types(node, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    topic_types = {}
    while time.monotonic() < deadline:
        topic_types = dict(node.get_topic_names_and_types())
        if all(topic in topic_types for topic in REQUIRED_TOPICS):
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    return topic_types


def run_health_check(duration_sec, minimum_image_hz, minimum_imu_hz):
    rclpy.init()
    discovery_node = Node('umi_dataset_topic_discovery')
    topic_types = discover_topic_types(discovery_node)
    discovery_node.destroy_node()

    missing_required = [topic for topic in REQUIRED_TOPICS if topic not in topic_types]
    missing_warnings = [topic for topic in WARNING_TOPICS if topic not in topic_types]
    if missing_required:
        rclpy.shutdown()
        print('\n[HEALTH] Missing required topics:')
        for topic in missing_required:
            print(f'  [FAIL] {topic}')
        return False

    monitor = TopicHealthMonitor(topic_types)
    print(f'\n[HEALTH] Monitoring topics for {duration_sec:.1f} seconds...')
    start_time = time.monotonic()
    deadline = start_time + duration_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(monitor, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic())))

    passed = True
    print('\n[HEALTH] Topic report')
    rate_topics = [(topic, minimum_image_hz) for topic in IMAGE_TOPICS]
    if IMU_TOPIC in REQUIRED_TOPICS:
        rate_topics.append((IMU_TOPIC, minimum_imu_hz))
    for topic, minimum_hz in rate_topics:
        count = monitor.counts[topic]
        rate = monitor.rate(topic)
        healthy = count >= 2 and rate >= minimum_hz
        passed &= healthy
        print(
            f'  [{"OK" if healthy else "FAIL"}] {topic}: '
            f'{rate:.1f} Hz, messages={count}'
        )

    for topic in (
        COLOR_INFO_TOPIC,
        DEPTH_INFO_TOPIC,
        INFRA_INFO_TOPIC,
        DEPTH_TO_TRACKING_TOPIC,
        DEPTH_TO_COLOR_TOPIC,
    ):
        count = monitor.counts[topic]
        healthy = count > 0
        passed &= healthy
        print(f'  [{"OK" if healthy else "FAIL"}] {topic}: messages={count}')

    for topic in WARNING_TOPICS:
        count = monitor.counts[topic]
        status = 'OK' if count else 'WARN'
        suffix = '' if topic not in missing_warnings else ' (not discovered)'
        print(f'  [{status}] {topic}: messages={count}{suffix}')

    common_stamps = set.intersection(
        *(monitor.stamps[topic] for topic in IMAGE_TOPICS)
    )
    sync_healthy = len(common_stamps) > 0
    passed &= sync_healthy
    print(
        f'  [{"OK" if sync_healthy else "FAIL"}] exact RGB-D synchronized '
        f'timestamps: {len(common_stamps)}'
    )

    monitor.destroy_node()
    rclpy.shutdown()
    return passed


def wait_with_status(seconds, label):
    if seconds <= 0:
        return
    deadline = time.monotonic() + seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        print(f'\r{label}: {remaining:4.1f} s ', end='', flush=True)
        time.sleep(min(0.1, remaining))
    print(f'\r{label}: done      ')


def stop_recorder(process):
    if process.poll() is not None:
        return process.returncode
    os.killpg(process.pid, signal.SIGINT)
    try:
        return process.wait(timeout=30.0)
    except subprocess.TimeoutExpired:
        print('[RECORD] Recorder did not stop after SIGINT; sending SIGTERM')
        os.killpg(process.pid, signal.SIGTERM)
        return process.wait(timeout=10.0)


def print_bag_summary(bag_path, wall_duration):
    size = sum(path.stat().st_size for path in bag_path.rglob('*') if path.is_file())
    print('\n[SUMMARY]')
    print(f'  Bag: {bag_path}')
    print(f'  Wall duration: {wall_duration:.2f} s')
    print(f'  Size: {format_bytes(size)}')
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        print('  [WARN] metadata.yaml was not generated')
        return
    with metadata_path.open() as stream:
        metadata = (yaml.safe_load(stream) or {}).get('rosbag2_bagfile_information', {})
    print(f'  Messages: {int(metadata.get("message_count", 0)):,}')
    for item in metadata.get('topics_with_message_count', []):
        topic = item.get('topic_metadata', {}).get('name', '<unknown>')
        print(f'    {topic}: {int(item.get("message_count", 0)):,}')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--sensor', choices=('rgbd', 'rgbd-inertial'), default='rgbd-inertial',
        help='Sensor mode; rgbd skips IMU checks and recording (default: rgbd-inertial)',
    )
    parser.add_argument('--output-root', type=Path, default=data_root() / 'bags')
    depth_group = parser.add_mutually_exclusive_group()
    depth_group.add_argument('--depth-topic', default=None)
    depth_group.add_argument(
        '--raw-depth',
        action='store_true',
        help=f'Record legacy raw depth from {DEPTH_TOPIC}',
    )
    depth_group.add_argument(
        '--compressed-depth',
        action='store_true',
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        '--prefix',
        default=None,
        help='Automatic name prefix; default is dataMMDD using the current date',
    )
    parser.add_argument('--name', default=None, help='Exact bag name instead of automatic numbering')
    parser.add_argument('--health-seconds', type=float, default=3.0)
    parser.add_argument('--stationary-seconds', type=float, default=5.0)
    parser.add_argument('--countdown', type=int, default=3)
    parser.add_argument('--max-cache-size', type=int, default=1_000_000_000)
    parser.add_argument('--min-image-hz', type=float, default=20.0)
    parser.add_argument('--min-imu-hz', type=float, default=50.0)
    parser.add_argument('--force', action='store_true', help='Record even if health checks fail')
    parser.add_argument('-y', '--yes', action='store_true', help='Skip confirmation prompt')
    return parser.parse_args()


def main():
    args = parse_args()
    if args.raw_depth:
        selected_depth_topic = DEPTH_TOPIC
    elif args.depth_topic:
        selected_depth_topic = args.depth_topic
    else:
        selected_depth_topic = DEPTH_COMPRESSED_TOPIC
    configure_depth_topic(selected_depth_topic, args.sensor)
    if args.health_seconds <= 0.0:
        raise SystemExit('--health-seconds must be positive')
    if args.stationary_seconds < 0.0 or args.countdown < 0:
        raise SystemExit('stationary time and countdown must be non-negative')
    if args.max_cache_size <= 0:
        raise SystemExit('--max-cache-size must be positive')

    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    prefix = args.prefix or datetime.now().strftime('data%m%d')
    bag_name = args.name or next_bag_name(output_root, prefix)
    bag_path = output_root / bag_name
    if bag_path.exists():
        raise SystemExit(f'Refusing to overwrite existing bag: {bag_path}')

    print('=' * 72)
    print('UMI RGB-D dataset collector')
    print(f'Sensor: {args.sensor}')
    print(f'Output: {bag_path}')
    print('=' * 72)
    health_passed = run_health_check(
        args.health_seconds, args.min_image_hz, args.min_imu_hz
    )
    if not health_passed and not args.force:
        raise SystemExit('\nHealth check failed; fix topics or rerun with --force')
    if not health_passed:
        print('\n[WARN] Health check failed, continuing because --force was used')

    if not args.yes:
        answer = input(f'\nPress Enter to record {bag_name}, or type q to cancel: ').strip().lower()
        if answer in ('q', 'quit', 'n', 'no'):
            print('Cancelled')
            return 0

    command = [
        'ros2', 'bag', 'record',
        '--storage', 'mcap',
        '--storage-preset-profile', 'fastwrite',
        '--max-cache-size', str(args.max_cache_size),
        '--output', str(bag_path),
        *RECORD_TOPICS,
    ]
    print(f'\n[RECORD] Starting {bag_name}')
    process = subprocess.Popen(command, start_new_session=True)
    start_time = time.monotonic()
    try:
        time.sleep(1.0)
        if process.poll() is not None:
            raise RuntimeError(f'ros2 bag record exited with code {process.returncode}')
        wait_with_status(args.stationary_seconds, 'Keep UMI completely stationary')
        print('Prepare to move')
        for value in range(args.countdown, 0, -1):
            print(value, flush=True)
            time.sleep(1.0)
        print('GO! Press Ctrl+C when the demonstration is finished.', flush=True)
        process.wait()
    except KeyboardInterrupt:
        print('\n[RECORD] Stopping recorder and finalizing metadata...')
    finally:
        return_code = stop_recorder(process)
    wall_duration = time.monotonic() - start_time
    if return_code not in (0, -signal.SIGINT):
        print(f'[WARN] ros2 bag record exited with code {return_code}')
    print_bag_summary(bag_path, wall_duration)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
