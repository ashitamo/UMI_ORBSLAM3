#!/usr/bin/env python3
"""Check whether a bag startup contains an ORB-SLAM3-compatible static IMU window."""

import argparse
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


RGB_TOPIC = '/camera/camera/infra1/image_rect_raw'
DEPTH_TOPIC = '/camera/camera/depth/image_rect_raw'
IMU_TOPIC = '/imu/data'
MIN_WINDOW_SEC = 1.8
MIN_IMU_SAMPLES = 200
MAX_GYRO_STD_NORM = 0.01
MAX_ACCEL_STD_NORM = 0.15
MIN_GRAVITY_NORM = 8.5
MAX_GRAVITY_NORM = 11.0


def stamp_ns(message, bag_time_ns):
    header = getattr(message, 'header', None)
    if header is None:
        return int(bag_time_ns)
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def vector3(message):
    return [float(message.x), float(message.y), float(message.z)]


def window_stats(times_sec, gyros, accelerations, start_index, end_index):
    gyro_std_norm = float(np.linalg.norm(np.std(gyros[start_index:end_index], axis=0)))
    acceleration_window = accelerations[start_index:end_index]
    accel_std_norm = float(np.linalg.norm(np.std(acceleration_window, axis=0)))
    gravity_norm = float(np.linalg.norm(np.mean(acceleration_window, axis=0)))
    duration = float(times_sec[end_index - 1] - times_sec[start_index])
    passed = (
        end_index - start_index >= MIN_IMU_SAMPLES
        and duration >= MIN_WINDOW_SEC
        and gyro_std_norm < MAX_GYRO_STD_NORM
        and accel_std_norm < MAX_ACCEL_STD_NORM
        and MIN_GRAVITY_NORM < gravity_norm < MAX_GRAVITY_NORM
    )
    score = (
        gyro_std_norm / MAX_GYRO_STD_NORM
        + accel_std_norm / MAX_ACCEL_STD_NORM
        + abs(gravity_norm - 9.81) / 1.31
    )
    return passed, score, gyro_std_norm, accel_std_norm, gravity_norm, duration


def find_static_window(times_sec, gyros, accelerations):
    best = None
    for start_index in range(len(times_sec)):
        end_index = int(np.searchsorted(times_sec, times_sec[start_index] + MIN_WINDOW_SEC, side='left')) + 1
        if end_index > len(times_sec):
            break
        stats = window_stats(times_sec, gyros, accelerations, start_index, end_index)
        record = (start_index, end_index, *stats)
        if best is None or record[3] < best[3]:
            best = record
        if stats[0]:
            return record
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--storage-id', required=True)
    parser.add_argument('--startup-sec', type=float, default=6.0)
    parser.add_argument('--depth-topic', default=DEPTH_TOPIC)
    args = parser.parse_args()
    if args.startup_sec < MIN_WINDOW_SEC:
        parser.error(f'--startup-sec must be at least {MIN_WINDOW_SEC}')

    bag_path = args.bag.expanduser().resolve()
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=args.storage_id),
            rosbag2_py.ConverterOptions('', ''),
        )
    except Exception as error:
        print(f'[STARTUP CHECK] ERROR unable to open {bag_path}: {error}', file=sys.stderr)
        return 1

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required_topics = [RGB_TOPIC, args.depth_topic, IMU_TOPIC]
    missing_topics = [topic for topic in required_topics if topic not in topic_types]
    if missing_topics:
        print(f'[STARTUP CHECK] ERROR missing topics: {missing_topics}', file=sys.stderr)
        return 1

    message_types = {topic: get_message(topic_types[topic]) for topic in required_topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=required_topics))
    counts = {topic: 0 for topic in required_topics}
    imu_records = []
    first_bag_time_ns = None
    startup_ns = int(args.startup_sec * 1e9)

    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if first_bag_time_ns is None:
            first_bag_time_ns = int(bag_time_ns)
        if int(bag_time_ns) - first_bag_time_ns > startup_ns:
            break
        message = deserialize_message(serialized, message_types[topic])
        counts[topic] += 1
        if topic == IMU_TOPIC:
            imu_records.append((
                stamp_ns(message, bag_time_ns),
                vector3(message.angular_velocity),
                vector3(message.linear_acceleration),
            ))

    print(f'[STARTUP CHECK] bag={bag_path.name} interval={args.startup_sec:.1f}s')
    print(
        f'[STARTUP CHECK] rgb={counts[RGB_TOPIC]} depth={counts[args.depth_topic]} '
        f'imu={counts[IMU_TOPIC]}'
    )
    if counts[RGB_TOPIC] == 0 or counts[args.depth_topic] == 0 or len(imu_records) < MIN_IMU_SAMPLES:
        print('[STARTUP CHECK] ERROR insufficient startup RGB-D/IMU messages', file=sys.stderr)
        return 1

    imu_records.sort(key=lambda record: record[0])
    first_imu_timestamp = imu_records[0][0]
    times_sec = np.asarray([(record[0] - first_imu_timestamp) / 1e9 for record in imu_records])
    gyros = np.asarray([record[1] for record in imu_records], dtype=float)
    accelerations = np.asarray([record[2] for record in imu_records], dtype=float)
    best = find_static_window(times_sec, gyros, accelerations)
    if best is None:
        print('[STARTUP CHECK] WARN no complete 1.8s IMU window', file=sys.stderr)
        return 2

    start_index, end_index, passed, _, gyro_std, accel_std, gravity_norm, duration = best
    print(
        f'[STARTUP CHECK] best_window={times_sec[start_index]:.3f}..'
        f'{times_sec[end_index - 1]:.3f}s samples={end_index - start_index} duration={duration:.3f}s '
        f'gyroStdNorm={gyro_std:.6f} accelStdNorm={accel_std:.6f} accNorm={gravity_norm:.6f}'
    )
    if passed:
        print('[STARTUP CHECK] STATIC_IMU=PASS')
        return 0
    print(
        '[STARTUP CHECK] STATIC_IMU=WARN: no startup window satisfies ORB-SLAM3 thresholds; '
        'relocalization may still work, but this bag is high risk.',
        file=sys.stderr,
    )
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
