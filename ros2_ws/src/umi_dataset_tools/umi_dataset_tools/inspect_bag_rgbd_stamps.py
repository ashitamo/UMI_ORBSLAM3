#!/usr/bin/env python3

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass
class Stats:
    topic: str
    count: int = 0
    duplicate_count: int = 0
    backward_count: int = 0
    gap_count: int = 0
    first_stamp_ns: Optional[int] = None
    previous_stamp_ns: Optional[int] = None
    previous_bag_time_ns: Optional[int] = None


def stamp_to_ns(msg) -> int:
    return (
        int(msg.header.stamp.sec) * 1_000_000_000
        + int(msg.header.stamp.nanosec)
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Inspect RGB/Infra1 and depth timestamps directly from a rosbag2 '
            'database without ros2 bag play, DDS, or subscribers.'
        )
    )
    parser.add_argument('bag_path')
    parser.add_argument(
        '--rgb-topic',
        default='/camera/camera/infra1/image_rect_raw',
    )
    parser.add_argument(
        '--depth-topic',
        default='/camera/camera/depth/image_rect_raw',
    )
    parser.add_argument('--expected-hz', type=float, default=60.0)
    parser.add_argument('--gap-factor', type=float, default=2.5)
    args = parser.parse_args()

    bag_path = str(Path(args.bag_path).expanduser().resolve())
    expected_period_ns = int(1e9 / args.expected_hz)
    gap_threshold_ns = int(expected_period_ns * args.gap_factor)

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='',
    )
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }

    target_topics = {
        args.rgb_topic,
        args.depth_topic,
    }

    missing = [
        topic
        for topic in target_topics
        if topic not in topic_types
    ]
    if missing:
        raise RuntimeError(
            'Topics not found in bag: ' + ', '.join(missing)
        )

    message_types: Dict[str, type] = {
        topic: get_message(topic_types[topic])
        for topic in target_topics
    }

    stats = {
        args.rgb_topic: Stats(args.rgb_topic),
        args.depth_topic: Stats(args.depth_topic),
    }

    reader.set_filter(
        rosbag2_py.StorageFilter(
            topics=list(target_topics)
        )
    )

    while reader.has_next():
        topic, serialized_data, bag_time_ns = reader.read_next()
        msg = deserialize_message(
            serialized_data,
            message_types[topic],
        )
        stamp_ns = stamp_to_ns(msg)
        item = stats[topic]
        item.count += 1

        if item.first_stamp_ns is None:
            item.first_stamp_ns = stamp_ns

        if item.previous_stamp_ns is not None:
            stamp_dt_ns = stamp_ns - item.previous_stamp_ns
            bag_dt_ns = bag_time_ns - item.previous_bag_time_ns

            if stamp_dt_ns == 0:
                item.duplicate_count += 1
                print(
                    f'[DUPLICATE] topic={topic} '
                    f'count={item.count} '
                    f'stamp={stamp_ns / 1e9:.9f} '
                    f'bag_record_dt={bag_dt_ns / 1e6:.3f} ms'
                )
            elif stamp_dt_ns < 0:
                item.backward_count += 1
                print(
                    f'[BACKWARD] topic={topic} '
                    f'count={item.count} '
                    f'previous={item.previous_stamp_ns / 1e9:.9f} '
                    f'current={stamp_ns / 1e9:.9f} '
                    f'dt={stamp_dt_ns / 1e6:.3f} ms'
                )
            elif stamp_dt_ns > gap_threshold_ns:
                item.gap_count += 1
                print(
                    f'[GAP] topic={topic} '
                    f'count={item.count} '
                    f'previous={item.previous_stamp_ns / 1e9:.9f} '
                    f'current={stamp_ns / 1e9:.9f} '
                    f'stamp_dt={stamp_dt_ns / 1e6:.3f} ms '
                    f'bag_record_dt={bag_dt_ns / 1e6:.3f} ms'
                )

        item.previous_stamp_ns = stamp_ns
        item.previous_bag_time_ns = bag_time_ns

    print('\n=== SUMMARY ===')
    for topic, item in stats.items():
        if (
            item.first_stamp_ns is not None
            and item.previous_stamp_ns is not None
            and item.previous_stamp_ns > item.first_stamp_ns
            and item.count > 1
        ):
            duration_sec = (
                item.previous_stamp_ns - item.first_stamp_ns
            ) / 1e9
            header_rate = (item.count - 1) / duration_sec
        else:
            header_rate = 0.0

        print(
            f'topic={topic}\n'
            f'  total={item.count}\n'
            f'  header_rate={header_rate:.3f} Hz\n'
            f'  duplicates={item.duplicate_count}\n'
            f'  backward={item.backward_count}\n'
            f'  gaps={item.gap_count}'
        )


if __name__ == '__main__':
    main()
