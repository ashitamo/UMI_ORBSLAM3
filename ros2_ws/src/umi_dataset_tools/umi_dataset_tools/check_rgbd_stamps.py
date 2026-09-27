#!/usr/bin/env python3

import argparse
import collections
import math
import time
from dataclasses import dataclass
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CompressedImage

if __package__:
    from .qos import reliable_image_qos
else:
    from qos import reliable_image_qos


@dataclass
class StreamStats:
    name: str
    topic: str
    count: int = 0
    duplicate_count: int = 0
    backward_count: int = 0
    gap_count: int = 0
    first_stamp_ns: Optional[int] = None
    previous_stamp_ns: Optional[int] = None
    first_arrival_ns: Optional[int] = None
    previous_arrival_ns: Optional[int] = None
    last_report_count: int = 0


class RGBDStampChecker(Node):
    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        expected_hz: float,
        gap_factor: float,
        pair_tolerance_ms: float,
        report_period_sec: float,
        recent_queue_size: int,
    ) -> None:
        super().__init__('rgbd_stamp_checker')

        self.expected_period_ns = int(1e9 / expected_hz)
        self.gap_threshold_ns = int(self.expected_period_ns * gap_factor)
        self.pair_tolerance_ns = int(pair_tolerance_ms * 1e6)

        self.rgb = StreamStats('RGB/Infra1', rgb_topic)
        self.depth = StreamStats('Depth', depth_topic)

        self.recent_rgb: Deque[int] = collections.deque(maxlen=recent_queue_size)
        self.recent_depth: Deque[int] = collections.deque(maxlen=recent_queue_size)

        self.exact_pair_count = 0
        self.near_pair_count = 0
        self.unmatched_rgb_count = 0
        self.unmatched_depth_count = 0

        self.rgb_sub = self.create_subscription(
            CompressedImage if rgb_topic.endswith(('/compressed', '/compressedDepth')) else Image,
            rgb_topic,
            self.rgb_callback,
            reliable_image_qos(),
        )
        self.depth_sub = self.create_subscription(
            CompressedImage if depth_topic.endswith('/compressedDepth') else Image,
            depth_topic,
            self.depth_callback,
            reliable_image_qos(),
        )
        self.report_timer = self.create_timer(
            report_period_sec,
            self.report,
        )

        self.get_logger().info(f'RGB topic: {rgb_topic}')
        self.get_logger().info(f'Depth topic: {depth_topic}')
        self.get_logger().info(
            f'Expected period: {self.expected_period_ns / 1e6:.3f} ms, '
            f'gap threshold: {self.gap_threshold_ns / 1e6:.3f} ms, '
            f'pair tolerance: {pair_tolerance_ms:.3f} ms'
        )

    @staticmethod
    def stamp_to_ns(msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def process_stream(self, stats: StreamStats, stamp_ns: int, arrival_ns: int) -> None:
        stats.count += 1

        if stats.first_stamp_ns is None:
            stats.first_stamp_ns = stamp_ns
            stats.first_arrival_ns = arrival_ns

        if stats.previous_stamp_ns is not None:
            stamp_dt_ns = stamp_ns - stats.previous_stamp_ns
            arrival_dt_ns = arrival_ns - stats.previous_arrival_ns

            if stamp_dt_ns == 0:
                stats.duplicate_count += 1
                self.get_logger().warning(
                    f'[{stats.name} DUPLICATE] count={stats.count}, '
                    f'stamp={stamp_ns / 1e9:.9f}, '
                    f'arrival_dt={arrival_dt_ns / 1e6:.3f} ms'
                )
            elif stamp_dt_ns < 0:
                stats.backward_count += 1
                self.get_logger().error(
                    f'[{stats.name} BACKWARD] count={stats.count}, '
                    f'previous={stats.previous_stamp_ns / 1e9:.9f}, '
                    f'current={stamp_ns / 1e9:.9f}, '
                    f'dt={stamp_dt_ns / 1e6:.3f} ms'
                )
            elif stamp_dt_ns > self.gap_threshold_ns:
                stats.gap_count += 1
                self.get_logger().warning(
                    f'[{stats.name} GAP] count={stats.count}, '
                    f'previous={stats.previous_stamp_ns / 1e9:.9f}, '
                    f'current={stamp_ns / 1e9:.9f}, '
                    f'stamp_dt={stamp_dt_ns / 1e6:.3f} ms, '
                    f'arrival_dt={arrival_dt_ns / 1e6:.3f} ms'
                )

        stats.previous_stamp_ns = stamp_ns
        stats.previous_arrival_ns = arrival_ns

    def rgb_callback(self, msg: Image) -> None:
        stamp_ns = self.stamp_to_ns(msg)
        arrival_ns = time.monotonic_ns()
        self.process_stream(self.rgb, stamp_ns, arrival_ns)
        self.match_rgb_stamp(stamp_ns)

    def depth_callback(self, msg: Image) -> None:
        stamp_ns = self.stamp_to_ns(msg)
        arrival_ns = time.monotonic_ns()
        self.process_stream(self.depth, stamp_ns, arrival_ns)
        self.match_depth_stamp(stamp_ns)

    def match_rgb_stamp(self, rgb_stamp_ns: int) -> None:
        if len(self.recent_rgb) == self.recent_rgb.maxlen:
            self.unmatched_rgb_count += 1
        self.recent_rgb.append(rgb_stamp_ns)
        self.match_pending_stamps()

    def match_depth_stamp(self, depth_stamp_ns: int) -> None:
        if len(self.recent_depth) == self.recent_depth.maxlen:
            self.unmatched_depth_count += 1
        self.recent_depth.append(depth_stamp_ns)
        self.match_pending_stamps()

    def match_pending_stamps(self) -> None:
        while self.recent_rgb and self.recent_depth:
            delta_ns = self.recent_rgb[0] - self.recent_depth[0]
            if abs(delta_ns) <= self.pair_tolerance_ns:
                self.recent_rgb.popleft()
                self.recent_depth.popleft()
                if delta_ns == 0:
                    self.exact_pair_count += 1
                else:
                    self.near_pair_count += 1
            elif delta_ns < 0:
                self.recent_rgb.popleft()
                self.unmatched_rgb_count += 1
            else:
                self.recent_depth.popleft()
                self.unmatched_depth_count += 1

    @staticmethod
    def header_rate(stats: StreamStats) -> float:
        if stats.first_stamp_ns is None or stats.previous_stamp_ns is None or stats.count < 2:
            return 0.0
        duration_sec = (stats.previous_stamp_ns - stats.first_stamp_ns) / 1e9
        if duration_sec <= 0.0:
            return math.inf
        return (stats.count - 1) / duration_sec

    @staticmethod
    def arrival_rate(stats: StreamStats) -> float:
        if stats.first_arrival_ns is None or stats.previous_arrival_ns is None or stats.count < 2:
            return 0.0
        duration_sec = (stats.previous_arrival_ns - stats.first_arrival_ns) / 1e9
        if duration_sec <= 0.0:
            return math.inf
        return (stats.count - 1) / duration_sec

    def report_stream(self, stats: StreamStats) -> str:
        new_messages = stats.count - stats.last_report_count
        stats.last_report_count = stats.count
        return (
            f'{stats.name}: total={stats.count}, new={new_messages}, '
            f'header_rate={self.header_rate(stats):.3f} Hz, '
            f'arrival_rate={self.arrival_rate(stats):.3f} Hz, '
            f'duplicates={stats.duplicate_count}, '
            f'backward={stats.backward_count}, gaps={stats.gap_count}'
        )

    def report(self) -> None:
        emit = self.get_logger().info if rclpy.ok(context=self.context) else print
        emit(self.report_stream(self.rgb))
        emit(self.report_stream(self.depth))
        emit(
            f'Pairing: exact={self.exact_pair_count}, near={self.near_pair_count}, '
            f'unmatched_rgb={self.unmatched_rgb_count}, '
            f'unmatched_depth={self.unmatched_depth_count}, '
            f'pending_rgb={len(self.recent_rgb)}, pending_depth={len(self.recent_depth)}'
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Check RGB/Infra1 and depth timestamps, duplicates, gaps, '
            'arrival rate, and timestamp pairing.'
        )
    )
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
    parser.add_argument('--pair-tolerance-ms', type=float, default=5.0)
    parser.add_argument('--report-period-sec', type=float, default=2.0)
    parser.add_argument('--recent-queue-size', type=int, default=30)
    cli = parser.parse_args()
    for name in ('expected_hz', 'gap_factor', 'report_period_sec'):
        if not math.isfinite(getattr(cli, name)) or getattr(cli, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    if not math.isfinite(cli.pair_tolerance_ms) or cli.pair_tolerance_ms < 0:
        parser.error('pair tolerance must be finite and non-negative')
    if cli.recent_queue_size < 1:
        parser.error('recent queue size must be positive')
    return cli


def main(args=None) -> None:
    cli = parse_args()
    rclpy.init(args=args)
    node = RGBDStampChecker(
        rgb_topic=cli.rgb_topic,
        depth_topic=cli.depth_topic,
        expected_hz=cli.expected_hz,
        gap_factor=cli.gap_factor,
        pair_tolerance_ms=cli.pair_tolerance_ms,
        report_period_sec=cli.report_period_sec,
        recent_queue_size=cli.recent_queue_size,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
