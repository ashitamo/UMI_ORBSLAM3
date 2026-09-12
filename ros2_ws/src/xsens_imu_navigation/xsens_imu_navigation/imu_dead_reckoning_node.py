#!/usr/bin/env python3
"""Short-horizon inertial dead reckoning for Xsens Free Acceleration.

This node is intended as a diagnostic tool. It integrates Xsens
/filter/free_acceleration (already gravity-compensated and expressed in the
Xsens local Earth frame) into velocity and position. Drift is reduced with:

* initial stationary bias estimation
* low-pass filtering and a small acceleration deadband
* gyroscope-aided stationary detection
* zero-velocity updates (ZUPT)
* slow bias adaptation while stationary
* timestamp sanity checks

It does not make absolute position observable. Without external position or
velocity measurements, long-term drift is unavoidable.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def vec3_to_np(vector) -> np.ndarray:
    return np.array([vector.x, vector.y, vector.z], dtype=np.float64)


def fill_vector3(target, value: np.ndarray) -> None:
    target.x = float(value[0])
    target.y = float(value[1])
    target.z = float(value[2])


class ImuDeadReckoningNode(Node):
    """Integrate Xsens free acceleration with ZUPT and bias adaptation."""

    def __init__(self) -> None:
        super().__init__('imu_dead_reckoning')

        # Topics and frames
        self.declare_parameter('free_accel_topic', '/filter/free_acceleration')
        self.declare_parameter('quaternion_topic', '/filter/quaternion')
        self.declare_parameter('gyro_topic', '/imu/angular_velocity')
        self.declare_parameter('world_frame', 'imu_nav')
        self.declare_parameter('body_frame', 'imu_link')
        self.declare_parameter('child_frame', 'imu_dead_reckoning')

        # Timing
        self.declare_parameter('nominal_rate_hz', 100.0)
        self.declare_parameter('use_fixed_dt', False)
        self.declare_parameter('min_dt', 0.002)
        self.declare_parameter('max_dt', 0.05)
        self.declare_parameter('max_aux_age', 0.05)

        # Initial calibration
        self.declare_parameter('initial_calibration_seconds', 5.0)
        self.declare_parameter('initial_accel_limit', 0.50)
        self.declare_parameter('initial_gyro_limit', 0.05)

        # Filtering and stationary detection
        self.declare_parameter('low_pass_cutoff_hz', 8.0)
        self.declare_parameter('accel_deadband', 0.025)
        self.declare_parameter('stationary_accel_threshold', 0.08)
        self.declare_parameter('stationary_gyro_threshold', 0.025)
        self.declare_parameter('stationary_hold_seconds', 0.25)
        self.declare_parameter('bias_adaptation_tau', 20.0)

        # Integration options
        self.declare_parameter('planar_mode', False)
        self.declare_parameter('velocity_leak_per_second', 0.0)
        self.declare_parameter('max_acceleration', 40.0)
        self.declare_parameter('path_publish_rate_hz', 20.0)
        self.declare_parameter('path_max_points', 10000)
        self.declare_parameter('publish_tf', True)

        self.free_accel_topic = str(self.get_parameter('free_accel_topic').value)
        self.quaternion_topic = str(self.get_parameter('quaternion_topic').value)
        self.gyro_topic = str(self.get_parameter('gyro_topic').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.body_frame = str(self.get_parameter('body_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)

        self.nominal_rate_hz = float(self.get_parameter('nominal_rate_hz').value)
        self.use_fixed_dt = bool(self.get_parameter('use_fixed_dt').value)
        self.min_dt = float(self.get_parameter('min_dt').value)
        self.max_dt = float(self.get_parameter('max_dt').value)
        self.max_aux_age = float(self.get_parameter('max_aux_age').value)

        self.initial_calibration_seconds = float(
            self.get_parameter('initial_calibration_seconds').value
        )
        self.initial_accel_limit = float(self.get_parameter('initial_accel_limit').value)
        self.initial_gyro_limit = float(self.get_parameter('initial_gyro_limit').value)

        self.low_pass_cutoff_hz = float(self.get_parameter('low_pass_cutoff_hz').value)
        self.accel_deadband = float(self.get_parameter('accel_deadband').value)
        self.stationary_accel_threshold = float(
            self.get_parameter('stationary_accel_threshold').value
        )
        self.stationary_gyro_threshold = float(
            self.get_parameter('stationary_gyro_threshold').value
        )
        self.stationary_hold_seconds = float(
            self.get_parameter('stationary_hold_seconds').value
        )
        self.bias_adaptation_tau = float(self.get_parameter('bias_adaptation_tau').value)

        self.planar_mode = bool(self.get_parameter('planar_mode').value)
        self.velocity_leak_per_second = float(
            self.get_parameter('velocity_leak_per_second').value
        )
        self.max_acceleration = float(self.get_parameter('max_acceleration').value)
        self.path_publish_rate_hz = float(
            self.get_parameter('path_publish_rate_hz').value
        )
        self.path_max_points = int(self.get_parameter('path_max_points').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        if self.nominal_rate_hz <= 0.0:
            raise ValueError('nominal_rate_hz must be positive')
        if self.low_pass_cutoff_hz <= 0.0:
            raise ValueError('low_pass_cutoff_hz must be positive')
        if self.path_publish_rate_hz <= 0.0:
            raise ValueError('path_publish_rate_hz must be positive')

        # Best-effort sensor-data QoS matches common IMU publishers.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.create_subscription(
            Vector3Stamped,
            self.free_accel_topic,
            self.free_accel_callback,
            sensor_qos,
        )
        self.create_subscription(
            QuaternionStamped,
            self.quaternion_topic,
            self.quaternion_callback,
            sensor_qos,
        )
        self.create_subscription(
            Vector3Stamped,
            self.gyro_topic,
            self.gyro_callback,
            sensor_qos,
        )

        self.odom_pub = self.create_publisher(Odometry, '/imu_nav/odometry', 20)
        self.path_pub = self.create_publisher(Path, '/imu_nav/path', 10)
        self.stationary_pub = self.create_publisher(Bool, '/imu_nav/stationary', 10)
        self.filtered_accel_pub = self.create_publisher(
            Vector3Stamped, '/imu_nav/filtered_acceleration', 20
        )
        self.bias_pub = self.create_publisher(Vector3Stamped, '/imu_nav/accel_bias', 10)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_service(Trigger, '/imu_nav/reset', self.reset_service)
        self.create_service(Trigger, '/imu_nav/recalibrate', self.recalibrate_service)

        self.position = np.zeros(3, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)
        self.accel_bias = np.zeros(3, dtype=np.float64)
        self.filtered_accel = np.zeros(3, dtype=np.float64)
        self.previous_accel = np.zeros(3, dtype=np.float64)

        self.latest_gyro = np.zeros(3, dtype=np.float64)
        self.latest_gyro_time: Optional[float] = None
        self.latest_quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self.latest_quaternion_time: Optional[float] = None

        self.last_accel_time: Optional[float] = None
        self.last_path_time: Optional[float] = None
        self.stationary_elapsed = 0.0
        self.stationary = False
        self.was_stationary = False
        self.time_since_zupt = 0.0

        self.calibration_samples: list[np.ndarray] = []
        self.calibrated = False
        self.required_calibration_samples = max(
            1, int(round(self.initial_calibration_seconds * self.nominal_rate_hz))
        )
        self.last_calibration_report = -1

        self.path_poses: Deque[PoseStamped] = deque(maxlen=self.path_max_points)
        self.warned_input_frame = False

        self.get_logger().info(
            'IMU dead reckoning started. Keep the sensor completely still for '
            f'{self.initial_calibration_seconds:.1f} s. '
            'This is short-horizon diagnostic odometry; long-term drift is unavoidable.'
        )

    def quaternion_callback(self, msg: QuaternionStamped) -> None:
        q = np.array(
            [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(q))
        if norm > 1.0e-9 and np.all(np.isfinite(q)):
            self.latest_quaternion = q / norm
            self.latest_quaternion_time = stamp_to_sec(msg.header.stamp)

    def gyro_callback(self, msg: Vector3Stamped) -> None:
        gyro = vec3_to_np(msg.vector)
        if np.all(np.isfinite(gyro)):
            self.latest_gyro = gyro
            self.latest_gyro_time = stamp_to_sec(msg.header.stamp)

    def get_gyro_for_time(self, timestamp: float) -> np.ndarray:
        if self.latest_gyro_time is None:
            return np.full(3, np.inf, dtype=np.float64)
        if abs(timestamp - self.latest_gyro_time) > self.max_aux_age:
            return np.full(3, np.inf, dtype=np.float64)
        return self.latest_gyro

    def get_quaternion_for_time(self, timestamp: float) -> np.ndarray:
        if self.latest_quaternion_time is None:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        if abs(timestamp - self.latest_quaternion_time) > self.max_aux_age:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        return self.latest_quaternion

    def free_accel_callback(self, msg: Vector3Stamped) -> None:
        raw_accel = vec3_to_np(msg.vector)
        if not np.all(np.isfinite(raw_accel)):
            self.get_logger().warning('Discarded non-finite free acceleration sample.')
            return
        if float(np.linalg.norm(raw_accel)) > self.max_acceleration:
            self.get_logger().warning(
                f'Discarded acceleration outlier: {np.linalg.norm(raw_accel):.3f} m/s^2'
            )
            return

        timestamp = stamp_to_sec(msg.header.stamp)
        if timestamp <= 0.0:
            timestamp = self.get_clock().now().nanoseconds * 1.0e-9

        if not self.warned_input_frame and msg.header.frame_id:
            self.get_logger().info(
                f'Input Free Acceleration frame_id is "{msg.header.frame_id}". '
                f'This node treats the vector as Xsens local Earth frame and publishes it in '
                f'"{self.world_frame}".'
            )
            self.warned_input_frame = True

        gyro = self.get_gyro_for_time(timestamp)

        if not self.calibrated:
            self.collect_initial_calibration(raw_accel, gyro, msg.header.stamp)
            self.last_accel_time = timestamp
            return

        if self.last_accel_time is None:
            self.last_accel_time = timestamp
            return

        measured_dt = timestamp - self.last_accel_time
        self.last_accel_time = timestamp

        if self.use_fixed_dt:
            dt = 1.0 / self.nominal_rate_hz
        else:
            dt = measured_dt
            if dt < self.min_dt or dt > self.max_dt:
                self.get_logger().warning(
                    f'Invalid IMU dt={dt:.6f} s; integration paused for this sample.'
                )
                self.previous_accel[:] = 0.0
                self.filtered_accel[:] = 0.0
                return

        unbiased_accel = raw_accel - self.accel_bias

        # First-order low-pass filter.
        rc = 1.0 / (2.0 * math.pi * self.low_pass_cutoff_hz)
        alpha = dt / (rc + dt)
        self.filtered_accel += alpha * (unbiased_accel - self.filtered_accel)

        # Suppress tiny residual components after gravity removal.
        accel_for_integration = self.filtered_accel.copy()
        accel_for_integration[
            np.abs(accel_for_integration) < self.accel_deadband
        ] = 0.0

        if self.planar_mode:
            accel_for_integration[2] = 0.0

        accel_norm = float(np.linalg.norm(unbiased_accel))
        gyro_norm = float(np.linalg.norm(gyro))
        stationary_candidate = (
            accel_norm < self.stationary_accel_threshold
            and gyro_norm < self.stationary_gyro_threshold
        )

        if stationary_candidate:
            self.stationary_elapsed += dt
        else:
            self.stationary_elapsed = 0.0

        self.stationary = self.stationary_elapsed >= self.stationary_hold_seconds

        if self.stationary:
            # ZUPT: velocity is exactly zero during confirmed stationary periods.
            self.velocity[:] = 0.0
            self.previous_accel[:] = 0.0
            self.time_since_zupt = 0.0

            # Slowly learn residual free-acceleration bias while stationary.
            if self.bias_adaptation_tau > 0.0:
                beta = min(1.0, dt / self.bias_adaptation_tau)
                self.accel_bias += beta * (raw_accel - self.accel_bias)
        else:
            self.time_since_zupt += dt
            if self.was_stationary:
                # Avoid blending the last stationary zero with the first motion sample.
                self.previous_accel = accel_for_integration.copy()

            average_accel = 0.5 * (self.previous_accel + accel_for_integration)
            self.position += self.velocity * dt + 0.5 * average_accel * dt * dt
            self.velocity += average_accel * dt

            if self.velocity_leak_per_second > 0.0:
                # Optional non-physical damping for visualization only; default is disabled.
                self.velocity *= math.exp(-self.velocity_leak_per_second * dt)

            if self.planar_mode:
                self.position[2] = 0.0
                self.velocity[2] = 0.0

            self.previous_accel = accel_for_integration.copy()

        self.was_stationary = self.stationary
        self.publish_outputs(msg, timestamp, accel_for_integration)

    def collect_initial_calibration(
        self, raw_accel: np.ndarray, gyro: np.ndarray, stamp
    ) -> None:
        accel_ok = float(np.linalg.norm(raw_accel)) < self.initial_accel_limit
        gyro_ok = float(np.linalg.norm(gyro)) < self.initial_gyro_limit

        if accel_ok and gyro_ok:
            self.calibration_samples.append(raw_accel.copy())
        else:
            if self.calibration_samples:
                self.get_logger().warning(
                    'Motion detected during initial calibration; restarting the stationary window.'
                )
                self.calibration_samples.clear()
                self.last_calibration_report = -1
            return

        percent = int(100 * len(self.calibration_samples) / self.required_calibration_samples)
        report_bucket = percent // 10
        if report_bucket != self.last_calibration_report:
            self.last_calibration_report = report_bucket
            self.get_logger().info(f'Initial bias calibration: {min(percent, 100)}%')

        if len(self.calibration_samples) >= self.required_calibration_samples:
            samples = np.vstack(self.calibration_samples)
            self.accel_bias = samples.mean(axis=0)
            sample_std = samples.std(axis=0)
            self.calibrated = True
            self.filtered_accel[:] = 0.0
            self.previous_accel[:] = 0.0
            self.position[:] = 0.0
            self.velocity[:] = 0.0
            self.stationary_elapsed = self.stationary_hold_seconds
            self.stationary = True
            self.was_stationary = True
            self.time_since_zupt = 0.0
            self.path_poses.clear()
            self.get_logger().info(
                'Initial calibration complete. '
                f'bias={self.accel_bias.tolist()} m/s^2, '
                f'std={sample_std.tolist()} m/s^2'
            )
            self.publish_bias(stamp)

    def publish_outputs(
        self, input_msg: Vector3Stamped, timestamp: float, filtered_accel: np.ndarray
    ) -> None:
        stamp = input_msg.header.stamp
        q = self.get_quaternion_for_time(timestamp)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = float(self.position[0])
        odom.pose.pose.position.y = float(self.position[1])
        odom.pose.pose.position.z = float(self.position[2])
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        fill_vector3(odom.twist.twist.linear, self.velocity)

        # Heuristic covariance growth since the last ZUPT. This is diagnostic,
        # not a calibrated probabilistic model.
        pos_sigma = 0.02 + 0.20 * self.time_since_zupt * self.time_since_zupt
        vel_sigma = 0.02 + 0.10 * self.time_since_zupt
        for index in (0, 7, 14):
            odom.pose.covariance[index] = pos_sigma * pos_sigma
            odom.twist.covariance[index] = vel_sigma * vel_sigma
        for index in (21, 28, 35):
            odom.pose.covariance[index] = 0.05 * 0.05
            odom.twist.covariance[index] = 0.10 * 0.10

        self.odom_pub.publish(odom)

        stationary_msg = Bool()
        stationary_msg.data = self.stationary
        self.stationary_pub.publish(stationary_msg)

        filtered_msg = Vector3Stamped()
        filtered_msg.header.stamp = stamp
        filtered_msg.header.frame_id = self.world_frame
        fill_vector3(filtered_msg.vector, filtered_accel)
        self.filtered_accel_pub.publish(filtered_msg)
        self.publish_bias(stamp)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.world_frame
            transform.child_frame_id = self.child_frame
            fill_vector3(transform.transform.translation, self.position)
            transform.transform.rotation.x = float(q[0])
            transform.transform.rotation.y = float(q[1])
            transform.transform.rotation.z = float(q[2])
            transform.transform.rotation.w = float(q[3])
            self.tf_broadcaster.sendTransform(transform)

        path_period = 1.0 / self.path_publish_rate_hz
        if self.last_path_time is None or timestamp - self.last_path_time >= path_period:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.world_frame
            pose.pose = odom.pose.pose
            self.path_poses.append(pose)

            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = self.world_frame
            path.poses = list(self.path_poses)
            self.path_pub.publish(path)
            self.last_path_time = timestamp

    def publish_bias(self, stamp) -> None:
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        fill_vector3(msg.vector, self.accel_bias)
        self.bias_pub.publish(msg)

    def reset_state(self, keep_bias: bool) -> None:
        self.position[:] = 0.0
        self.velocity[:] = 0.0
        self.filtered_accel[:] = 0.0
        self.previous_accel[:] = 0.0
        self.last_accel_time = None
        self.last_path_time = None
        self.stationary_elapsed = 0.0
        self.stationary = False
        self.was_stationary = False
        self.time_since_zupt = 0.0
        self.path_poses.clear()

        if not keep_bias:
            self.accel_bias[:] = 0.0
            self.calibration_samples.clear()
            self.calibrated = False
            self.last_calibration_report = -1

    def reset_service(self, _request, response):
        self.reset_state(keep_bias=True)
        response.success = True
        response.message = 'Position and velocity reset; acceleration bias retained.'
        return response

    def recalibrate_service(self, _request, response):
        self.reset_state(keep_bias=False)
        response.success = True
        response.message = (
            'State reset and calibration restarted. Keep the IMU completely still.'
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuDeadReckoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
