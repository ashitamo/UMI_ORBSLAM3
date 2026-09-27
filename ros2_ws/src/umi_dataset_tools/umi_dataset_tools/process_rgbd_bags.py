#!/usr/bin/env python3
"""Export trajectories, RGB images, colored point clouds and gripper width."""

import argparse
from bisect import bisect_left
import csv
from pathlib import Path

import cv2
import cv2.aruco as aruco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import yaml
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation

from umi_dataset_tools.project_paths import data_root
from umi_dataset_tools.compressed_depth import decode_compressed_depth


RGB_DEFAULT = '/camera/camera/color/image_raw/compressed'
INFRA_DEFAULT = '/camera/camera/infra1/image_rect_raw'
DEPTH_DEFAULT = '/camera/camera/depth/image_rect_raw'
DEPTH_COMPRESSED_DEFAULT = '/camera/camera/depth/image_rect_raw/compressedDepth'
COLOR_INFO_DEFAULT = '/camera/camera/color/camera_info'
DEPTH_INFO_DEFAULT = '/camera/camera/depth/camera_info'
DEPTH_TO_TRACKING_DEFAULT = '/camera/camera/extrinsics/depth_to_infra1'
DEPTH_TO_COLOR_DEFAULT = '/camera/camera/extrinsics/depth_to_color'
TRACKING_FRAME = 'camera_infra1_optical_frame'
TRAJECTORY_MATCH_TOLERANCE_NS = 1_000_000


# Naming rule:
#   T_A_B = pose of target frame B expressed in origin/reference frame A
#   R_A_B = orientation of target frame B expressed in frame A
# Example: T_BASE_EE is the EE pose expressed in BASE.
# trajectory.csv contains persistent ORB-SLAM map poses T_WORLD_TRACKING.


def stamp_ns(message, bag_time):
    header = getattr(message, 'header', None)
    if header is None:
        return int(bag_time)
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def read_trajectory(path):
    result = []
    with Path(path).expanduser().open() as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 8 or fields[0].startswith('#'):
                continue
            values = [float(value) for value in fields[:8]]
            result.append((int(round(values[0] * 1e9)), np.array(values[1:4]), np.array(values[4:8])))
    if not result:
        raise RuntimeError(f'No trajectory records found: {path}')
    result.sort(key=lambda record: record[0])
    return result


def quaternion_matrix(quaternion):
    return Rotation.from_quat(np.asarray(quaternion, dtype=float)).as_matrix()


def matrix_quaternion(rotation):
    return Rotation.from_matrix(np.asarray(rotation, dtype=float)).as_quat()


def clip_trajectory_by_radius(records, head_radius, tail_radius):
    positions = np.asarray([record[1] for record in records], dtype=float)
    distances = np.linalg.norm(positions - positions[0], axis=1)
    start_index = next(
        (
            index
            for index, distance in enumerate(distances)
            if distance > head_radius
        ),
        None,
    )
    if start_index is None:
        return records, {
            'triggered': False,
            'reason': 'trajectory never exceeded head radius',
            'head_radius_m': float(head_radius),
            'tail_radius_m': float(tail_radius),
            'start_index': 0,
            'end_index': len(records) - 1,
            'start_timestamp_ns': records[0][0],
            'end_timestamp_ns': records[-1][0],
            'max_distance_from_first_m': float(np.max(distances)),
        }

    last_tail_outside_index = next(
        (
            index
            for index in range(len(records) - 1, -1, -1)
            if distances[index] > tail_radius
        ),
        None,
    )
    if last_tail_outside_index is None:
        end_index = len(records) - 1
        reason = 'head crossing found; trajectory never exceeded tail radius'
    else:
        end_index = max(
            start_index,
            min(last_tail_outside_index + 1, len(records) - 1),
        )
        reason = 'head and tail radius crossings found'
    return records[start_index:end_index + 1], {
        'triggered': True,
        'reason': reason,
        'head_radius_m': float(head_radius),
        'tail_radius_m': float(tail_radius),
        'start_index': start_index,
        'end_index': end_index,
        'start_timestamp_ns': records[start_index][0],
        'end_timestamp_ns': records[end_index][0],
        'start_distance_from_first_m': float(distances[start_index]),
        'end_distance_from_first_m': float(distances[end_index]),
        'max_distance_from_first_m': float(np.max(distances)),
    }


def nearest_trajectory_record(records, timestamps, timestamp):
    insertion = bisect_left(timestamps, timestamp)
    candidates = []
    if insertion < len(records):
        candidates.append(records[insertion])
    if insertion > 0:
        candidates.append(records[insertion - 1])
    record = min(candidates, key=lambda item: abs(item[0] - timestamp))
    if abs(record[0] - timestamp) > TRAJECTORY_MATCH_TOLERANCE_NS:
        return None
    return record


def build_frame_availability(stream_timestamps, trajectory, trajectory_timestamps):
    camera_timestamps = sorted(set().union(*stream_timestamps.values()))
    rows = []
    trajectory_matches = {}
    for timestamp in camera_timestamps:
        record = nearest_trajectory_record(
            trajectory,
            trajectory_timestamps,
            timestamp,
        )
        has_infra1 = timestamp in stream_timestamps['infra1']
        has_depth = timestamp in stream_timestamps['depth']
        has_color = timestamp in stream_timestamps['color']
        has_trajectory = record is not None
        is_complete = has_infra1 and has_depth and has_color and has_trajectory
        missing = []
        for name, present in (
            ('infra1', has_infra1),
            ('depth', has_depth),
            ('color', has_color),
            ('trajectory', has_trajectory),
        ):
            if not present:
                missing.append(name)
        if record is not None:
            trajectory_matches[timestamp] = record
        rows.append({
            'timestamp_ns': timestamp,
            'timestamp_sec': timestamp / 1e9,
            'has_infra1': has_infra1,
            'has_depth': has_depth,
            'has_color': has_color,
            'has_trajectory': has_trajectory,
            'is_complete': is_complete,
            'trajectory_timestamp_ns': record[0] if record is not None else '',
            'trajectory_minus_frame_sec': (
                (record[0] - timestamp) / 1e9 if record is not None else ''
            ),
            'missing_streams': ';'.join(missing),
        })
    return rows, trajectory_matches


def write_missing_timestamps_csv(path, rows):
    fields = [
        'timestamp_ns',
        'timestamp_sec',
        'has_infra1',
        'has_depth',
        'has_color',
        'has_trajectory',
        'trajectory_timestamp_ns',
        'trajectory_minus_frame_sec',
        'missing_streams',
    ]
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            if row['is_complete']:
                continue
            output = dict(row)
            output.pop('is_complete')
            output['timestamp_sec'] = f'{row["timestamp_sec"]:.9f}'
            if row['trajectory_minus_frame_sec'] != '':
                output['trajectory_minus_frame_sec'] = (
                    f'{row["trajectory_minus_frame_sec"]:.9f}'
                )
            for field in (
                'has_infra1',
                'has_depth',
                'has_color',
                'has_trajectory',
            ):
                output[field] = int(row[field])
            writer.writerow(output)


def write_missing_timestamps_image(path, rows):
    labels = [
        ('Infra1', 'has_infra1'),
        ('Depth', 'has_depth'),
        ('Color', 'has_color'),
        ('Trajectory', 'has_trajectory'),
        ('Complete', 'is_complete'),
    ]
    figure, axis = plt.subplots(figsize=(14, 4.5), constrained_layout=True)
    if rows:
        start_sec = rows[0]['timestamp_sec']
        duration = max(rows[-1]['timestamp_sec'] - start_sec, 1e-9)
        for row_index, (_, field) in enumerate(labels):
            axis.hlines(row_index, 0.0, duration, color='#8fd694', linewidth=5)
            missing_times = [
                row['timestamp_sec'] - start_sec
                for row in rows
                if not row[field]
            ]
            axis.vlines(
                missing_times,
                row_index - 0.32,
                row_index + 0.32,
                color='#d62728',
                linewidth=1.5,
            )
        axis.set_xlim(0.0, duration)
    axis.set_yticks(range(len(labels)), [label for label, _ in labels])
    axis.invert_yaxis()
    axis.set_xlabel('Elapsed time (s)')
    axis.set_title('Missing timestamps (red=missing, green=available)')
    axis.grid(axis='x', color='#d9d9d9', linewidth=0.7)
    figure.savefig(path, dpi=150)
    plt.close(figure)


def make_transform(rotation, translation):
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def pose_transform(record):
    transform = np.eye(4)
    transform[:3, :3] = quaternion_matrix(record[2])
    transform[:3, 3] = record[1]
    return transform


def checked_transform(value, name):
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise RuntimeError(f'{name} must be a finite 4x4 transform')
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise RuntimeError(f'{name} has an invalid homogeneous last row')
    return transform


def end_effector_conversion(calibration):
    if not calibration or 'end_effector_calibration' not in calibration:
        return None
    try:
        end_effector = calibration['end_effector_calibration']
        cad_reference = end_effector['cad_reference']
        measurement = end_effector['measurement']
        camera_frame = measurement['camera_frame_id']
        if 'T_base_from_yam_ee' in cad_reference and 'yam_ee_pose' in measurement:
            ee_frame = 'yam_ee'
            includes_umi_to_yam_rotation = True
            T_BASE_EE_0 = checked_transform(
                cad_reference['T_base_from_yam_ee'],
                'T_BASE_YAM_EE_0',
            )
            T_CAMERA_EE = checked_transform(
                measurement['yam_ee_pose']['T_camera_from_yam_ee'],
                'T_CAMERA_YAM_EE',
            )
        else:
            ee_frame = 'legacy_umi_ee'
            includes_umi_to_yam_rotation = False
            T_BASE_EE_0 = checked_transform(
                cad_reference['T_base_from_umi_ee'],
                'T_BASE_UMI_EE_0',
            )
            T_CAMERA_EE = checked_transform(
                measurement['umi_ee_pose']['T_camera_from_umi_ee'],
                'T_CAMERA_UMI_EE',
            )
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            'End-effector conversion requires matching YAM EE or legacy UMI EE '
            'transforms in cad_reference and measurement'
        ) from error
    return {
        'T_BASE_EE_0': T_BASE_EE_0,
        'camera_frame': camera_frame,
        'T_CAMERA_EE': T_CAMERA_EE,
        'ee_frame': ee_frame,
        'includes_umi_to_yam_rotation': includes_umi_to_yam_rotation,
    }


def tracking_from_end_effector(conversion, T_TRACKING_DEPTH, T_COLOR_DEPTH):
    camera_frame = conversion['camera_frame']
    T_CAMERA_EE = conversion['T_CAMERA_EE']
    if camera_frame == TRACKING_FRAME:
        return T_CAMERA_EE, np.eye(4)
    if camera_frame != 'camera_color_optical_frame':
        raise RuntimeError(
            f'Unsupported hand-eye camera frame {camera_frame!r}; expected '
            f'{TRACKING_FRAME!r} or camera_color_optical_frame'
        )
    if T_TRACKING_DEPTH is None or T_COLOR_DEPTH is None:
        raise RuntimeError(
            'Converting color-camera hand-eye calibration to the ORB tracking frame '
            'requires depth_to_infra1 and depth_to_color extrinsics from the bag'
        )
    T_TRACKING_COLOR = T_TRACKING_DEPTH @ np.linalg.inv(T_COLOR_DEPTH)
    return T_TRACKING_COLOR @ T_CAMERA_EE, T_TRACKING_COLOR


def convert_world_tracking_to_base_ee(records, T_BASE_EE_0, T_TRACKING_EE):
    T_WORLD_TRACKING_0 = pose_transform(records[0])
    T_TRACKING_0_WORLD = np.linalg.inv(T_WORLD_TRACKING_0)
    T_EE_TRACKING = np.linalg.inv(T_TRACKING_EE)
    T_BASE_TRACKING_0 = T_BASE_EE_0 @ T_EE_TRACKING
    converted = []
    for record in records:
        timestamp = record[0]
        T_WORLD_TRACKING_t = pose_transform(record)
        T_TRACKING_0_TRACKING_t = T_TRACKING_0_WORLD @ T_WORLD_TRACKING_t
        T_BASE_EE_t = T_BASE_TRACKING_0 @ T_TRACKING_0_TRACKING_t @ T_TRACKING_EE
        converted.append((
            timestamp,
            T_BASE_EE_t[:3, 3].copy(),
            matrix_quaternion(T_BASE_EE_t[:3, :3]),
        ))
    first_error = float(np.max(np.abs(pose_transform(converted[0]) - T_BASE_EE_0)))
    return converted, {
        'T_WORLD_TRACKING_0': T_WORLD_TRACKING_0,
        'T_TRACKING_0_WORLD': T_TRACKING_0_WORLD,
        'T_EE_TRACKING': T_EE_TRACKING,
        'T_BASE_TRACKING_0': T_BASE_TRACKING_0,
        'first_pose_max_abs_error': first_error,
    }


def bag_label(bag_path):
    if bag_path.is_dir():
        return bag_path.name
    label = bag_path.stem
    return label[:-2] if label.endswith('_0') else label


def resolve_trajectory_path(trajectory_arg, bag_path):
    path = Path(trajectory_arg).expanduser()
    if path.is_file():
        return path
    label = bag_label(bag_path)
    candidates = [
        path / label / 'CameraTrajectory.txt',
        path / f'{label}_CameraTrajectory.txt',
        path / f'{label}.txt',
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    tried = '\n'.join(str(candidate) for candidate in candidates)
    raise RuntimeError(f'No trajectory file found for {label}. Tried:\n{tried}')


def decode_image(message, bridge, compressed):
    if compressed:
        array = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError('Unable to decode compressed color image')
        return image
    return bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')


def depth_image(message, bridge):
    if hasattr(message, 'format'):
        image = decode_compressed_depth(message)
    else:
        image = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
    image = np.asarray(image)
    if image.dtype == np.uint16:
        return image.astype(np.float32) * 0.001
    return image.astype(np.float32)


def camera_matrix(message):
    return np.asarray(message.k, dtype=float).reshape(3, 3)


def camera_info_record(message, timestamp):
    return {
        'timestamp_ns': int(timestamp),
        'frame_id': message.header.frame_id,
        'width': int(message.width),
        'height': int(message.height),
        'distortion_model': message.distortion_model,
        'distortion_coefficients': [float(value) for value in message.d],
        'camera_matrix': np.asarray(message.k, dtype=float).reshape(3, 3).tolist(),
        'rectification_matrix': np.asarray(message.r, dtype=float).reshape(3, 3).tolist(),
        'projection_matrix': np.asarray(message.p, dtype=float).reshape(3, 4).tolist(),
    }


def extrinsics_record(message, timestamp, source_frame, target_frame):
    transform = make_transform(message.rotation, message.translation)
    return {
        'timestamp_ns': int(timestamp),
        'source_frame': source_frame,
        'target_frame': target_frame,
        'convention': f'p_{target_frame} = rotation * p_{source_frame} + translation_m',
        'rotation': transform[:3, :3].tolist(),
        'translation_m': transform[:3, 3].tolist(),
        'transform_4x4': transform.tolist(),
    }


def detect_aruco_markers(image):
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(aruco, 'ArucoDetector'):
        detector = aruco.ArucoDetector(dictionary)
        return detector.detectMarkers(gray)
    parameters = aruco.DetectorParameters_create()
    return aruco.detectMarkers(gray, dictionary, parameters=parameters)


def marker_measurement(image, matrix, distortion, calibration):
    if matrix is None or calibration is None:
        return None
    corners, ids, _ = detect_aruco_markers(image)
    if ids is None:
        return None
    marker_length = float(calibration.get('marker_length_m', 0.0166))
    marker_ids = [int(marker_id) for marker_id in calibration.get('marker_ids', [0, 5])]
    if len(marker_ids) != 2:
        raise RuntimeError('distance_calibration.marker_ids must contain exactly two marker ids')
    object_points = np.array([
        [-marker_length / 2, marker_length / 2, 0],
        [marker_length / 2, marker_length / 2, 0],
        [marker_length / 2, -marker_length / 2, 0],
        [-marker_length / 2, -marker_length / 2, 0],
    ], dtype=float)
    positions = {}
    for corner, marker_id in zip(corners, ids.reshape(-1)):
        if int(marker_id) not in marker_ids:
            continue
        ok, _, translation = cv2.solvePnP(object_points, corner[0].astype(float), matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            positions[int(marker_id)] = translation.reshape(3)
    if marker_ids[0] not in positions or marker_ids[1] not in positions:
        return None
    distance = float(np.linalg.norm(positions[marker_ids[0]] - positions[marker_ids[1]]))
    zero = (calibration.get('zero_percent') or {}).get('center_distance_m')
    full = (calibration.get('hundred_percent') or {}).get('center_distance_m')
    percentage = None
    if zero is not None and full is not None and abs(float(full) - float(zero)) > 1e-12:
        percentage = (distance - float(zero)) / (float(full) - float(zero)) * 100.0
    return distance, percentage


def write_trajectory(path, records):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['timestamp_ns', 'timestamp_sec', 'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw'])
        for timestamp, position, quaternion in records:
            writer.writerow([timestamp, f'{timestamp / 1e9:.9f}', *position, *quaternion])


def write_trajectory_tum(path, records):
    with path.open('w') as stream:
        for timestamp, position, quaternion in records:
            stream.write(
                f'{timestamp / 1e9:.9f} '
                f'{position[0]:.9f} {position[1]:.9f} {position[2]:.9f} '
                f'{quaternion[0]:.9f} {quaternion[1]:.9f} {quaternion[2]:.9f} {quaternion[3]:.9f}\n'
            )


def write_ply(path, points, colors, comments=None):
    with path.open('w') as stream:
        stream.write('ply\nformat ascii 1.0\n')
        for comment in comments or []:
            stream.write(f'comment {comment}\n')
        stream.write(f'element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\n')
        stream.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for point, color in zip(points, colors):
            stream.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {color[0]} {color[1]} {color[2]}\n')


def point_colors_from_color_camera(color_points, color_image, color_matrix):
    z = color_points[:, 2]
    valid_z = z > 1e-9
    u = np.full(len(color_points), -1, dtype=np.int32)
    v = np.full(len(color_points), -1, dtype=np.int32)
    u[valid_z] = np.rint((color_points[valid_z, 0] * color_matrix[0, 0] / z[valid_z]) + color_matrix[0, 2]).astype(np.int32)
    v[valid_z] = np.rint((color_points[valid_z, 1] * color_matrix[1, 1] / z[valid_z]) + color_matrix[1, 2]).astype(np.int32)
    height, width = color_image.shape[:2]
    inside = valid_z & (u >= 0) & (u < width) & (v >= 0) & (v < height)
    frame_colors = np.full((len(color_points), 3), 180, dtype=np.uint8)
    frame_colors[inside] = color_image[v[inside], u[inside], ::-1]
    return frame_colors, int(np.count_nonzero(inside))


def process_bag(args, bag_path):
    label = bag_label(bag_path)
    output = Path(args.output).expanduser() / label
    output.mkdir(parents=True, exist_ok=True)
    for obsolete_name in (
        'frame_availability.csv',
        'frame_availability.png',
        'summary.txt',
    ):
        obsolete_path = output / obsolete_name
        if obsolete_path.exists():
            obsolete_path.unlink()
    color_dir = output / 'color'
    color_dir.mkdir(exist_ok=True)
    for previous_color_path in color_dir.glob('*.png'):
        previous_color_path.unlink()
    pointcloud_frames_dir = output / 'pointcloud_frames'
    pointcloud_frames_dir.mkdir(exist_ok=True)
    base_ee_trajectory_path = output / 'trajectory_base_ee.csv'
    if base_ee_trajectory_path.exists():
        base_ee_trajectory_path.unlink()
    legacy_pointcloud_path = output / 'pointcloud.ply'
    if legacy_pointcloud_path.exists():
        legacy_pointcloud_path.unlink()
    for previous_frame_path in pointcloud_frames_dir.glob('*.ply'):
        previous_frame_path.unlink()
    trajectory_path = resolve_trajectory_path(args.trajectory, bag_path)
    trajectory = read_trajectory(trajectory_path)
    trajectory_timestamps = [record[0] for record in trajectory]
    calibration_data = {}
    if args.calibration:
        with Path(args.calibration).expanduser().open() as stream:
            calibration_data = yaml.safe_load(stream) or {}
    calibration = calibration_data.get('distance_calibration')
    ee_conversion = end_effector_conversion(calibration_data)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    requested = [
        args.infra_topic,
        args.rgb_topic,
        args.depth_topic,
        args.color_info_topic,
        args.depth_info_topic,
        args.depth_to_tracking_topic,
        args.depth_to_color_topic,
    ]
    missing_required = [
        topic
        for topic in (
            args.infra_topic,
            args.rgb_topic,
            args.depth_topic,
            args.color_info_topic,
            args.depth_info_topic,
            args.depth_to_color_topic,
        )
        if topic not in topic_types
    ]
    if missing_required:
        raise RuntimeError(f'{bag_path} is missing required topics: {missing_required}')
    available = [topic for topic in requested if topic in topic_types]
    missing_optional = [topic for topic in requested if topic not in topic_types and topic not in missing_required]
    message_types = {topic: get_message(topic_types[topic]) for topic in available}
    reader.set_filter(rosbag2_py.StorageFilter(topics=available))

    bridge = CvBridge()
    color_matrix = depth_matrix = None
    color_distortion = None
    depth_to_tracking = None
    depth_to_color = None
    color_info = None
    depth_info = None
    depth_to_tracking_info = None
    depth_to_color_info = None
    image_topic_to_stream = {
        args.infra_topic: 'infra1',
        args.depth_topic: 'depth',
        args.rgb_topic: 'color',
    }
    stream_timestamps = {
        'infra1': set(),
        'depth': set(),
        'color': set(),
    }
    gripper_rows = []
    pointcloud_frame_rows = []
    color_count = depth_count = 0
    point_count = 0
    colorized_points = 0
    frame_index = 0

    while reader.has_next():
        topic, serialized, bag_time = reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        timestamp = stamp_ns(message, bag_time)
        if topic in image_topic_to_stream:
            stream_timestamps[image_topic_to_stream[topic]].add(timestamp)
        elif topic == args.color_info_topic:
            color_matrix = camera_matrix(message)
            color_distortion = np.asarray(message.d, dtype=float)
            color_info = camera_info_record(message, timestamp)
        elif topic == args.depth_info_topic:
            depth_matrix = camera_matrix(message)
            depth_info = camera_info_record(message, timestamp)
        elif topic == args.depth_to_tracking_topic and hasattr(message, 'rotation') and hasattr(message, 'translation'):
            depth_to_tracking = make_transform(message.rotation, message.translation)
            depth_to_tracking_info = extrinsics_record(
                message, timestamp, 'camera_depth_optical_frame', 'camera_infra1_optical_frame'
            )
        elif topic == args.depth_to_color_topic and hasattr(message, 'rotation') and hasattr(message, 'translation'):
            depth_to_color = make_transform(message.rotation, message.translation)
            depth_to_color_info = extrinsics_record(
                message, timestamp, 'camera_depth_optical_frame', 'camera_color_optical_frame'
            )

    T_TRACKING_EE = None
    T_TRACKING_COLOR = None
    conversion_matrices = None
    base_ee_trajectory_full = None
    if ee_conversion is not None:
        T_TRACKING_EE, T_TRACKING_COLOR = tracking_from_end_effector(
            ee_conversion,
            depth_to_tracking,
            depth_to_color,
        )
        base_ee_trajectory_full, conversion_matrices = (
            convert_world_tracking_to_base_ee(
                trajectory,
                ee_conversion['T_BASE_EE_0'],
                T_TRACKING_EE,
            )
        )
        clip_basis = base_ee_trajectory_full
        clip_basis_transform = 'T_BASE_EE(t)'
    else:
        clip_basis = trajectory
        clip_basis_transform = 'T_WORLD_TRACKING(t)'

    _, trajectory_clip_info = clip_trajectory_by_radius(
        clip_basis,
        args.trajectory_clip_head_radius,
        args.trajectory_clip_tail_radius,
    )
    clip_start_timestamp = trajectory_clip_info['start_timestamp_ns']
    clip_end_timestamp = trajectory_clip_info['end_timestamp_ns']
    clipped_trajectory = [
        record
        for record in trajectory
        if clip_start_timestamp <= record[0] <= clip_end_timestamp
    ]
    clipped_base_ee_trajectory = (
        [
            record
            for record in base_ee_trajectory_full
            if clip_start_timestamp <= record[0] <= clip_end_timestamp
        ]
        if base_ee_trajectory_full is not None
        else None
    )
    trajectory_clip_info.update({
        'basis_transform': clip_basis_transform,
        'full_record_count': len(trajectory),
        'clipped_record_count': len(clipped_trajectory),
    })

    write_trajectory(output / 'trajectory_full.csv', trajectory)
    write_trajectory_tum(output / 'trajectory_full_tum.txt', trajectory)
    write_trajectory(output / 'trajectory.csv', clipped_trajectory)
    write_trajectory_tum(output / 'trajectory_tum.txt', clipped_trajectory)
    if clipped_base_ee_trajectory is not None:
        write_trajectory(base_ee_trajectory_path, clipped_base_ee_trajectory)

    availability_rows, trajectory_matches = build_frame_availability(
        stream_timestamps,
        trajectory,
        trajectory_timestamps,
    )
    availability_rows = [
        row
        for row in availability_rows
        if clip_start_timestamp <= row['timestamp_ns'] <= clip_end_timestamp
    ]
    write_missing_timestamps_csv(output / 'missing_timestamps.csv', availability_rows)
    write_missing_timestamps_image(output / 'missing_timestamps.png', availability_rows)
    complete_timestamps = {
        row['timestamp_ns'] for row in availability_rows if row['is_complete']
    }
    if not complete_timestamps:
        raise RuntimeError(
            f'{bag_path} has no timestamps containing infra1, depth, color, and trajectory'
        )
    if depth_matrix is None:
        raise RuntimeError(f'{bag_path} has no usable depth camera intrinsics')
    if color_matrix is None:
        raise RuntimeError(f'{bag_path} has no usable color camera intrinsics')
    if depth_to_color is None:
        raise RuntimeError(f'{bag_path} has no usable depth-to-color extrinsics')

    pending_colors = {}
    pending_depths = {}
    paired_count = 0

    def process_complete_frame(timestamp, color_image, depth_data):
        nonlocal frame_index, point_count, colorized_points, paired_count
        paired_count += 1
        synchronized_frame_index = frame_index
        frame_index += 1
        if synchronized_frame_index % args.pointcloud_stride != 0:
            return

        trajectory_record = trajectory_matches[timestamp]
        height, width = depth_data.shape[:2]
        rows, cols = np.indices((height, width), dtype=np.float32)
        valid = (
            np.isfinite(depth_data)
            & (depth_data > args.min_depth)
            & (depth_data < args.max_depth)
        )
        if args.pixel_stride > 1:
            sample_mask = np.zeros_like(valid, dtype=bool)
            sample_mask[::args.pixel_stride, ::args.pixel_stride] = True
            valid &= sample_mask
        z = depth_data[valid]
        x = (cols[valid] - depth_matrix[0, 2]) * z / depth_matrix[0, 0]
        y = (rows[valid] - depth_matrix[1, 2]) * z / depth_matrix[1, 1]
        depth_points = np.column_stack((x, y, z))
        color_points = (
            (depth_to_color[:3, :3] @ depth_points.T).T
            + depth_to_color[:3, 3]
        )
        frame_colors, colored_count = point_colors_from_color_camera(
            color_points,
            color_image,
            color_matrix,
        )
        colorized_points += colored_count
        point_count += len(color_points)
        trajectory_timestamp = trajectory_record[0]
        pointcloud_filename = f'{timestamp}.ply'
        write_ply(
            pointcloud_frames_dir / pointcloud_filename,
            color_points,
            frame_colors,
            comments=[
                f'frame_timestamp_ns {timestamp}',
                f'infra1_timestamp_ns {timestamp}',
                f'depth_timestamp_ns {timestamp}',
                f'color_timestamp_ns {timestamp}',
                f'trajectory_timestamp_ns {trajectory_timestamp}',
                'coordinate_frame camera_color_optical_frame',
                'source_frame camera_depth_optical_frame',
                'transform T_COLOR_DEPTH',
                'units meter',
            ],
        )
        pointcloud_frame_rows.append([
            synchronized_frame_index,
            timestamp,
            f'{timestamp / 1e9:.9f}',
            trajectory_timestamp,
            f'{trajectory_timestamp / 1e9:.9f}',
            f'{(timestamp - trajectory_timestamp) / 1e9:.9f}',
            timestamp,
            '0.000000000',
            len(color_points),
            f'pointcloud_frames/{pointcloud_filename}',
            'camera_color_optical_frame',
        ])

    image_reader = rosbag2_py.SequentialReader()
    image_reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=''),
        rosbag2_py.ConverterOptions('', ''),
    )
    image_reader.set_filter(
        rosbag2_py.StorageFilter(topics=[args.rgb_topic, args.depth_topic])
    )
    while image_reader.has_next():
        topic, serialized, bag_time = image_reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        timestamp = stamp_ns(message, bag_time)
        if timestamp not in complete_timestamps:
            continue
        if topic == args.rgb_topic:
            color_image = decode_image(
                message,
                bridge,
                args.rgb_topic.endswith('/compressed'),
            )
            if not cv2.imwrite(str(color_dir / f'{timestamp}.png'), color_image):
                raise RuntimeError(f'Unable to write color image at {timestamp}')
            measurement = marker_measurement(
                color_image,
                color_matrix,
                color_distortion,
                calibration,
            )
            if measurement:
                distance, percentage = measurement
                gripper_rows.append([
                    timestamp,
                    timestamp / 1e9,
                    distance,
                    percentage,
                ])
            pending_colors[timestamp] = color_image
            color_count += 1
        elif topic == args.depth_topic:
            pending_depths[timestamp] = depth_image(message, bridge)
            depth_count += 1
        if timestamp in pending_colors and timestamp in pending_depths:
            process_complete_frame(
                timestamp,
                pending_colors.pop(timestamp),
                pending_depths.pop(timestamp),
            )

    if paired_count != len(complete_timestamps):
        raise RuntimeError(
            f'Expected {len(complete_timestamps)} synchronized frames but decoded '
            f'{paired_count}'
        )

    trajectory_conversion_info = None
    if ee_conversion is not None:
        trajectory_conversion_info = {
            'naming_rule': {
                'T_A_B': 'pose of target frame B expressed in origin/reference frame A',
                'R_A_B': 'orientation of target frame B expressed in frame A',
            },
            'source_trajectory': {
                'file': 'trajectory_full.csv',
                'transform': 'T_WORLD_TRACKING(t)',
                'world_frame': 'persistent ORB-SLAM Atlas map frame',
                'tracking_frame': TRACKING_FRAME,
            },
            'converted_trajectory': {
                'file': base_ee_trajectory_path.name,
                'transform': 'T_BASE_EE(t)',
                'base_frame': 'UMI handle bottom-center frame',
                'ee_frame': ee_conversion['ee_frame'],
                'includes_umi_to_yam_local_z_180_deg': (
                    ee_conversion['includes_umi_to_yam_rotation']
                ),
            },
            'formula': {
                'relative_tracking_motion': (
                    'T_TRACKING_0_TRACKING(t) = T_TRACKING_0_WORLD * '
                    'T_WORLD_TRACKING(t)'
                ),
                'base_tracking_first': (
                    'T_BASE_TRACKING_0 = T_BASE_EE_0 * T_EE_TRACKING'
                ),
                'base_ee': (
                    'T_BASE_EE(t) = T_BASE_TRACKING_0 * T_TRACKING_0_TRACKING(t) * '
                    'T_TRACKING_EE'
                ),
                'expanded': (
                    'T_BASE_EE(t) = T_BASE_EE_0 * T_EE_TRACKING * '
                    'T_TRACKING_0_WORLD * T_WORLD_TRACKING(t) * T_TRACKING_EE'
                ),
            },
            'assumption': (
                'The first T_WORLD_TRACKING sample corresponds to the calibrated '
                'initial end-effector pose T_BASE_EE_0.'
            ),
            'hand_eye_source_frame': ee_conversion['camera_frame'],
            'ee_frame_convention': ee_conversion['ee_frame'],
            'includes_umi_to_yam_local_z_180_deg': (
                ee_conversion['includes_umi_to_yam_rotation']
            ),
            'T_BASE_EE_0': ee_conversion['T_BASE_EE_0'].tolist(),
            'T_CAMERA_EE': ee_conversion['T_CAMERA_EE'].tolist(),
            'T_TRACKING_COLOR': T_TRACKING_COLOR.tolist(),
            'T_TRACKING_EE': T_TRACKING_EE.tolist(),
            'T_WORLD_TRACKING_0': conversion_matrices['T_WORLD_TRACKING_0'].tolist(),
            'T_TRACKING_0_WORLD': conversion_matrices['T_TRACKING_0_WORLD'].tolist(),
            'T_EE_TRACKING': conversion_matrices['T_EE_TRACKING'].tolist(),
            'T_BASE_TRACKING_0': conversion_matrices['T_BASE_TRACKING_0'].tolist(),
            'first_pose_max_abs_error': conversion_matrices['first_pose_max_abs_error'],
            'trajectory_clipping': trajectory_clip_info,
        }

    with (output / 'pointcloud_frames.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'frame_index',
            'depth_timestamp_ns',
            'depth_timestamp_sec',
            'trajectory_timestamp_ns',
            'trajectory_timestamp_sec',
            'depth_minus_trajectory_sec',
            'color_timestamp_ns',
            'color_minus_depth_sec',
            'point_count',
            'pointcloud_file',
            'coordinate_frame',
        ])
        writer.writerows(pointcloud_frame_rows)
    with (output / 'gripper.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['timestamp_ns', 'timestamp_sec', 'opening_m', 'opening_percent'])
        writer.writerows(gripper_rows)
    calibration_output = {
        'units': {'translation': 'meter', 'depth': 'meter'},
        'coordinate_convention': {
            'naming_rule': 'T_A_B is the pose of target frame B expressed in reference frame A',
            'trajectory_transform': 'T_WORLD_TRACKING; tracking frame is camera_infra1_optical_frame',
            'trajectory_origin': (
                'ORB-SLAM3 SaveTrajectoryTUM places the Atlas keyframe with the smallest ID at the origin. '
                'Because locating loads an existing Atlas, this is the loaded map origin, not the first frame of this bag.'
            ),
            'pointcloud': (
                'each PLY is expressed in camera_color_optical_frame at its own timestamp'
            ),
            'pointcloud_transform': 'p_COLOR = R_COLOR_DEPTH * p_DEPTH + t_COLOR_DEPTH',
            'pointcloud_world_transform_applied': False,
        },
        'processing': {
            'pointcloud_stride': int(args.pointcloud_stride),
            'pixel_stride': int(args.pixel_stride),
            'min_depth_m': float(args.min_depth),
            'max_depth_m': float(args.max_depth),
            'trajectory_clipping': trajectory_clip_info,
            'alignment_policy': {
                'camera_streams': 'exact equality of ROS header timestamps in nanoseconds',
                'trajectory_match_tolerance_ns': TRAJECTORY_MATCH_TOLERANCE_NS,
                'trajectory_tolerance_reason': (
                    'ORB-SLAM TUM timestamps are written with limited decimal precision'
                ),
                'required_streams': ['infra1', 'depth', 'color', 'trajectory'],
            },
            'missing_timestamps_csv': 'missing_timestamps.csv',
            'missing_timestamps_image': 'missing_timestamps.png',
        },
        'topics': {
            'infra1_image': args.infra_topic,
            'color_image': args.rgb_topic,
            'depth_image': args.depth_topic,
            'color_camera_info': args.color_info_topic,
            'depth_camera_info': args.depth_info_topic,
            'depth_to_tracking': args.depth_to_tracking_topic,
            'depth_to_color': args.depth_to_color_topic,
        },
        'color_camera': color_info,
        'depth_camera': depth_info,
        'extrinsics': {
            'depth_to_tracking': depth_to_tracking_info,
            'depth_to_color': depth_to_color_info,
        },
        'trajectory_conversion': trajectory_conversion_info,
    }
    with (output / 'camera_calibration.yaml').open('w') as stream:
        yaml.safe_dump(calibration_output, stream, sort_keys=False)
    if trajectory_conversion_info is not None:
        base_ee_tree_entry = '├── trajectory_base_ee.csv\n'
        converted_ee_name = (
            'YAM EE/TCP'
            if ee_conversion['includes_umi_to_yam_rotation']
            else 'legacy UMI EE'
        )
        base_ee_description = (
            f'| `trajectory_base_ee.csv` | `T_BASE_EE(t)` | CSV | {converted_ee_name} '
            '在 UMI 把手底部中心座標系中的姿態 |\n'
        )
        base_ee_conversion = (
            '### `T_BASE_EE(t)` 轉換\n\n'
            '```text\n'
            'T_TRACKING_0_TRACKING(t) = T_TRACKING_0_WORLD * T_WORLD_TRACKING(t)\n'
            'T_BASE_TRACKING_0 = T_BASE_EE_0 * T_EE_TRACKING\n'
            'T_BASE_EE(t) = T_BASE_TRACKING_0 * T_TRACKING_0_TRACKING(t)\n'
            '               * T_TRACKING_EE\n'
            '```\n\n'
            '- `T_TRACKING_0_WORLD` 是 `T_WORLD_TRACKING_0` 對調 frame 前後順序後的變換。\n'
            '- `T_EE_TRACKING` 是 `T_TRACKING_EE` 對調 frame 前後順序後的變換。\n'
            '- `T_TRACKING_EE` 由 hand-eye calibration 與 bag 內相機外參組合得到。\n'
            f'- EE frame convention：`{ee_conversion["ee_frame"]}`；'
            f'已包含 UMI→YAM local-Z 180°：'
            f'`{ee_conversion["includes_umi_to_yam_rotation"]}`。\n'
            '- 假設軌跡第一筆姿態對應校準時定義的 `T_BASE_EE_0`。\n'
            f'- 完整來源軌跡第一筆轉換結果相對 `T_BASE_EE_0` 的最大絕對誤差為 '
            f'`{trajectory_conversion_info["first_pose_max_abs_error"]:.3e}`。\n\n'
        )
    else:
        base_ee_tree_entry = ''
        base_ee_description = ''
        base_ee_conversion = (
            '### `T_BASE_EE(t)` 轉換\n\n'
            '本次未提供完整 end-effector calibration，因此沒有輸出 '
            '`trajectory_base_ee.csv`。\n\n'
        )
    missing_timestamp_count = len(availability_rows) - len(complete_timestamps)
    (output / 'DATASET_INFO.md').write_text(
        f'# {label} RGB-D 處理資料說明\n\n'
        '本資料集由 ROS 2 bag、ORB-SLAM3 軌跡及夾爪 ArUco 校準離線產生。'
        '位置、深度與點雲的長度單位皆為公尺。\n\n'
        '## 資料夾結構\n\n'
        '```text\n'
        f'{label}/\n'
        '├── DATASET_INFO.md\n'
        '├── camera_calibration.yaml\n'
        '├── trajectory_full.csv\n'
        '├── trajectory_full_tum.txt\n'
        '├── trajectory.csv\n'
        '├── trajectory_tum.txt\n'
        f'{base_ee_tree_entry}'
        '├── gripper.csv\n'
        '├── missing_timestamps.csv\n'
        '├── missing_timestamps.png\n'
        '├── pointcloud_frames.csv\n'
        '├── color/\n'
        '│   └── <timestamp_ns>.png\n'
        '└── pointcloud_frames/\n'
        '    └── <timestamp_ns>.ply\n'
        '```\n\n'
        '## 軌跡檔案說明\n\n'
        '| 檔案 | 變換 | 格式 | 說明 |\n'
        '| --- | --- | --- | --- |\n'
        '| `trajectory_full.csv` | `T_WORLD_TRACKING(t)` | CSV | 完整 ORB-SLAM3 原始相機軌跡 |\n'
        '| `trajectory_full_tum.txt` | `T_WORLD_TRACKING(t)` | TUM | 完整原始軌跡的 TUM 副本 |\n'
        '| `trajectory.csv` | `T_WORLD_TRACKING(t)` | CSV | 半徑裁切後的 ORB-SLAM3 軌跡 |\n'
        '| `trajectory_tum.txt` | `T_WORLD_TRACKING(t)` | TUM | 半徑裁切後的 TUM 副本 |\n'
        f'{base_ee_description}'
        '\n'
        '- CSV pose 欄位為 `timestamp_ns, timestamp_sec, x_m, y_m, z_m, qx, qy, qz, qw`。\n'
        '- TUM 欄位為 `timestamp_sec x y z qx qy qz qw`。\n'
        '- `WORLD` 原點是 ORB-SLAM3 Atlas 中 ID 最小的 keyframe；定位模式載入既有 '
        'Atlas，因此不是這份 bag 的第一個 frame。\n'
        '- 完整原始軌跡保存在 `trajectory_full.*`；其他輸出只保留裁切區間內的 timestamp。\n\n'
        '### 半徑裁切\n\n'
        f'- 裁切基準：`{trajectory_clip_info["basis_transform"]}` 的平移位置。\n'
        f'- 頭部半徑：`{trajectory_clip_info["head_radius_m"]:.6f} m`；'
        f'尾部半徑：`{trajectory_clip_info["tail_radius_m"]:.6f} m`。\n'
        '- 起點從軌跡頭部向後找第一個「距離第一點大於頭部半徑」的 pose。\n'
        '- 終點從軌跡尾部向前找最後一個位於尾部半徑外的 pose，'
        '並包含其後第一個回到尾部半徑內的 pose；若最後仍在半徑外，終點使用最後一筆。\n'
        f'- 是否觸發裁切：`{trajectory_clip_info["triggered"]}`；'
        f'索引範圍：`{trajectory_clip_info["start_index"]}`–'
        f'`{trajectory_clip_info["end_index"]}`；'
        f'timestamp：`{clip_start_timestamp}`–`{clip_end_timestamp}`。\n'
        f'- 完整／裁切後 pose 數：`{len(trajectory):,}`／`{len(clipped_trajectory):,}`。\n\n'
        f'{base_ee_conversion}'
        '## 座標系 Frame 說明\n\n'
        '命名規則：`T_A_B` 表示 frame B 在 frame A 中的 pose，'
        '`R_A_B` 表示 frame B 的方向在 frame A 中的表示。\n\n'
        '| Frame | 實際 frame / 用途 |\n'
        '| --- | --- |\n'
        '| `WORLD` | ORB-SLAM3 persistent Atlas map frame |\n'
        '| `TRACKING` | `camera_infra1_optical_frame`，ORB-SLAM3 使用的相機 frame |\n'
        '| `DEPTH` | `camera_depth_optical_frame`，depth 點的原始座標系 |\n'
        '| `COLOR` | `camera_color_optical_frame`，彩色影像與 ArUco 校準 frame |\n'
        '| `BASE` | UMI 把手底部的中心位置與其固定座標軸 |\n'
        f'| `EE` | 校準定義的 `{ee_conversion["ee_frame"] if ee_conversion else "unavailable"}` frame |\n\n'
        '- Bag 的 `depth_to_infra1` 表示 `T_TRACKING_DEPTH`。\n'
        '- Bag 的 `depth_to_color` 表示 `T_COLOR_DEPTH`。\n'
        '- 相機內參、畸變、影像尺寸及完整 4×4 外參保存在 '
        '`camera_calibration.yaml`。\n\n'
        '## 點雲說明\n\n'
        '- 每個 `pointcloud_frames/<timestamp_ns>.ply` 對應一個完整同步 frame，'
        '不會融合成單一 `pointcloud.ply`。\n'
        '- 每份 PLY 的點座標位於該 timestamp 的 `COLOR` '
        '（`camera_color_optical_frame`），單位為公尺。\n'
        '- 點座標只套用 `p_COLOR = R_COLOR_DEPTH * p_DEPTH + t_COLOR_DEPTH`，'
        '不套用 ORB-SLAM3 的 `T_WORLD_TRACKING(t)`。\n'
        '- 不同 timestamp 的 `COLOR` 是隨相機移動的局部 frame，'
        '因此不同 PLY 不可直接疊加或融合；若需要全域點雲，必須另外搭配軌跡轉換。\n'
        '- Depth 3D 點經 `T_COLOR_DEPTH` 轉到 color camera，再使用 color 內參取色；'
        '不會直接以相同 pixel 座標取色。\n'
        '- `pointcloud_frames.csv` 記錄每個 PLY 的 frame index、時間戳、點數、路徑及 frame。\n'
        f'- 每 `{args.pointcloud_stride}` 個完整 frame 輸出一份點雲；depth pixel stride 為 '
        f'`{args.pixel_stride}`，有效 depth 範圍為 `{args.min_depth}`–`{args.max_depth}` m。\n\n'
        '## Color 影像說明\n\n'
        '- `color/<timestamp_ns>.png` 使用 color ROS header timestamp 命名。\n'
        '- 只有 infra1、depth、color 與 trajectory 都存在的 timestamp 才會輸出。\n'
        '- Infra1、depth 與 color timestamp 必須 nanosecond 完全相同。\n'
        f'- ORB-SLAM3 TUM timestamp 因文字精度限制，使用固定 '
        f'`{TRAJECTORY_MATCH_TOLERANCE_NS / 1e6:.1f} ms` 容許值配對。\n\n'
        '## 夾爪與缺失資料\n\n'
        '- `gripper.csv` 的 `opening_m` 保存 ArUco marker 中心距離，'
        '`opening_percent` 保存校準後的開合百分比。\n'
        '- 沒有同時偵測到兩個 marker 的 frame 不會寫入。\n'
        '- `missing_timestamps.csv` 只記錄不完整 timestamp 與 `missing_streams`。\n'
        '- `missing_timestamps.png` 使用 Matplotlib 將缺失位置畫成紅線。\n\n'
        '## 資料 Summary\n\n'
        '| 項目 | 數量 / 值 |\n'
        '| --- | ---: |\n'
        f'| 原始 infra1 frames | {len(stream_timestamps["infra1"]):,} |\n'
        f'| 原始 depth frames | {len(stream_timestamps["depth"]):,} |\n'
        f'| 原始 color frames | {len(stream_timestamps["color"]):,} |\n'
        f'| 完整 ORB-SLAM3 trajectory records | {len(trajectory):,} |\n'
        f'| 裁切後 trajectory records | {len(clipped_trajectory):,} |\n'
        f'| 裁切區間候選 timestamps | {len(availability_rows):,} |\n'
        f'| 完整同步 frames | {len(complete_timestamps):,} |\n'
        f'| 缺失 timestamps | {missing_timestamp_count:,} |\n'
        f'| 輸出 color images | {color_count:,} |\n'
        f'| 輸出 pointcloud files | {len(pointcloud_frame_rows):,} |\n'
        f'| 點雲總點數 | {point_count:,} |\n'
        f'| 成功彩色化點數 | {colorized_points:,} |\n'
        f'| 夾爪量測筆數 | {len(gripper_rows):,} |\n\n'
        '### 輸入來源\n\n'
        f'- Bag：`{bag_path}`\n'
        f'- ORB-SLAM3 trajectory：`{trajectory_path}`\n'
        f'- 缺少的 optional topics：`{missing_optional}`\n'
    )
    print(
        f'{bag_path.name}: trajectory={len(trajectory)}->{len(clipped_trajectory)}, '
        f'clip={trajectory_clip_info["triggered"]}, color={color_count}, '
        f'depth={depth_count}, points={point_count}, gripper={len(gripper_rows)} '
        f'-> {output}'
    )


def main():
    parser = argparse.ArgumentParser(description='Process rosbag2 RGB-D recordings offline.')
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--trajectory', required=True, type=Path)
    parser.add_argument('--output', default=data_root() / 'processed', type=Path)
    parser.add_argument('--calibration', default=None, type=Path)
    parser.add_argument('--infra-topic', default=INFRA_DEFAULT)
    parser.add_argument('--rgb-topic', default=RGB_DEFAULT)
    depth_group = parser.add_mutually_exclusive_group()
    depth_group.add_argument('--depth-topic', default=None)
    depth_group.add_argument(
        '--raw-depth',
        action='store_true',
        help=f'Use legacy raw depth topic {DEPTH_DEFAULT}',
    )
    depth_group.add_argument(
        '--compressed-depth',
        action='store_true',
        help=argparse.SUPPRESS,
    )
    parser.add_argument('--color-info-topic', default=COLOR_INFO_DEFAULT)
    parser.add_argument('--depth-info-topic', default=DEPTH_INFO_DEFAULT)
    parser.add_argument('--depth-to-tracking-topic', '--extrinsics-topic', default=DEPTH_TO_TRACKING_DEFAULT, dest='depth_to_tracking_topic')
    parser.add_argument('--depth-to-color-topic', default=DEPTH_TO_COLOR_DEFAULT)
    parser.add_argument('--pointcloud-stride', type=int, default=1)
    parser.add_argument('--pixel-stride', type=int, default=4)
    parser.add_argument('--min-depth', type=float, default=0.0)
    parser.add_argument('--max-depth', type=float, default=5.0)
    parser.add_argument(
        '--trajectory-clip-head-radius',
        '--trajectory_clip_head_radius',
        type=float,
        default=0.02,
        help='Start after moving this far from the first pose (default: 0.02 m)',
    )
    parser.add_argument(
        '--trajectory-clip-tail-radius',
        '--trajectory_clip_tail_radius',
        type=float,
        default=0.10,
        help='End after returning within this radius of the first pose (default: 0.10 m)',
    )
    parser.add_argument(
        '--trajectory-clip-radius',
        '--trajectory_clip_radius',
        type=float,
        default=None,
        help='Deprecated: set both head and tail clipping radii to this value',
    )
    args = parser.parse_args()
    if args.raw_depth:
        args.depth_topic = DEPTH_DEFAULT
    elif args.depth_topic:
        args.depth_topic = args.depth_topic
    else:
        args.depth_topic = DEPTH_COMPRESSED_DEFAULT
    if args.pointcloud_stride < 1 or args.pixel_stride < 1:
        parser.error('stride values must be positive')
    if args.trajectory_clip_radius is not None:
        args.trajectory_clip_head_radius = args.trajectory_clip_radius
        args.trajectory_clip_tail_radius = args.trajectory_clip_radius
        print(
            'WARNING: --trajectory-clip-radius is deprecated; use the separate '
            '--trajectory-clip-head-radius and --trajectory-clip-tail-radius options'
        )
    if (
        args.trajectory_clip_head_radius < 0.0
        or args.trajectory_clip_tail_radius < 0.0
    ):
        parser.error('trajectory clipping radii must be non-negative')
    for bag in args.bags:
        process_bag(args, bag.expanduser().resolve())


if __name__ == '__main__':
    main()
