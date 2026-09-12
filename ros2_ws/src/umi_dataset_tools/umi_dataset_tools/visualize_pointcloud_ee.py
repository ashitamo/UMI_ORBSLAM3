#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml


def pose_transform(row):
    quaternion = np.asarray(
        [row['qx'], row['qy'], row['qz'], row['qw']], dtype=float
    )
    quaternion /= np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = [row['x_m'], row['y_m'], row['z_m']]
    return transform


def read_trajectory(path):
    records = {}
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            numeric_row = {
                key: float(value) if key != 'timestamp_ns' else int(value)
                for key, value in row.items()
            }
            records[numeric_row['timestamp_ns']] = pose_transform(numeric_row)
    return records


def read_pointcloud_index(path):
    frames = []
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            coordinate_frame = row['coordinate_frame']
            if coordinate_frame not in (
                'camera_color_optical_frame',
                'orbslam_world_map',
            ):
                raise RuntimeError(
                    f'Unsupported point-cloud frame {coordinate_frame!r}'
                )
            frames.append({
                'timestamp_ns': int(row['depth_timestamp_ns']),
                'trajectory_timestamp_ns': int(row['trajectory_timestamp_ns']),
                'relative_path': row['pointcloud_file'],
                'point_count': int(row['point_count']),
                'coordinate_frame': coordinate_frame,
            })
    return frames


def read_ascii_ply(path):
    with path.open() as stream:
        if stream.readline().strip() != 'ply':
            raise RuntimeError(f'Not a PLY file: {path}')
        vertex_count = None
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError(f'Incomplete PLY header: {path}')
            fields = line.split()
            if fields[:2] == ['format', 'ascii']:
                pass
            elif fields[:2] == ['element', 'vertex']:
                vertex_count = int(fields[2])
            elif fields[0] == 'end_header':
                break
        if vertex_count is None:
            raise RuntimeError(f'PLY has no vertex count: {path}')
        values = np.loadtxt(stream, max_rows=vertex_count, ndmin=2)
    if values.shape[1] < 6:
        raise RuntimeError(f'PLY must contain XYZRGB vertices: {path}')
    return values[:, :3], np.clip(values[:, 3:6] / 255.0, 0.0, 1.0)


def load_calibration_transforms(path):
    with path.open() as stream:
        data = yaml.safe_load(stream) or {}
    conversion = data.get('trajectory_conversion')
    if not conversion:
        raise RuntimeError(
            'camera_calibration.yaml has no trajectory_conversion; '
            'process the bag with end-effector calibration first'
        )
    if conversion.get('hand_eye_source_frame') != 'camera_color_optical_frame':
        raise RuntimeError('T_CAMERA_EE is not calibrated in camera_color_optical_frame')
    transform = np.asarray(conversion.get('T_CAMERA_EE'), dtype=float)
    if transform.shape != (4, 4):
        raise RuntimeError('trajectory_conversion.T_CAMERA_EE is not a 4x4 matrix')
    T_BASE_TRACKING_0 = np.asarray(
        conversion.get('T_BASE_TRACKING_0'), dtype=float
    )
    T_TRACKING_0_WORLD = np.asarray(
        conversion.get('T_TRACKING_0_WORLD'), dtype=float
    )
    if T_BASE_TRACKING_0.shape != (4, 4) or T_TRACKING_0_WORLD.shape != (4, 4):
        raise RuntimeError(
            'trajectory_conversion requires T_BASE_TRACKING_0 and '
            'T_TRACKING_0_WORLD to display legacy world-frame point clouds'
        )
    return transform, T_BASE_TRACKING_0 @ T_TRACKING_0_WORLD


def transform_points(transform, points):
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def draw_frame(axis, transform, name, length):
    origin = transform[:3, 3]
    colors = ('r', 'g', 'b')
    for index, color in enumerate(colors):
        direction = transform[:3, index] * length
        axis.quiver(*origin, *direction, color=color, linewidth=2)
    axis.text(*origin, f' {name}', fontsize=10)


def equal_axes(axis, points, minimum_span):
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    center = (lower + upper) / 2.0
    span = max(float(np.max(upper - lower)), minimum_span)
    half_span = span * 0.55
    axis.set_xlim(center[0] - half_span, center[0] + half_span)
    axis.set_ylim(center[1] - half_span, center[1] + half_span)
    axis.set_zlim(center[2] - half_span, center[2] + half_span)
    axis.set_box_aspect((1, 1, 1))


class FrameViewer:
    def __init__(
        self,
        dataset,
        frames,
        trajectory,
        T_COLOR_EE,
        T_BASE_WORLD,
        point_size,
        axis_length,
        max_distance,
    ):
        self.dataset = dataset
        self.frames = frames
        self.trajectory = trajectory
        self.T_EE_COLOR = np.linalg.inv(T_COLOR_EE)
        self.T_BASE_WORLD = T_BASE_WORLD
        self.point_size = point_size
        self.axis_length = axis_length
        self.max_distance = max_distance
        self.index = 0
        self.figure = plt.figure(figsize=(12, 9))
        self.axis = self.figure.add_subplot(111, projection='3d')
        self.figure.canvas.mpl_connect('key_press_event', self.on_key)

    def draw(self):
        frame = self.frames[self.index]
        timestamp = frame['timestamp_ns']
        trajectory_timestamp = frame['trajectory_timestamp_ns']
        T_BASE_EE = self.trajectory[trajectory_timestamp]
        T_BASE_COLOR = T_BASE_EE @ self.T_EE_COLOR
        points, colors = read_ascii_ply(
            self.dataset / frame['relative_path']
        )
        if frame['coordinate_frame'] == 'camera_color_optical_frame':
            points_base = transform_points(T_BASE_COLOR, points)
        else:
            points_base = transform_points(self.T_BASE_WORLD, points)
        distances_from_color = np.linalg.norm(
            points_base - T_BASE_COLOR[:3, 3], axis=1
        )
        visible = distances_from_color <= self.max_distance
        points_base = points_base[visible]
        colors = colors[visible]
        if not len(points_base):
            raise RuntimeError(
                f'Frame {timestamp} has no points within {self.max_distance:.3f} m '
                'of the color camera'
            )

        self.axis.clear()
        self.axis.scatter(
            points_base[:, 0],
            points_base[:, 1],
            points_base[:, 2],
            c=colors,
            s=self.point_size,
            depthshade=False,
        )
        draw_frame(self.axis, np.eye(4), 'BASE', self.axis_length)
        draw_frame(self.axis, T_BASE_EE, 'EE', self.axis_length)
        draw_frame(self.axis, T_BASE_COLOR, 'COLOR', self.axis_length)
        bounds = np.vstack((
            points_base,
            np.zeros((1, 3)),
            T_BASE_EE[:3, 3],
            T_BASE_COLOR[:3, 3],
        ))
        equal_axes(self.axis, bounds, self.axis_length * 4.0)
        self.axis.set_xlabel('BASE X [m]')
        self.axis.set_ylabel('BASE Y [m]')
        self.axis.set_zlabel('BASE Z [m]')
        self.axis.set_title(
            f'Frame {self.index + 1}/{len(self.frames)}  timestamp={timestamp}\n'
            f"stored frame={frame['coordinate_frame']}  "
            f'visible={len(points_base)}/{len(points)}  max_distance={self.max_distance:.2f} m\n'
            '←/A: previous   →/D/Space: next   Home/End: first/last   Q/Esc: quit'
        )
        print(
            f'frame={self.index + 1}/{len(self.frames)} timestamp_ns={timestamp} '
            f'trajectory_timestamp_ns={trajectory_timestamp} '
            f'EE_xyz_base={T_BASE_EE[:3, 3]} '
            f'COLOR_xyz_base={T_BASE_COLOR[:3, 3]}'
        )
        self.figure.canvas.draw_idle()

    def on_key(self, event):
        if event.key in ('right', 'd', ' ', 'enter'):
            self.index = min(self.index + 1, len(self.frames) - 1)
        elif event.key in ('left', 'a', 'backspace'):
            self.index = max(self.index - 1, 0)
        elif event.key == 'home':
            self.index = 0
        elif event.key == 'end':
            self.index = len(self.frames) - 1
        elif event.key in ('q', 'escape'):
            plt.close(self.figure)
            return
        else:
            return
        self.draw()

    def show(self):
        self.draw()
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Step through color-frame point clouds with current EE and color-camera poses.'
    )
    parser.add_argument('dataset', type=Path, help='Processed bag directory')
    parser.add_argument('--point-size', type=float, default=1.0)
    parser.add_argument('--axis-length', type=float, default=0.05, help='Frame axis length in meters')
    parser.add_argument(
        '--max-distance',
        type=float,
        default=1.0,
        help='Only display points within this distance of the color camera (default: 1.0 m)',
    )
    args = parser.parse_args()
    if args.max_distance <= 0.0:
        parser.error('--max-distance must be positive')

    dataset = args.dataset.expanduser().resolve()
    if not dataset.is_dir():
        raise RuntimeError(
            f'Processed dataset directory not found: {dataset}\n'
            'Pass a directory containing pointcloud_frames.csv, for example '
            'processed_data/data0815_01'
        )
    frames = read_pointcloud_index(dataset / 'pointcloud_frames.csv')
    trajectory = read_trajectory(dataset / 'trajectory_base_ee.csv')
    T_COLOR_EE, T_BASE_WORLD = load_calibration_transforms(
        dataset / 'camera_calibration.yaml'
    )
    if not frames:
        raise RuntimeError(f'No point-cloud frames found in {dataset}')
    missing = [
        frame['trajectory_timestamp_ns']
        for frame in frames
        if frame['trajectory_timestamp_ns'] not in trajectory
    ]
    if missing:
        raise RuntimeError(
            f'{len(missing)} point-cloud timestamps have no T_BASE_EE pose; first={missing[0]}'
        )
    FrameViewer(
        dataset,
        frames,
        trajectory,
        T_COLOR_EE,
        T_BASE_WORLD,
        args.point_size,
        args.axis_length,
        args.max_distance,
    ).show()


if __name__ == '__main__':
    main()
