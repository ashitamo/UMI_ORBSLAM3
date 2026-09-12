#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Line3DCollection


def load_base_ee_trajectory(dataset: Path) -> tuple[np.ndarray, Path]:
    trajectory_path = dataset / 'trajectory_base_ee.csv'
    if not dataset.is_dir():
        raise RuntimeError(f'Dataset directory not found: {dataset}')
    if not trajectory_path.is_file():
        raise RuntimeError(
            f'T_BASE_EE trajectory not found: {trajectory_path}\n'
            'Run process_rgbd_bags with a valid end-effector calibration first.'
        )

    records = []
    required_fields = ('timestamp_sec', 'x_m', 'y_m', 'z_m')
    with trajectory_path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        missing_fields = [
            field for field in required_fields if field not in (reader.fieldnames or [])
        ]
        if missing_fields:
            raise RuntimeError(
                f'{trajectory_path} is missing columns: {missing_fields}'
            )
        for row in reader:
            values = [float(row[field]) for field in required_fields]
            if np.all(np.isfinite(values)):
                records.append(values)

    if len(records) < 2:
        raise RuntimeError('At least two valid T_BASE_EE poses are required')
    data = np.asarray(records, dtype=np.float64)
    data = data[np.argsort(data[:, 0])]
    data = data[np.concatenate(([True], np.diff(data[:, 0]) > 0.0))]
    if len(data) < 2:
        raise RuntimeError('At least two unique trajectory timestamps are required')
    return data, trajectory_path


def set_axes_equal(axis, points: np.ndarray) -> None:
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    center = (minimum + maximum) / 2.0
    radius = max(float((maximum - minimum).max()) / 2.0, 1e-3)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)
    axis.set_box_aspect((1, 1, 1))


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Visualize trajectory_base_ee.csv colored by translational speed.'
    )
    parser.add_argument(
        'dataset',
        type=Path,
        help='Processed dataset directory containing trajectory_base_ee.csv',
    )
    parser.add_argument('-o', '--output', type=Path, default=None)
    parser.add_argument('--max-gap', type=float, default=1.0)
    parser.add_argument('--percentile', type=float, default=95.0)
    parser.add_argument('--line-width', type=float, default=3.0)
    parser.add_argument('--no-show', action='store_true')
    args = parser.parse_args()

    if args.max_gap <= 0:
        parser.error('--max-gap must be positive')
    if not 0 < args.percentile <= 100:
        parser.error('--percentile must be in (0, 100]')
    if args.line_width <= 0:
        parser.error('--line-width must be positive')

    dataset = args.dataset.expanduser().resolve()
    data, trajectory_path = load_base_ee_trajectory(dataset)
    timestamps = data[:, 0]
    positions = data[:, 1:4]

    delta_time = np.diff(timestamps)
    delta_position = np.diff(positions, axis=0)
    valid = (delta_time > 0.0) & (delta_time <= args.max_gap)
    if not np.any(valid):
        raise RuntimeError('No valid segments remain; increase --max-gap')

    segments = np.stack(
        (positions[:-1][valid], positions[1:][valid]), axis=1
    )
    speeds = np.linalg.norm(delta_position[valid], axis=1) / delta_time[valid]
    color_maximum = max(
        float(np.percentile(speeds, args.percentile)), 1e-9
    )
    normalization = Normalize(vmin=0.0, vmax=color_maximum)
    color_map = plt.get_cmap('turbo')

    figure = plt.figure(figsize=(11, 8))
    axis = figure.add_subplot(111, projection='3d')
    line = Line3DCollection(
        segments,
        cmap=color_map,
        norm=normalization,
        linewidth=args.line_width,
    )
    line.set_array(np.clip(speeds, 0.0, color_maximum))
    axis.add_collection3d(line)
    axis.scatter(*positions[0], marker='o', s=60, c='black', label='Start')
    axis.scatter(*positions[-1], marker='X', s=70, c='black', label='End')

    axis.set_xlabel('BASE X [m]')
    axis.set_ylabel('BASE Y [m]')
    axis.set_zlabel('BASE Z [m]')
    axis.set_title(f'{dataset.name}: T_BASE_EE trajectory colored by speed')
    set_axes_equal(axis, positions)
    axis.legend(loc='upper right')

    scalar_map = ScalarMappable(norm=normalization, cmap=color_map)
    scalar_map.set_array([])
    colorbar = figure.colorbar(scalar_map, ax=axis, pad=0.10, shrink=0.78)
    colorbar.set_label(
        f'EE translational speed [m/s], clipped at P{args.percentile:g}'
    )

    duration = timestamps[-1] - timestamps[0]
    total_distance = np.linalg.norm(delta_position, axis=1).sum()
    statistics = (
        f'Poses: {len(data)}\n'
        f'Duration: {duration:.2f} s\n'
        f'Distance: {total_distance:.3f} m\n'
        f'Mean speed: {speeds.mean():.3f} m/s\n'
        f'Max speed: {speeds.max():.3f} m/s'
    )
    axis.text2D(
        0.02,
        0.98,
        statistics,
        transform=axis.transAxes,
        va='top',
        bbox={'boxstyle': 'round', 'facecolor': 'white', 'alpha': 0.85},
    )
    figure.tight_layout()

    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output, dpi=220, bbox_inches='tight')
        print(f'Saved: {output}')

    print(f'Dataset: {dataset}')
    print(f'Trajectory: {trajectory_path}')
    print(f'Poses: {len(data)}')
    print(f'Valid segments: {len(segments)}')
    print(f'Mean speed: {speeds.mean():.6f} m/s')
    print(f'Median speed: {np.median(speeds):.6f} m/s')
    print(f'Maximum speed: {speeds.max():.6f} m/s')
    print(f'P{args.percentile:g} color maximum: {color_maximum:.6f} m/s')

    if args.no_show:
        plt.close(figure)
    else:
        plt.show()


if __name__ == '__main__':
    main()
