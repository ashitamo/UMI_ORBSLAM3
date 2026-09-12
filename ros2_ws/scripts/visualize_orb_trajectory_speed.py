#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Line3DCollection


def load_trajectory(path: Path) -> np.ndarray:
    data = np.loadtxt(path, dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 8:
        raise ValueError("Expected: timestamp tx ty tz qx qy qz qw")
    data = data[:, :8]
    data = data[np.all(np.isfinite(data), axis=1)]
    data = data[np.argsort(data[:, 0])]
    keep = np.concatenate(([True], np.diff(data[:, 0]) > 0.0))
    data = data[keep]
    if len(data) < 2:
        raise ValueError("At least two valid poses are required.")
    return data


def set_axes_equal(ax, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) / 2.0
    radius = max(float((maxs - mins).max()) / 2.0, 1e-3)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize an ORB-SLAM3 TUM trajectory colored by speed."
    )
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--max-gap", type=float, default=1.0)
    parser.add_argument("--percentile", type=float, default=95.0)
    parser.add_argument("--line-width", type=float, default=3.0)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    if args.max_gap <= 0:
        raise ValueError("--max-gap must be positive.")
    if not 0 < args.percentile <= 100:
        raise ValueError("--percentile must be in (0, 100].")

    data = load_trajectory(args.trajectory)
    timestamps = data[:, 0]
    positions = data[:, 1:4]

    dt = np.diff(timestamps)
    delta = np.diff(positions, axis=0)
    valid = (dt > 0.0) & (dt <= args.max_gap)

    if not np.any(valid):
        raise ValueError("No valid segments remain. Increase --max-gap.")

    segments = np.stack(
        (positions[:-1][valid], positions[1:][valid]),
        axis=1,
    )
    speeds = np.linalg.norm(delta[valid], axis=1) / dt[valid]

    vmax = max(float(np.percentile(speeds, args.percentile)), 1e-9)
    norm = Normalize(vmin=0.0, vmax=vmax)
    cmap = plt.get_cmap("turbo")

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")

    line = Line3DCollection(
        segments,
        cmap=cmap,
        norm=norm,
        linewidth=args.line_width,
    )
    line.set_array(np.clip(speeds, 0.0, vmax))
    ax.add_collection3d(line)

    ax.scatter(*positions[0], marker="o", s=60, c="black", label="Start")
    ax.scatter(*positions[-1], marker="X", s=70, c="black", label="End")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("ORB-SLAM3 trajectory colored by translational speed")
    set_axes_equal(ax, positions)
    ax.legend(loc="upper right")

    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    colorbar = fig.colorbar(sm, ax=ax, pad=0.10, shrink=0.78)
    colorbar.set_label(f"Speed [m/s], clipped at P{args.percentile:g}")

    duration = timestamps[-1] - timestamps[0]
    total_distance = np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()
    stats = (
        f"Poses: {len(data)}\n"
        f"Duration: {duration:.2f} s\n"
        f"Distance: {total_distance:.2f} m\n"
        f"Mean speed: {speeds.mean():.3f} m/s\n"
        f"Max speed: {speeds.max():.3f} m/s"
    )
    ax.text2D(
        0.02,
        0.98,
        stats,
        transform=ax.transAxes,
        va="top",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    fig.tight_layout()

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=220, bbox_inches="tight")
        print(f"Saved: {args.output}")

    print(f"Poses: {len(data)}")
    print(f"Segments: {len(segments)}")
    print(f"Mean speed: {speeds.mean():.6f} m/s")
    print(f"Median speed: {np.median(speeds):.6f} m/s")
    print(f"Maximum speed: {speeds.max():.6f} m/s")
    print(f"P{args.percentile:g} color maximum: {vmax:.6f} m/s")

    if args.no_show:
        plt.close(fig)
    else:
        plt.show()


if __name__ == "__main__":
    main()
