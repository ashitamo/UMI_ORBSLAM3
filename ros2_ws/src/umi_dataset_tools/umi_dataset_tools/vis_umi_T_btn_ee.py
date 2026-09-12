#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import yaml


DEFAULT_CALIBRATION_FILE = (
    Path(__file__).resolve().parent.parent / 'aruco_calibration.yaml'
)

parser = argparse.ArgumentParser(
    description='Visualize Base, YAM EE/TCP, and calibrated camera frames.'
)
parser.add_argument(
    '--calibration',
    type=Path,
    default=DEFAULT_CALIBRATION_FILE,
    help='ArUco calibration YAML path',
)
args = parser.parse_args()
CALIBRATION_FILE = args.calibration.expanduser().resolve()

# ============================================================
# 1) CAD 幾何輸入
# ============================================================
# 你的 EE 點定義：Min / Max XYZ 各自取平均
# 單位：mm
MIN_DIST_MM = np.array([155.600, 1.385, 101.300], dtype=np.float64)
MAX_DIST_MM = np.array([165.600, 1.474, 123.300], dtype=np.float64)

EE_POSITION_MM = 0.5 * (MIN_DIST_MM + MAX_DIST_MM)

# ============================================================
# 2) ArUco 繞自己 Z 軸的固定安裝偏角
# ============================================================
# 如果你把 ID3 貼在末端時，圖案繞著「穿入紙面」方向多轉了一個角度，
# 就改這裡。
#
# 常見可試：
#   0, 90, -90, 180
#
ARUCO_Z_ROT_OFFSET_DEG = 0.0

# ============================================================
# 3) YAM 一致的 EE 名義座標系
# ============================================================
# 根據 YAM tip_right 的慣例，採用：
#
#   EE +Z = Base +X   (夾爪方向)
#   EE +X = +Base Y
#   EE +Y = +Base Z
#
# 這是一組右手座標系，且與你確認的
#   "夾爪方向應當朝 Base X"
# 一致。
#
X_EE_NOM_IN_BASE = np.array([0.0, 1.0, 0.0], dtype=np.float64)
Y_EE_NOM_IN_BASE = np.array([0.0, 0.0, 1.0], dtype=np.float64)
Z_EE_NOM_IN_BASE = np.array([1.0,  0.0,  0.0], dtype=np.float64)

R_BASE_FROM_EE_NOM = np.column_stack([
    X_EE_NOM_IN_BASE,
    Y_EE_NOM_IN_BASE,
    Z_EE_NOM_IN_BASE,
])

# ============================================================
# 4) 在 EE 自己的 Z 軸上套一個固定旋轉
# ============================================================
def rotz_deg(deg):
    rad = np.deg2rad(deg)
    c = np.cos(rad)
    s = np.sin(rad)
    return np.array([
        [ c, -s, 0.0],
        [ s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

R_EE_NOM_FROM_EE_ACTUAL = rotz_deg(ARUCO_Z_ROT_OFFSET_DEG)

# columns are actual EE axes expressed in Base
R_BASE_FROM_EE_MODEL = R_BASE_FROM_EE_NOM @ R_EE_NOM_FROM_EE_ACTUAL

T_BASE_FROM_EE_MODEL = np.eye(4, dtype=np.float64)
T_BASE_FROM_EE_MODEL[:3, :3] = R_BASE_FROM_EE_MODEL
T_BASE_FROM_EE_MODEL[:3, 3] = EE_POSITION_MM / 1000.0


def load_calibrated_transforms(path):
    if not path.is_file():
        raise RuntimeError(f'Calibration YAML not found: {path}')
    with path.open(encoding='utf-8') as stream:
        calibration = yaml.safe_load(stream) or {}
    try:
        end_effector = calibration['end_effector_calibration']
        cad_reference = end_effector['cad_reference']
        measurement = end_effector['measurement']
        if 'T_base_from_yam_ee' in cad_reference and 'yam_ee_pose' in measurement:
            transform_names = ('T_base_from_yam_ee', 'T_yam_ee_from_camera')
            T_base_from_ee = np.asarray(
                cad_reference['T_base_from_yam_ee'], dtype=np.float64
            )
            T_ee_from_camera = np.asarray(
                measurement['yam_ee_pose']['T_yam_ee_from_camera'],
                dtype=np.float64,
            )
        else:
            transform_names = ('T_base_from_umi_ee', 'T_umi_ee_from_camera')
            T_base_from_ee = np.asarray(
                cad_reference['T_base_from_umi_ee'], dtype=np.float64
            )
            T_ee_from_camera = np.asarray(
                measurement['umi_ee_pose']['T_umi_ee_from_camera'],
                dtype=np.float64,
            )
        camera_frame_id = end_effector['measurement']['camera_frame_id']
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            'Calibration YAML must contain matching YAM EE or legacy UMI EE transforms'
        ) from error
    for name, transform in zip(
        transform_names,
        (T_base_from_ee, T_ee_from_camera),
    ):
        if transform.shape != (4, 4) or not np.allclose(transform[3], [0, 0, 0, 1]):
            raise RuntimeError(f'{name} in {path} is not a valid 4x4 transform')
    return T_base_from_ee, T_ee_from_camera, camera_frame_id


T_BASE_FROM_EE, T_EE_FROM_CAMERA, CAMERA_FRAME_ID = (
    load_calibrated_transforms(CALIBRATION_FILE)
)
T_EE_FROM_BASE = np.linalg.inv(T_BASE_FROM_EE)
T_BASE_FROM_CAMERA = T_BASE_FROM_EE @ T_EE_FROM_CAMERA
T_CAMERA_FROM_BASE = np.linalg.inv(T_BASE_FROM_CAMERA)
T_CAMERA_FROM_EE = np.linalg.inv(T_EE_FROM_CAMERA)

R_BASE_FROM_EE = T_BASE_FROM_EE[:3, :3]
R_BASE_FROM_CAMERA = T_BASE_FROM_CAMERA[:3, :3]
EE_POSITION_MM = T_BASE_FROM_EE[:3, 3] * 1000.0
CAMERA_POSITION_MM = T_BASE_FROM_CAMERA[:3, 3] * 1000.0

if not np.allclose(T_BASE_FROM_EE, T_BASE_FROM_EE_MODEL, atol=1e-9):
    print('WARNING: YAML EE frame differs from the current YAM convention.')


def rotation_matrix_to_rpy_xyz(rotation):
    pitch = np.arctan2(
        -rotation[2, 0],
        np.hypot(rotation[0, 0], rotation[1, 0]),
    )
    if np.hypot(rotation[0, 0], rotation[1, 0]) > 1e-12:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=np.float64)


def print_pose_6d(name, transform):
    position = transform[:3, 3]
    rpy_deg = np.degrees(rotation_matrix_to_rpy_xyz(transform[:3, :3]))
    print(
        f'{name}: '
        f'xyz_m=[{position[0]:+.6f}, {position[1]:+.6f}, {position[2]:+.6f}], '
        f'rpy_xyz_deg=[{rpy_deg[0]:+.3f}, {rpy_deg[1]:+.3f}, {rpy_deg[2]:+.3f}]'
    )


# ============================================================
# 5) 印出結果
# ============================================================
print("=" * 76)
print("CAD / YAM / ArUco-Z-offset 視覺化確認")
print("=" * 76)
print(f"Min [mm] = {MIN_DIST_MM}")
print(f"Max [mm] = {MAX_DIST_MM}")
print(f"EE  [mm] = {EE_POSITION_MM}")
print()
print("YAM 名義 EE 定義：")
print("  EE +X = +Base Y")
print("  EE +Y = +Base Z")
print("  EE +Z = +Base X   <-- 夾爪方向")
print()
print(f"ARUCO_Z_ROT_OFFSET_DEG = {ARUCO_Z_ROT_OFFSET_DEG:.3f} deg")
print(f"Calibration YAML = {CALIBRATION_FILE}")
print(f"Camera frame = {CAMERA_FRAME_ID}")
print()
print("R_base_from_ee_nom =")
print(np.array2string(R_BASE_FROM_EE_NOM, precision=6, suppress_small=True))
print()
print("R_base_from_ee =")
print(np.array2string(R_BASE_FROM_EE, precision=6, suppress_small=True))
print()
print("T_base_from_ee =")
print(np.array2string(T_BASE_FROM_EE, precision=9, suppress_small=True))
print()
print("T_ee_from_base =")
print(np.array2string(T_EE_FROM_BASE, precision=9, suppress_small=True))
print()
print("T_ee_from_camera =")
print(np.array2string(T_EE_FROM_CAMERA, precision=9, suppress_small=True))
print()
print("T_base_from_camera =")
print(np.array2string(T_BASE_FROM_CAMERA, precision=9, suppress_small=True))
print()
print(f"Camera position in Base [mm] = {CAMERA_POSITION_MM}")
print()
print(f"det(R_base_from_ee) = {np.linalg.det(R_BASE_FROM_EE):.12f}")
print(f"det(R_base_from_camera) = {np.linalg.det(R_BASE_FROM_CAMERA):.12f}")
print()
print('6D poses: T_A_B = frame B expressed in frame A')
print_pose_6d('T_BASE_EE', T_BASE_FROM_EE)
print_pose_6d('T_EE_BASE', T_EE_FROM_BASE)
print_pose_6d('T_BASE_CAMERA', T_BASE_FROM_CAMERA)
print_pose_6d('T_CAMERA_BASE', T_CAMERA_FROM_BASE)
print_pose_6d('T_EE_CAMERA', T_EE_FROM_CAMERA)
print_pose_6d('T_CAMERA_EE', T_CAMERA_FROM_EE)
print("=" * 76)


# ============================================================
# 6) 畫圖工具
# ============================================================
FRAME_AXIS_LENGTH_MM = 24.0

X_LIMITS = (-25.0, 215.0)
Y_LIMITS = (-85.0, 85.0)
Z_LIMITS = (-30.0, 230.0)


def draw_frame_3d(ax, origin, R, prefix):
    labels = [f"{prefix}X", f"{prefix}Y", f"{prefix}Z"]
    for i in range(3):
        axis = R[:, i]
        ax.quiver(
            origin[0], origin[1], origin[2],
            axis[0] * FRAME_AXIS_LENGTH_MM,
            axis[1] * FRAME_AXIS_LENGTH_MM,
            axis[2] * FRAME_AXIS_LENGTH_MM,
            arrow_length_ratio=0.18,
            linewidth=2.0
        )
        tip = origin + axis * FRAME_AXIS_LENGTH_MM * 1.12
        ax.text(tip[0], tip[1], tip[2], labels[i], fontsize=10, fontweight='bold')


def draw_axis_2d(ax, origin, axis, idx_a, idx_b, label):
    start = np.array([origin[idx_a], origin[idx_b]])
    delta = np.array([axis[idx_a], axis[idx_b]]) * FRAME_AXIS_LENGTH_MM

    if np.linalg.norm(delta) < 0.5:
        ax.scatter([start[0]], [start[1]], s=45, zorder=7)
        ax.text(start[0] + 2.5, start[1] + 2.5, label + " ⊙/⊗", fontsize=8)
        return

    ax.arrow(
        start[0], start[1],
        delta[0], delta[1],
        width=0.4,
        head_width=3.6,
        head_length=5.0,
        length_includes_head=True,
        zorder=6
    )
    tip = start + delta * 1.12
    ax.text(tip[0], tip[1], label, fontsize=9, fontweight='bold')


def draw_frame_2d(ax, origin, R, plane, prefix):
    if plane == "xy":
        a, b = 0, 1
        ax.set_xlabel("X [mm]")
        ax.set_ylabel("Y [mm]")
    elif plane == "xz":
        a, b = 0, 2
        ax.set_xlabel("X [mm]")
        ax.set_ylabel("Z [mm]")
    elif plane == "yz":
        a, b = 1, 2
        ax.set_xlabel("Y [mm]")
        ax.set_ylabel("Z [mm]")
    else:
        raise ValueError(plane)

    draw_axis_2d(ax, origin, R[:, 0], a, b, f"{prefix}X")
    draw_axis_2d(ax, origin, R[:, 1], a, b, f"{prefix}Y")
    draw_axis_2d(ax, origin, R[:, 2], a, b, f"{prefix}Z")

    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal", adjustable="box")


def draw_geometry_2d(ax, plane):
    if plane == "xy":
        a, b = 0, 1
    elif plane == "xz":
        a, b = 0, 2
    elif plane == "yz":
        a, b = 1, 2
    else:
        raise ValueError(plane)

    base = np.zeros(3)
    ee = EE_POSITION_MM
    camera = CAMERA_POSITION_MM

    ax.plot(
        [base[a], ee[a]],
        [base[b], ee[b]],
        "--",
        linewidth=1.8
    )

    ax.scatter([base[a]], [base[b]], s=45, zorder=8)
    ax.scatter([ee[a]], [ee[b]], s=55, zorder=8)
    ax.plot(
        [ee[a], camera[a]],
        [ee[b], camera[b]],
        ":",
        linewidth=1.8
    )
    ax.scatter([camera[a]], [camera[b]], s=65, marker="s", zorder=8)

    ax.text(base[a] + 3, base[b] + 3, "Base", fontsize=9)
    ax.text(ee[a] + 3, ee[b] + 3, "EE", fontsize=9)
    ax.text(camera[a] + 3, camera[b] + 3, "Camera", fontsize=9)


# ============================================================
# 7) 主程式：3D + 2D 視角
# ============================================================
base_origin = np.zeros(3, dtype=np.float64)
R_base = np.eye(3, dtype=np.float64)

fig = plt.figure(figsize=(13, 9))

# 3D
ax3d = fig.add_subplot(2, 2, 1, projection="3d")
ax3d.plot(
    [0.0, EE_POSITION_MM[0]],
    [0.0, EE_POSITION_MM[1]],
    [0.0, EE_POSITION_MM[2]],
    "--",
    linewidth=1.8
)
ax3d.plot(
    [EE_POSITION_MM[0], CAMERA_POSITION_MM[0]],
    [EE_POSITION_MM[1], CAMERA_POSITION_MM[1]],
    [EE_POSITION_MM[2], CAMERA_POSITION_MM[2]],
    ":",
    linewidth=1.8
)
ax3d.scatter([0.0], [0.0], [0.0], s=50)
ax3d.scatter([EE_POSITION_MM[0]], [EE_POSITION_MM[1]], [EE_POSITION_MM[2]], s=60)
ax3d.scatter(
    [CAMERA_POSITION_MM[0]],
    [CAMERA_POSITION_MM[1]],
    [CAMERA_POSITION_MM[2]],
    s=70,
    marker="s"
)

draw_frame_3d(ax3d, base_origin, R_base, "B")
draw_frame_3d(ax3d, EE_POSITION_MM, R_BASE_FROM_EE, "E")
draw_frame_3d(ax3d, CAMERA_POSITION_MM, R_BASE_FROM_CAMERA, "C")

ax3d.text(4, 4, 4, "Base")
ax3d.text(
    EE_POSITION_MM[0] + 4,
    EE_POSITION_MM[1] + 4,
    EE_POSITION_MM[2] + 4,
    "EE"
)
ax3d.text(
    CAMERA_POSITION_MM[0] + 4,
    CAMERA_POSITION_MM[1] + 4,
    CAMERA_POSITION_MM[2] + 4,
    "Camera"
)

ax3d.set_xlim(*X_LIMITS)
ax3d.set_ylim(*Y_LIMITS)
ax3d.set_zlim(*Z_LIMITS)
ax3d.set_box_aspect((
    X_LIMITS[1] - X_LIMITS[0],
    Y_LIMITS[1] - Y_LIMITS[0],
    Z_LIMITS[1] - Z_LIMITS[0],
))
ax3d.set_xlabel("Base X [mm]")
ax3d.set_ylabel("Base Y [mm]")
ax3d.set_zlabel("Base Z [mm]")
ax3d.set_title("3D")
ax3d.view_init(elev=22, azim=-60)

# XY
ax_xy = fig.add_subplot(2, 2, 2)
draw_geometry_2d(ax_xy, "xy")
draw_frame_2d(ax_xy, base_origin, R_base, "xy", "B")
draw_frame_2d(ax_xy, EE_POSITION_MM, R_BASE_FROM_EE, "xy", "E")
draw_frame_2d(ax_xy, CAMERA_POSITION_MM, R_BASE_FROM_CAMERA, "xy", "C")
ax_xy.set_xlim(*X_LIMITS)
ax_xy.set_ylim(*Y_LIMITS)
ax_xy.set_title("XY - Top view")

# XZ
ax_xz = fig.add_subplot(2, 2, 3)
draw_geometry_2d(ax_xz, "xz")
draw_frame_2d(ax_xz, base_origin, R_base, "xz", "B")
draw_frame_2d(ax_xz, EE_POSITION_MM, R_BASE_FROM_EE, "xz", "E")
draw_frame_2d(ax_xz, CAMERA_POSITION_MM, R_BASE_FROM_CAMERA, "xz", "C")
ax_xz.set_xlim(*X_LIMITS)
ax_xz.set_ylim(*Z_LIMITS)
ax_xz.set_title("XZ - Side view")

# YZ
ax_yz = fig.add_subplot(2, 2, 4)
draw_geometry_2d(ax_yz, "yz")
draw_frame_2d(ax_yz, base_origin, R_base, "yz", "B")
draw_frame_2d(ax_yz, EE_POSITION_MM, R_BASE_FROM_EE, "yz", "E")
draw_frame_2d(ax_yz, CAMERA_POSITION_MM, R_BASE_FROM_CAMERA, "yz", "C")
ax_yz.set_xlim(*Y_LIMITS)
ax_yz.set_ylim(*Z_LIMITS)
ax_yz.set_title("YZ - Front view")

fig.suptitle(
    "Base / EE / Camera Frames from ArUco Calibration",
    fontsize=14,
    y=0.98
)

fig.tight_layout(rect=[0.02, 0.02, 0.98, 0.95])
plt.show()
