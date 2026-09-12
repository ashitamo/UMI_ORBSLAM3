#!/usr/bin/env python3
import math
import select
import sys
import termios
import threading
import tty
from datetime import datetime
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


# ============================================================
# User settings
# ============================================================

# Linear travel calibration markers
MARKER_ID_1 = 0
MARKER_ID_2 = 5
DISTANCE_MARKER_LENGTH = 0.0166       # 16.6 mm

# End-effector marker used to define the YAM-compatible EE/TCP frame
END_EFFECTOR_MARKER_ID = 3
END_EFFECTOR_MARKER_LENGTH = 0.02108  # 21.08 mm    

IMAGE_TOPIC = '/camera/camera/color/image_rect_raw'
CAMERA_INFO_TOPIC = '/camera/camera/color/camera_info'

# Save in the parent directory of this script.
CALIBRATION_FILE = (
    Path(__file__).resolve().parent.parent / 'aruco_calibration.yaml'
)

# ----------------------------------------------------------------
# UMI handle Base -> YAM EE fixed geometry
# ----------------------------------------------------------------
# The desired EE origin is defined from the CAD Min/Max component-wise
# midpoint, exactly as confirmed from the CAD model.
#
# Unit below: mm
CAD_MIN_DIST_MM = np.array([
    155.600,
    1.385,
    101.300,
], dtype=np.float64)

CAD_MAX_DIST_MM = np.array([
    165.600,
    1.474,
    123.300,
], dtype=np.float64)

CAD_EE_POSITION_MM = 0.5 * (CAD_MIN_DIST_MM + CAD_MAX_DIST_MM)
CAD_EE_POSITION_M = CAD_EE_POSITION_MM / 1000.0

# YAM EE axes expressed in the UMI handle bottom-center Base frame.
# The previous UMI convention had the same +Z but opposite +X/+Y.  The local
# Z-axis 180-degree UMI-to-YAM correction is included here, so downstream
# replay code must not apply it again.
#
#   EE +X = +Base Y
#   EE +Y = +Base Z
#   EE +Z = +Base X   <-- gripper/tool direction
#
# Each COLUMN is one EE axis expressed in the Base frame.
R_BASE_FROM_EE = np.array([
    [ 0.0,  0.0,  1.0],
    [ 1.0,  0.0,  0.0],
    [ 0.0,  1.0,  0.0],
], dtype=np.float64)

T_BASE_FROM_EE = np.eye(4, dtype=np.float64)
T_BASE_FROM_EE[:3, :3] = R_BASE_FROM_EE
T_BASE_FROM_EE[:3, 3] = CAD_EE_POSITION_M
T_EE_FROM_BASE = np.linalg.inv(T_BASE_FROM_EE)


# ----------------------------------------------------------------
# ID 3 ArUco -> YAM EE frame convention
# ----------------------------------------------------------------
# Raw ArUco object frame:
#   +X : marker image right
#   +Y : marker image up
#   +Z : out of the paper
#
# Desired YAM EE convention at zero Z-offset:
#   +X : marker image left
#   +Y : marker image up
#   +Z : INTO / THROUGH the paper
#
# This includes the local-Z 180-degree correction from the previous UMI EE
# convention.  Relative to raw ArUco: X and Z are reversed, Y is unchanged.
#
# The paper normal fixes EE +Z, but the printed marker can still be rotated
# around that Z axis.  Use this parameter to compensate the physical sticker
# installation angle without changing the rest of the calibration.
#
# Typical values to test: 0, +90, -90, 180 deg.
ARUCO_Z_ROT_OFFSET_DEG = 0.0

R_RAW_ARUCO_FROM_EE_ZERO = np.array([
    [-1.0, 0.0,  0.0],
    [ 0.0, 1.0,  0.0],
    [0.0,  0.0, -1.0],
], dtype=np.float64)


def make_square_object_points(marker_length):
    """OpenCV SOLVEPNP_IPPE_SQUARE point order."""
    h = marker_length / 2.0
    return np.array([
        [-h,  h, 0.0],  # top-left
        [ h,  h, 0.0],  # top-right
        [ h, -h, 0.0],  # bottom-right
        [-h, -h, 0.0],  # bottom-left
    ], dtype=np.float64)


def make_transform(rotation_matrix, translation):
    """Return a 4x4 homogeneous transform."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return T


def matrix_to_list(matrix):
    return [
        [float(v) for v in row]
        for row in np.asarray(matrix, dtype=np.float64)
    ]


def vector_to_list(vector):
    return [
        float(v)
        for v in np.asarray(vector, dtype=np.float64).reshape(-1)
    ]


def rotz_deg(deg):
    """Rotation around local +Z, degrees."""
    rad = math.radians(deg)
    c = math.cos(rad)
    s = math.sin(rad)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


# Fixed raw-ArUco -> YAM-EE rotation.
#
# R_raw_from_ee = Rx(180 deg) followed by a configurable rotation about
# the EE local +Z axis.
R_RAW_ARUCO_FROM_EE = (
    R_RAW_ARUCO_FROM_EE_ZERO
    @ rotz_deg(ARUCO_Z_ROT_OFFSET_DEG)
)

T_RAW_ARUCO_FROM_EE = make_transform(
    R_RAW_ARUCO_FROM_EE,
    np.zeros(3, dtype=np.float64)
)

T_EE_FROM_RAW_ARUCO = np.linalg.inv(T_RAW_ARUCO_FROM_EE)


def rotation_matrix_to_quaternion_xyzw(R):
    """Convert 3x3 rotation matrix to normalized quaternion [x, y, z, w]."""
    R = np.asarray(R, dtype=np.float64)
    trace = np.trace(R)

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm > 0.0:
        q /= norm
    return q


def rotation_matrix_to_rpy(R):
    """
    ROS/URDF-style roll-pitch-yaw representation:
        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    Returns [roll, pitch, yaw] in radians.
    """
    R = np.asarray(R, dtype=np.float64)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-9

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return np.array([roll, pitch, yaw], dtype=np.float64)


class ArucoCalibrationNode(Node):
    def __init__(self):
        super().__init__('aruco_calibration')

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Camera parameters from CameraInfo.
        self.camera_matrix = None
        self.dist_coeffs = None
        self.have_camera_info = False
        self.camera_frame_id = ''

        # Latest 0/100% measurement.
        self.latest_positions = None
        self.latest_distance = None

        # Latest ID 3 pose.
        self.latest_end_effector_pose = None

        # Stored calibration records.
        self.zero_calibration = None
        self.full_calibration = None
        self.end_effector_calibration = None

        # ArUco detector.
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(
            self.aruco_dict,
            self.aruco_params
        )

        # Two independent physical marker sizes.
        self.distance_obj_points = make_square_object_points(
            DISTANCE_MARKER_LENGTH
        )
        self.end_effector_obj_points = make_square_object_points(
            END_EFFECTOR_MARKER_LENGTH
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )

        self.load_existing_calibration()

        # Terminal keyboard listener.
        self.keyboard_thread = None
        self.terminal_fd = None
        self.old_terminal_settings = None

        if sys.stdin.isatty():
            self.keyboard_thread = threading.Thread(
                target=self.keyboard_loop,
                daemon=True
            )
            self.keyboard_thread.start()

        self.get_logger().info('====================================================')
        self.get_logger().info('ArUco calibration started')
        self.get_logger().info(
            f'Distance markers: IDs {MARKER_ID_1}/{MARKER_ID_2}, '
            f'size = {DISTANCE_MARKER_LENGTH * 1000.0:.2f} mm'
        )
        self.get_logger().info(
            f'End-effector marker: ID {END_EFFECTOR_MARKER_ID}, '
            f'size = {END_EFFECTOR_MARKER_LENGTH * 1000.0:.2f} mm'
        )
        self.get_logger().info(f'Image topic: {IMAGE_TOPIC}')
        self.get_logger().info(f'CameraInfo topic: {CAMERA_INFO_TOPIC}')
        self.get_logger().info('Press [0] : save current IDs 0/5 as 0%')
        self.get_logger().info('Press [1] : save current IDs 0/5 as 100%')
        self.get_logger().info(
            'Press [3] : save ID 3 camera <-> YAM EE transform'
        )
        self.get_logger().info('Press [q] or [ESC] : quit')
        self.get_logger().info(f'YAML file: {CALIBRATION_FILE}')
        self.get_logger().info(
            'ID 3 EE: +Z into paper; X/Y use YAM convention + Z-offset'
        )
        self.get_logger().info(
            f'ArUco Z rotation offset: {ARUCO_Z_ROT_OFFSET_DEG:.1f} deg'
        )
        self.get_logger().info(
            'YAM EE axes: +X=+BaseY, +Y=+BaseZ, +Z=+BaseX'
        )
        self.get_logger().info(
            f'CAD EE midpoint [mm]: {CAD_EE_POSITION_MM.tolist()}'
        )
        self.get_logger().info('====================================================')

    # --------------------------------------------------------
    # Camera info
    # --------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(
            msg.k,
            dtype=np.float64
        ).reshape(3, 3)

        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.dist_coeffs = np.zeros(5, dtype=np.float64)

        self.camera_frame_id = msg.header.frame_id

        if not self.have_camera_info:
            self.have_camera_info = True
            self.get_logger().info(
                f'CameraInfo received. frame_id="{self.camera_frame_id}"'
            )
            self.get_logger().info(
                'K =\n' + np.array2string(
                    self.camera_matrix,
                    precision=4
                )
            )

    # --------------------------------------------------------
    # Pose estimation
    # --------------------------------------------------------
    def estimate_marker_pose(self, corner, object_points):
        img_points = corner[0].astype(np.float64)

        success, rvec, tvec = cv2.solvePnP(
            objectPoints=object_points,
            imagePoints=img_points,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success:
            return None

        return (
            rvec.reshape(3).astype(np.float64),
            tvec.reshape(3).astype(np.float64)
        )

    def build_end_effector_pose(self, raw_rvec, raw_tvec):
        """
        solvePnP gives the raw ArUco pose:

            p_camera = R_camera_from_raw * p_raw + t_camera_from_raw

        We convert it directly to the YAM-compatible EE/TCP frame:

            T_camera_from_ee
                =
            T_camera_from_raw @ T_raw_from_ee

        The EE/TCP origin is the ArUco ID3 center, so only orientation is
        changed here.  EE +Z points into the marker paper.  The fixed local-Z
        180-degree UMI-to-YAM correction is already included, after which the
        sticker installation angle is applied by ARUCO_Z_ROT_OFFSET_DEG.
        """
        R_camera_from_raw, _ = cv2.Rodrigues(
            np.asarray(raw_rvec, dtype=np.float64).reshape(3, 1)
        )

        T_camera_from_raw = make_transform(
            R_camera_from_raw,
            raw_tvec
        )

        T_camera_from_ee = (
            T_camera_from_raw @ T_RAW_ARUCO_FROM_EE
        )

        R_camera_from_ee = T_camera_from_ee[:3, :3]
        t_camera_from_ee = T_camera_from_ee[:3, 3]

        T_raw_from_camera = np.linalg.inv(T_camera_from_raw)
        T_ee_from_camera = np.linalg.inv(T_camera_from_ee)

        corrected_rvec, _ = cv2.Rodrigues(R_camera_from_ee)
        corrected_rvec = corrected_rvec.reshape(3)

        return {
            'raw_rvec': np.asarray(raw_rvec, dtype=np.float64).reshape(3),
            'raw_tvec': np.asarray(raw_tvec, dtype=np.float64).reshape(3),

            'R_camera_from_raw': R_camera_from_raw,
            'T_camera_from_raw': T_camera_from_raw,
            'T_raw_from_camera': T_raw_from_camera,

            'R_camera_from_ee': R_camera_from_ee,
            'T_camera_from_ee': T_camera_from_ee,
            'T_ee_from_camera': T_ee_from_camera,
            't_camera_from_ee': t_camera_from_ee,

            # Backward-compatible aliases for code that used the previous names.
            'R_camera_from_tcp': R_camera_from_ee,
            'T_camera_from_tcp': T_camera_from_ee,
            'T_tcp_from_camera': T_ee_from_camera,

            'corrected_rvec': corrected_rvec,
        }

    # --------------------------------------------------------
    # Image processing
    # --------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().warning(
                f'cv_bridge conversion failed: {e}'
            )
            return

        if frame is None:
            return

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(display, corners, ids)

        if not self.have_camera_info:
            cv2.putText(
                display,
                f'Waiting CameraInfo: {CAMERA_INFO_TOPIC}',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2
            )
            self.show_frame(display)
            return

        distance_poses = {}
        end_effector_pose = None

        if ids is not None:
            for i, corner in enumerate(corners):
                marker_id = int(ids[i][0])

                # ------------------------------------------------
                # IDs 0 / 5: 16.6 mm markers
                # ------------------------------------------------
                if marker_id in (MARKER_ID_1, MARKER_ID_2):
                    pose = self.estimate_marker_pose(
                        corner,
                        self.distance_obj_points
                    )

                    if pose is None:
                        continue

                    rvec, tvec = pose
                    distance_poses[marker_id] = (rvec, tvec)

                    cv2.drawFrameAxes(
                        display,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        DISTANCE_MARKER_LENGTH * 0.35
                    )

                # ------------------------------------------------
                # ID 3: 21.08 mm end-effector marker
                # ------------------------------------------------
                elif marker_id == END_EFFECTOR_MARKER_ID:
                    pose = self.estimate_marker_pose(
                        corner,
                        self.end_effector_obj_points
                    )

                    if pose is None:
                        continue

                    raw_rvec, raw_tvec = pose
                    end_effector_pose = self.build_end_effector_pose(
                        raw_rvec,
                        raw_tvec
                    )

                    # Draw the corrected YAM EE axes, not the raw marker axes.
                    cv2.drawFrameAxes(
                        display,
                        self.camera_matrix,
                        self.dist_coeffs,
                        end_effector_pose['corrected_rvec'],
                        raw_tvec,
                        END_EFFECTOR_MARKER_LENGTH * 1.5
                    )

        # ----------------------------------------------------
        # Update IDs 0/5 distance measurement
        # ----------------------------------------------------
        both_visible = (
            MARKER_ID_1 in distance_poses and
            MARKER_ID_2 in distance_poses
        )

        if both_visible:
            p1 = distance_poses[MARKER_ID_1][1].copy()
            p2 = distance_poses[MARKER_ID_2][1].copy()
            distance = float(np.linalg.norm(p1 - p2))

            with self.lock:
                self.latest_positions = {
                    MARKER_ID_1: p1,
                    MARKER_ID_2: p2,
                }
                self.latest_distance = distance

            self.draw_distance_measurement(
                display,
                p1,
                p2,
                distance
            )
        else:
            with self.lock:
                self.latest_positions = None
                self.latest_distance = None

            cv2.putText(
                display,
                f'0/1 calibration: need IDs {MARKER_ID_1} and {MARKER_ID_2}',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 255),
                2
            )

        # ----------------------------------------------------
        # Update ID 3 end-effector pose
        # ----------------------------------------------------
        with self.lock:
            if end_effector_pose is None:
                self.latest_end_effector_pose = None
            else:
                # Deep enough copy for all numpy arrays.
                self.latest_end_effector_pose = {
                    key: value.copy()
                    if isinstance(value, np.ndarray)
                    else value
                    for key, value in end_effector_pose.items()
                }

        if end_effector_pose is not None:
            self.draw_end_effector_measurement(
                display,
                end_effector_pose
            )
        else:
            cv2.putText(
                display,
                f'3 calibration: ID {END_EFFECTOR_MARKER_ID} not visible',
                (20, 150),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 165, 255),
                2
            )

        self.draw_calibration_status(display)
        self.show_frame(display)

    # --------------------------------------------------------
    # Display
    # --------------------------------------------------------
    def draw_distance_measurement(self, frame, p1, p2, distance):
        green = (0, 255, 0)
        white = (255, 255, 255)

        cv2.putText(
            frame,
            f'Current center distance: {distance:.6f} m',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            green,
            2
        )

        cv2.putText(
            frame,
            (
                f'ID {MARKER_ID_1}: '
                f'X={p1[0]:+.4f} Y={p1[1]:+.4f} Z={p1[2]:+.4f} m'
            ),
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            white,
            1
        )

        cv2.putText(
            frame,
            (
                f'ID {MARKER_ID_2}: '
                f'X={p2[0]:+.4f} Y={p2[1]:+.4f} Z={p2[2]:+.4f} m'
            ),
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            white,
            1
        )

        if (
            self.zero_calibration is not None and
            self.full_calibration is not None
        ):
            d0 = self.zero_calibration['center_distance_m']
            d100 = self.full_calibration['center_distance_m']
            span = d100 - d0

            if abs(span) > 1e-9:
                percentage = (distance - d0) / span * 100.0
                cv2.putText(
                    frame,
                    f'Calculated position: {percentage:.2f} %',
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 0),
                    2
                )

    def draw_end_effector_measurement(self, frame, pose):
        t = pose['t_camera_from_ee']
        R = pose['R_camera_from_ee']
        rpy_deg = np.degrees(rotation_matrix_to_rpy(R))

        cv2.putText(
            frame,
            (
                f'ID 3 EE: X={t[0]:+.4f} Y={t[1]:+.4f} '
                f'Z={t[2]:+.4f} m'
            ),
            (20, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 200, 0),
            2
        )

        cv2.putText(
            frame,
            (
                f'EE RPY: R={rpy_deg[0]:+.1f} '
                f'P={rpy_deg[1]:+.1f} Y={rpy_deg[2]:+.1f} deg'
            ),
            (20, 178),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 200, 0),
            1
        )

    def draw_calibration_status(self, frame):
        y = frame.shape[0] - 92

        if self.zero_calibration is None:
            zero_text = '0%: NOT SET'
            zero_color = (0, 0, 255)
        else:
            zero_text = (
                f"0%: {self.zero_calibration['center_distance_m']:.6f} m"
            )
            zero_color = (0, 255, 0)

        if self.full_calibration is None:
            full_text = '100%: NOT SET'
            full_color = (0, 0, 255)
        else:
            full_text = (
                f"100%: {self.full_calibration['center_distance_m']:.6f} m"
            )
            full_color = (0, 255, 0)

        if self.end_effector_calibration is None:
            ee_text = 'ID 3 EE: NOT SET'
            ee_color = (0, 0, 255)
        else:
            ee_text = 'ID 3 EE: SET'
            ee_color = (0, 255, 0)

        cv2.putText(
            frame,
            zero_text,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            zero_color,
            2
        )
        cv2.putText(
            frame,
            full_text,
            (20, y + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            full_color,
            2
        )
        cv2.putText(
            frame,
            ee_text,
            (20, y + 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            ee_color,
            2
        )

    def show_frame(self, frame):
        cv2.imshow(
            'ArUco Calibration - press 0 / 1 / 3',
            frame
        )
        key = cv2.waitKey(1) & 0xFF

        if key == ord('0'):
            self.capture_distance_calibration(0)
        elif key == ord('1'):
            self.capture_distance_calibration(100)
        elif key == ord('3'):
            self.capture_end_effector_calibration()
        elif key in (ord('q'), 27):
            self.get_logger().info(
                'Quit requested from OpenCV window.'
            )
            rclpy.shutdown()

    # --------------------------------------------------------
    # Keyboard handling
    # --------------------------------------------------------
    def keyboard_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        self.terminal_fd = fd
        self.old_terminal_settings = old_settings

        try:
            tty.setcbreak(fd)

            while rclpy.ok():
                ready, _, _ = select.select(
                    [sys.stdin],
                    [],
                    [],
                    0.1
                )
                if not ready:
                    continue

                key = sys.stdin.read(1)

                if key == '0':
                    self.capture_distance_calibration(0)
                elif key == '1':
                    self.capture_distance_calibration(100)
                elif key == '3':
                    self.capture_end_effector_calibration()
                elif key.lower() == 'q':
                    self.get_logger().info(
                        'Quit requested from terminal.'
                    )
                    rclpy.shutdown()
                    break

        except Exception as e:
            self.get_logger().warning(
                f'Terminal keyboard listener stopped: {e}'
            )

        finally:
            try:
                termios.tcsetattr(
                    fd,
                    termios.TCSADRAIN,
                    old_settings
                )
            except Exception:
                pass

    # --------------------------------------------------------
    # 0% / 100% capture
    # --------------------------------------------------------
    def capture_distance_calibration(self, percentage):
        with self.lock:
            if (
                self.latest_positions is None or
                self.latest_distance is None
            ):
                self.get_logger().warning(
                    f'Cannot save {percentage}%: IDs '
                    f'{MARKER_ID_1} and {MARKER_ID_2} must both '
                    'be visible right now.'
                )
                return

            p1 = self.latest_positions[MARKER_ID_1].copy()
            p2 = self.latest_positions[MARKER_ID_2].copy()
            distance = float(self.latest_distance)

        record = {
            'timestamp': datetime.now().isoformat(
                timespec='seconds'
            ),
            'marker_1': {
                'id': MARKER_ID_1,
                'position_camera_m': {
                    'x': float(p1[0]),
                    'y': float(p1[1]),
                    'z': float(p1[2]),
                },
            },
            'marker_2': {
                'id': MARKER_ID_2,
                'position_camera_m': {
                    'x': float(p2[0]),
                    'y': float(p2[1]),
                    'z': float(p2[2]),
                },
            },
            'center_distance_m': distance,
        }

        if percentage == 0:
            self.zero_calibration = record
        elif percentage == 100:
            self.full_calibration = record
        else:
            return

        self.save_calibration_file()

        self.get_logger().info('')
        self.get_logger().info(
            '===================================================='
        )
        self.get_logger().info(
            f'[{percentage}% CALIBRATION SAVED]'
        )
        self.get_logger().info(
            f'ArUco {MARKER_ID_1}: '
            f'X={p1[0]:+.6f}, Y={p1[1]:+.6f}, Z={p1[2]:+.6f} m'
        )
        self.get_logger().info(
            f'ArUco {MARKER_ID_2}: '
            f'X={p2[0]:+.6f}, Y={p2[1]:+.6f}, Z={p2[2]:+.6f} m'
        )
        self.get_logger().info(
            f'Center distance = {distance:.6f} m'
        )
        self.get_logger().info(
            f'Saved to: {CALIBRATION_FILE}'
        )

        if (
            self.zero_calibration is not None and
            self.full_calibration is not None
        ):
            d0 = self.zero_calibration['center_distance_m']
            d100 = self.full_calibration['center_distance_m']

            self.get_logger().info(
                '----------------------------------------------------'
            )
            self.get_logger().info(
                f'0% distance   = {d0:.6f} m'
            )
            self.get_logger().info(
                f'100% distance = {d100:.6f} m'
            )
            self.get_logger().info(
                f'Span          = {d100 - d0:+.6f} m'
            )

            if abs(d100 - d0) <= 1e-9:
                self.get_logger().warning(
                    '0% and 100% distances are identical. '
                    'Please recalibrate.'
                )

        self.get_logger().info(
            '===================================================='
        )

    # --------------------------------------------------------
    # ID 3 / end-effector capture
    # --------------------------------------------------------
    def capture_end_effector_calibration(self):
        with self.lock:
            if self.latest_end_effector_pose is None:
                self.get_logger().warning(
                    f'Cannot save ID {END_EFFECTOR_MARKER_ID}: '
                    'the marker must be visible right now.'
                )
                return

            pose = {
                key: value.copy()
                if isinstance(value, np.ndarray)
                else value
                for key, value
                in self.latest_end_effector_pose.items()
            }

        t_raw = pose['raw_tvec']
        t_ee = pose['t_camera_from_ee']

        R_raw = pose['R_camera_from_raw']
        R_ee = pose['R_camera_from_ee']

        q_raw = rotation_matrix_to_quaternion_xyzw(R_raw)
        q_ee = rotation_matrix_to_quaternion_xyzw(R_ee)

        rpy_raw = rotation_matrix_to_rpy(R_raw)
        rpy_ee = rotation_matrix_to_rpy(R_ee)

        self.end_effector_calibration = {
            'timestamp': datetime.now().isoformat(
                timespec='seconds'
            ),
            'camera_frame_id': self.camera_frame_id,
            'marker_id': END_EFFECTOR_MARKER_ID,
            'marker_length_m': float(
                END_EFFECTOR_MARKER_LENGTH
            ),

            'origin_definition': (
                'ArUco ID 3 center is the YAM-compatible EE/TCP origin.'
            ),

            'raw_aruco_pose': {
                'frame_axes': {
                    'x': 'marker right',
                    'y': 'marker up',
                    'z': 'out of paper',
                },
                'translation_camera_m': {
                    'x': float(t_raw[0]),
                    'y': float(t_raw[1]),
                    'z': float(t_raw[2]),
                },
                'rotation_matrix_camera_from_aruco3': (
                    matrix_to_list(R_raw)
                ),
                'quaternion_camera_from_aruco3_xyzw': (
                    vector_to_list(q_raw)
                ),
                'rpy_camera_from_aruco3_rad': (
                    vector_to_list(rpy_raw)
                ),
                'rpy_camera_from_aruco3_deg': (
                    vector_to_list(np.degrees(rpy_raw))
                ),
                'T_camera_from_aruco3_raw': (
                    matrix_to_list(
                        pose['T_camera_from_raw']
                    )
                ),
                'T_aruco3_raw_from_camera': (
                    matrix_to_list(
                        pose['T_raw_from_camera']
                    )
                ),
            },

            'yam_ee_pose': {
                'frame_axes': {
                    'x': 'YAM EE +X; at zero sticker offset, marker image left',
                    'y': 'YAM EE +Y; at zero sticker offset, marker image up',
                    'z': 'into marker paper; gripper/tool direction',
                },

                'includes_umi_to_yam_local_z_180_deg': True,

                'aruco_z_rotation_offset_deg': float(
                    ARUCO_Z_ROT_OFFSET_DEG
                ),

                'translation_camera_m': {
                    'x': float(t_ee[0]),
                    'y': float(t_ee[1]),
                    'z': float(t_ee[2]),
                },

                'rotation_matrix_camera_from_yam_ee': (
                    matrix_to_list(R_ee)
                ),

                'quaternion_camera_from_yam_ee_xyzw': (
                    vector_to_list(q_ee)
                ),

                'rpy_camera_from_yam_ee_rad': (
                    vector_to_list(rpy_ee)
                ),

                'rpy_camera_from_yam_ee_deg': (
                    vector_to_list(np.degrees(rpy_ee))
                ),

                'T_camera_from_yam_ee': (
                    matrix_to_list(
                        pose['T_camera_from_ee']
                    )
                ),

                'T_yam_ee_from_camera': (
                    matrix_to_list(
                        pose['T_ee_from_camera']
                    )
                ),

                'T_camera_from_gripper_tcp': (
                    matrix_to_list(
                        pose['T_camera_from_ee']
                    )
                ),
                'T_gripper_tcp_from_camera': (
                    matrix_to_list(
                        pose['T_ee_from_camera']
                    )
                ),
            },
        }

        self.save_calibration_file()

        self.get_logger().info('')
        self.get_logger().info(
            '===================================================='
        )
        self.get_logger().info(
            '[ID 3 / YAM EE CALIBRATION SAVED]'
        )
        self.get_logger().info(
            f'Camera frame: {self.camera_frame_id}'
        )
        self.get_logger().info(
            f'EE translation in camera = '
            f'[{t_ee[0]:+.6f}, {t_ee[1]:+.6f}, {t_ee[2]:+.6f}] m'
        )
        self.get_logger().info(
            'EE convention: +Z into paper; '
            f'Z-offset={ARUCO_Z_ROT_OFFSET_DEG:.1f} deg'
        )
        self.get_logger().info(
            'T_camera_from_yam_ee =\n'
            + np.array2string(
                pose['T_camera_from_ee'],
                precision=6,
                suppress_small=True
            )
        )
        self.get_logger().info(
            'T_yam_ee_from_camera =\n'
            + np.array2string(
                pose['T_ee_from_camera'],
                precision=6,
                suppress_small=True
            )
        )
        self.get_logger().info(
            f'Saved to: {CALIBRATION_FILE}'
        )
        self.get_logger().info(
            '===================================================='
        )

    # --------------------------------------------------------
    # YAML save / load
    # --------------------------------------------------------
    def save_calibration_file(self):
        data = {
            'format_version': 4,

            'camera': {
                'image_topic': IMAGE_TOPIC,
                'camera_info_topic': CAMERA_INFO_TOPIC,
                'frame_id': self.camera_frame_id,
            },

            'distance_calibration': {
                'marker_ids': [
                    MARKER_ID_1,
                    MARKER_ID_2
                ],
                'marker_length_m': float(
                    DISTANCE_MARKER_LENGTH
                ),
                'zero_percent': self.zero_calibration,
                'hundred_percent': self.full_calibration,
            },

            'end_effector_calibration': {
                'marker_id': END_EFFECTOR_MARKER_ID,
                'marker_length_m': float(
                    END_EFFECTOR_MARKER_LENGTH
                ),

                'yam_frame_definition': {
                    'base_frame': 'UMI handle bottom-center frame',
                    'ee_frame': 'YAM-compatible EE/TCP frame',
                    'ee_origin': 'ArUco ID 3 center',
                    'ee_x_axis': '+Base Y',
                    'ee_y_axis': '+Base Z',
                    'ee_z_axis': '+Base X (gripper/tool direction)',
                    'includes_umi_to_yam_local_z_180_deg': True,
                    'R_base_from_yam_ee': (
                        matrix_to_list(R_BASE_FROM_EE)
                    ),
                },

                'cad_reference': {
                    'min_dist_mm': (
                        vector_to_list(CAD_MIN_DIST_MM)
                    ),
                    'max_dist_mm': (
                        vector_to_list(CAD_MAX_DIST_MM)
                    ),
                    'ee_midpoint_mm': (
                        vector_to_list(CAD_EE_POSITION_MM)
                    ),
                    'ee_midpoint_m': (
                        vector_to_list(CAD_EE_POSITION_M)
                    ),
                    'T_base_from_yam_ee': (
                        matrix_to_list(T_BASE_FROM_EE)
                    ),
                    'T_yam_ee_from_base': (
                        matrix_to_list(T_EE_FROM_BASE)
                    ),
                },

                'frame_assumption': {
                    'description': (
                        'ID 3 center is the YAM-compatible EE/TCP origin. '
                        'EE +Z points into the marker paper. '
                        'The local-Z 180-degree UMI-to-YAM correction is included, '
                        'with an optional fixed sticker rotation around +Z.'
                    ),

                    'raw_aruco_axes': {
                        'x': 'marker right',
                        'y': 'marker up',
                        'z': 'out of paper',
                    },

                    'yam_ee_axes_at_zero_z_offset': {
                        'x': 'marker left',
                        'y': 'marker up',
                        'z': 'into paper',
                    },

                    'aruco_z_rotation_offset_deg': float(
                        ARUCO_Z_ROT_OFFSET_DEG
                    ),

                    'fixed_rotation_raw_aruco_from_yam_ee': (
                        matrix_to_list(
                            R_RAW_ARUCO_FROM_EE
                        )
                    ),

                    'T_aruco3_raw_from_yam_ee': (
                        matrix_to_list(
                            T_RAW_ARUCO_FROM_EE
                        )
                    ),

                    'T_yam_ee_from_aruco3_raw': (
                        matrix_to_list(
                            T_EE_FROM_RAW_ARUCO
                        )
                    ),
                },

                'measurement': self.end_effector_calibration,
            },
        }

        try:
            CALIBRATION_FILE.parent.mkdir(
                parents=True,
                exist_ok=True
            )

            tmp_file = CALIBRATION_FILE.with_suffix(
                CALIBRATION_FILE.suffix + '.tmp'
            )

            with tmp_file.open(
                'w',
                encoding='utf-8'
            ) as f:
                yaml.safe_dump(
                    data,
                    f,
                    sort_keys=False,
                    allow_unicode=True,
                    default_flow_style=False
                )

            tmp_file.replace(CALIBRATION_FILE)

        except Exception as e:
            self.get_logger().error(
                f'Failed to save YAML calibration file: {e}'
            )

    def load_existing_calibration(self):
        if not CALIBRATION_FILE.exists():
            return

        try:
            with CALIBRATION_FILE.open(
                'r',
                encoding='utf-8'
            ) as f:
                data = yaml.safe_load(f) or {}

            distance_data = data.get(
                'distance_calibration',
                {}
            )
            self.zero_calibration = distance_data.get(
                'zero_percent'
            )
            self.full_calibration = distance_data.get(
                'hundred_percent'
            )

            ee_data = data.get(
                'end_effector_calibration',
                {}
            )
            existing_measurement = ee_data.get('measurement')
            if (
                existing_measurement is not None
                and 'yam_ee_pose' not in existing_measurement
            ):
                self.end_effector_calibration = None
                self.get_logger().warning(
                    'Legacy UMI EE calibration detected. Press [3] to recalibrate '
                    'ID 3 using the new YAM EE convention before processing bags.'
                )
            else:
                self.end_effector_calibration = existing_measurement

            self.get_logger().info(
                f'Loaded existing YAML calibration: '
                f'{CALIBRATION_FILE}'
            )

        except Exception as e:
            self.get_logger().warning(
                f'Existing YAML calibration could not be loaded: {e}'
            )

    # --------------------------------------------------------
    # Shutdown
    # --------------------------------------------------------
    def destroy_node(self):
        if (
            self.terminal_fd is not None and
            self.old_terminal_settings is not None
        ):
            try:
                termios.tcsetattr(
                    self.terminal_fd,
                    termios.TCSADRAIN,
                    self.old_terminal_settings
                )
            except Exception:
                pass

        cv2.destroyAllWindows()
        self.get_logger().info(
            'ArUco calibration node stopped.'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()
