# RGB-D Bag Processing

This package provides `process_rgbd_bags`, an offline exporter for the
`data0812_*` rosbag2 recordings.

## Inputs

- RGB-D bags recorded with the topics in `locating.txt`.
- One ORB-SLAM3 TUM trajectory per bag, usually `CameraTrajectory.txt`.
- Optional gripper distance calibration from `aruco_calibration.yaml`.

For per-bag trajectories, put files in one of these layouts:

```text
trajectories/data0812_01/CameraTrajectory.txt
trajectories/data0812_02/CameraTrajectory.txt
trajectories/data0812_03/CameraTrajectory.txt
```

or:

```text
trajectories/data0812_01_CameraTrajectory.txt
trajectories/data0812_02_CameraTrajectory.txt
trajectories/data0812_03_CameraTrajectory.txt
```

## Full Pipeline

Use the wrapper script when trajectories still need to be generated. It runs
ORB-SLAM3 once per bag, saves each trajectory under `trajectories/<bag_name>/`,
then calls `process_rgbd_bags`.

Choose `--sensor rgbd` for image-only SLAM or `--sensor rgbd-inertial` (default)
for IMU SLAM. This selects the executable and its 60 Hz locating YAML; the
inertial YAML additionally expects 200 Hz IMU. An explicit `--settings` or
`SETTINGS` environment variable overrides that default. Use `--atlas-path` to
select the corresponding sensor's Atlas independently of the working directory.

```bash
cd ~/umi_ORB_SLAM3
ros2_ws/scripts/run_orbslam_and_process_bags.sh \
  --sensor rgbd \
  --atlas-path "$PWD/maps/lab_b36_rgbd/atlas.osa" \
  --domain-id 42 \
  --bag-rate 0.4 \
  data0927_{01..20}
```

RGB-D mode skips IMU topic validation, IMU publisher conflict checks, IMU
subscription readiness and the startup IMU check. Image validation and trajectory
checks remain active. Both modes use the same exporter, which still requires
color images, camera information and extrinsics. `--raw-depth`, `--skip-export`
and `--skip-orbslam` work with either mode. The two startup-IMU options have no
effect in RGB-D mode.

The wrapper reads `storage_identifier` from each bag's `metadata.yaml` and
passes it explicitly to `ros2 bag play`, avoiding MCAP auto-detection failures.
It starts playback paused, waits until ORB-SLAM3 has RGB, depth (and, in inertial mode, IMU)
subscriptions, then resumes through `/rosbag2_player/resume`. This preserves
the recorded startup interval used by Atlas relocalization and static-IMU
initialization.

In inertial mode, before ORB-SLAM3, the wrapper inspects the first six seconds of each bag using
the same static-IMU thresholds as the current tracking code. The report is
saved to `trajectories/<bag_name>/startup_check.txt`. A failed static check is
a warning by default; use `--skip-high-risk-startup` to skip those bags. Bags
with invalid storage, missing required topics, or no valid camera trajectory
are always skipped, and processing continues with the next bag.

```bash
cd ~/umi_ORB_SLAM3/ros2_ws
colcon build --packages-select umi_dataset_tools

cd ~/umi_ORB_SLAM3
chmod +x ros2_ws/scripts/run_orbslam_and_process_bags.sh

ros2_ws/scripts/run_orbslam_and_process_bags.sh \
  --bag-rate 0.4 \
  --skip-high-risk-startup \
  data0908_{01..21} data0909_{01..42}
```

Bash brace expansion uses two dots (`{01..20}`), not three dots. By default,
bags whose startup IMU check is high risk are still attempted. To skip them:

```bash
ros2_ws/scripts/run_orbslam_and_process_bags.sh \
  --bag-rate 0.4 \
  --skip-high-risk-startup \
  --allow-live-publishers \
  data0814_{01..20}
```

For future recordings, keep the camera and robot stationary while viewing the
mapped scene for at least 3--5 seconds after recording starts. Paused playback
prevents the beginning of a bag from being lost while ORB-SLAM3 loads, but it
cannot recreate a static initialization interval that was not recorded.

The wrapper uses the locating command from `locating.txt`:

```text
camera/rgb   -> /camera/camera/infra1/image_rect_raw
camera/depth -> /camera/camera/depth/image_rect_raw
```

## Export Only

Use `process_rgbd_bags` directly when per-bag ORB-SLAM3 trajectories already
exist.

```bash
cd ~/umi_ORB_SLAM3/ros2_ws
colcon build --packages-select umi_dataset_tools
source install/setup.bash

ros2 run umi_dataset_tools process_rgbd_bags \
  ../data/bags/data0908_{01..21} ../data/bags/data0909_{01..42} \
  --trajectory ../data/trajectories \
  --calibration src/umi_dataset_tools/aruco_calibration.yaml \
  --output ../data/processed
```

`--trajectory` may also be a single TUM file. Use that only when all bags should
share exactly the same trajectory timestamps.

PNG compressed depth is the default:

```bash
ros2 run umi_dataset_tools process_rgbd_bags \
  ../data/bags/data0914_01 \
  --trajectory ../data/trajectories \
  --calibration src/umi_dataset_tools/aruco_calibration.yaml \
  --output ../data/processed
```

The default topic is `/camera/camera/depth/image_rect_raw/compressedDepth`.
Use `--raw-depth` for old raw-depth bags, or `--depth-topic` for a custom topic.
PNG compressedDepth is supported; RVL is rejected with an explicit error.

## Outputs

Each bag writes to `data/processed/<bag_name>/` by default:

- `trajectory_full.csv`: complete timestamped ORB-SLAM3 pose table before clipping.
- `trajectory_full_tum.txt`: complete TUM-format trajectory copy before clipping.
- `trajectory.csv`: radius-clipped ORB-SLAM3 pose table.
- `trajectory_tum.txt`: radius-clipped TUM-format trajectory copy.
- `trajectory_base_ee.csv`: timestamped `T_BASE_EE(t)` converted from
  `T_WORLD_TRACKING(t)` using the first-frame pose and hand-eye calibration.
- `pointcloud_frames/<depth_timestamp_ns>.ply`: one color-camera-frame point cloud per accepted depth frame.
- `pointcloud_frames.csv`: depth/trajectory/color timestamp association for every per-frame point cloud.
- `color/<timestamp_ns>.png`: color frames whose timestamp also has infra1, depth, and trajectory data.
- `gripper.csv`: ArUco marker center distance in meters plus opening percent.
- `missing_timestamps.csv`: only incomplete timestamps and their missing stream names.
- `missing_timestamps.png`: Matplotlib timeline of missing infra1, depth, color, trajectory, and complete frames.
- `camera_calibration.yaml`: color/depth intrinsics, distortion, and depth-to-infra1/depth-to-color extrinsics from the bag.
- `DATASET_INFO.md`: folder structure, trajectories, coordinate frames, point clouds, color images, synchronization rules, and dataset summary.

## Geometry Notes

Depth points are created in the depth camera optical frame. Each output point
cloud is transformed only into the color camera optical frame:

```text
p_COLOR = R_COLOR_DEPTH * p_DEPTH + t_COLOR_DEPTH
```

using `/camera/camera/extrinsics/depth_to_color` from the bag. The ORB-SLAM3
trajectory is not applied to PLY coordinates. Therefore, each PLY uses the moving
color-camera frame at its own timestamp; point clouds from different timestamps
must not be directly merged without an additional trajectory-based transform.

The trajectory is `world_T_infra1`. ORB-SLAM3 `SaveTrajectoryTUM` normalizes the
trajectory to the Atlas keyframe with the smallest ID. With the locating settings,
an existing Atlas is loaded, so the origin is the loaded map origin rather than the
first frame of the processed bag.

Transform names follow `T_A_B`: pose of target frame B expressed in reference
frame A. `trajectory.csv` remains the persistent Atlas pose
`T_WORLD_TRACKING(t)`. The converted trajectory uses:

```text
T_TRACKING_0_TRACKING(t) = T_TRACKING_0_WORLD * T_WORLD_TRACKING(t)
T_BASE_TRACKING_0 = T_BASE_EE_0 * T_EE_TRACKING
T_BASE_EE(t) = T_BASE_TRACKING_0 * T_TRACKING_0_TRACKING(t) * T_TRACKING_EE
```

`BASE` is the fixed frame at the center of the bottom of the UMI handle.
`T_TRACKING_0_WORLD` and `T_EE_TRACKING` are the corresponding transforms with
the source and target frame order swapped.

The hand-eye YAML is calibrated in `camera_color_optical_frame`, so the exporter
uses the bag's depth-to-color and depth-to-infra1 extrinsics to derive
`T_TRACKING_EE`. This conversion assumes the first trajectory sample corresponds
to the calibrated initial pose `T_BASE_EE_0`.

Calibration format version 4 defines `EE` directly as the YAM-compatible EE/TCP
frame. Its origin is ArUco ID 3 center, and its axes are `+X=+BaseY`,
`+Y=+BaseZ`, `+Z=+BaseX`. The former UMI convention had opposite X/Y, so the
local-Z 180-degree UMI-to-YAM correction is already included in
`T_base_from_yam_ee` and `T_camera_from_yam_ee`. Isaac Sim and real-robot replay
code must not apply that fixed rotation again.

Version 3 calibration files remain readable as `legacy_umi_ee`, but should be
recalibrated by running `aruco_calibration.py` and pressing `3` before producing
new datasets.

For color, the script does not index the color image with depth pixels directly.
It projects each depth 3D point through `/camera/camera/extrinsics/depth_to_color`
and the color camera intrinsics, then samples the color image at that projected
pixel.

Useful controls:

- `--pointcloud-stride 10`: use every 10th depth frame for the point cloud.
- `--pixel-stride 4`: use every 4th depth pixel in both x and y.
- `--trajectory-clip-head-radius 0.02`: start at the first pose whose translation
  distance from the first pose exceeds 2 cm.
- `--trajectory-clip-tail-radius 0.10`: search backward for the last pose outside
  10 cm, then include the following pose that returned inside the radius. Color,
  point clouds, gripper data, and converted trajectories use the same interval.
- `--trajectory-clip-radius VALUE` remains as a deprecated compatibility option
  that assigns the same value to both radii.

Infra1, depth, and color are accepted only when their ROS header timestamps are
exactly equal in nanoseconds. ORB-SLAM3 TUM timestamps have limited decimal
precision, so trajectory matching uses a fixed internal tolerance of 1 ms. Only
timestamps containing all four sources are exported. This policy is intentionally
not adjustable from command-line arguments.

## Point Cloud and EE Viewer

After building and sourcing the package, inspect each point-cloud frame with its
current EE and color-camera poses:

```bash
ros2 run umi_dataset_tools visualize_pointcloud_ee \
  ../data/processed/data0814_01
```

The viewer leaves each stored PLY unchanged. It uses `T_BASE_EE(t)` and the
calibrated `T_COLOR_EE` only for display, transforming the local color-frame cloud
into `BASE`. Red, green, and blue axes represent each frame's +X, +Y, and +Z.
Legacy `orbslam_world_map` PLY files are also supported: the viewer uses the saved
`T_BASE_TRACKING_0` and `T_TRACKING_0_WORLD` to display them in `BASE` without
rewriting the files.

- `Left` / `A`: previous frame.
- `Right` / `D` / `Space`: next frame.
- `Home` / `End`: first or last frame.
- `Q` / `Esc`: close the viewer.
- `--point-size 1.0`: point marker size.
- `--axis-length 0.05`: coordinate-axis length in meters.
- `--max-distance 1.0`: only display points within this distance of the current
  color-camera origin; the default is 1.0 meter and stored PLY files are unchanged.

## Gripper and RGB Viewer

Build and source the package, then open one processed dataset:

```bash
cd ~/umi_ORB_SLAM3/ros2_ws
colcon build --packages-select umi_dataset_tools --symlink-install
source install/setup.bash

ros2 run umi_dataset_tools visualize_gripper_rgb \
  ../data/processed/data0812_01
```

Use `--value opening_percent` to plot calibrated percentage instead of meters.
The plot shows both the raw measurement and a zero-phase-style forward/backward
low-pass result. Set its cutoff frequency with `--lowpass-cutoff 2.0` (Hz).
The viewer supports the slider, waveform clicking, Play/Pause buttons, Left/Right
frame stepping, Up/Down playback speed, Space to play/pause, and `q` to close.
