# UMI RGB-D Bag Collector

## Build

```bash
cd ~/umi_ORB_SLAM3/ros2_ws
colcon build --packages-select umi_dataset_tools --symlink-install
source install/setup.bash
```

Start the RealSense camera and Xsens driver before running the collector.

## Sensor Mode

The default `--sensor rgbd-inertial` requires and records `/imu/data`.
For RGB-D without an IMU, start only RealSense and run:

```bash
ros2 run umi_dataset_tools collect_rgbd_bag --raw-depth --sensor rgbd --prefix data0927
```

RGB-D mode does not subscribe to, check, or record `/imu/data`, even if an IMU
publisher is running. Image rates, exact timestamps, camera information and
extrinsics are still checked. `--min-imu-hz` applies only to `rgbd-inertial`.
Both modes require color: enable `enable_color:=true` and verify
`/camera/camera/color/image_raw/compressed` publishes synchronized images.

The stationary timer remains five seconds in both modes to allow preparation;
it does not detect SLAM readiness. For RGB-D you may set `--stationary-seconds 0`
to keep only the three-second countdown. Bag naming, depth options and output
locations are identical for both modes. The batch replay script also supports
`--sensor rgbd` and `--atlas-path <path>` to generate trajectories and export data.

## Automatic Naming

The command can run from any directory after sourcing the workspace:

```bash
ros2 run umi_dataset_tools collect_rgbd_bag --raw-depth
```

The default prefix is the current date in `dataMMDD` format. Existing directories
are scanned, so `data0902_01` and `data0902_02` cause the next bag to be named
`data0902_03`.

The default output is `~/umi_ORB_SLAM3/data/bags`. Set `UMI_DATA_ROOT` to put
all dataset data on another disk.

Specify a prefix or an exact name when needed:

```bash
ros2 run umi_dataset_tools collect_rgbd_bag --raw-depth --prefix data0902
ros2 run umi_dataset_tools collect_rgbd_bag --raw-depth --name calibration_test
```

## Recording Sequence

The tool performs these steps:

1. Verifies all required topics exist.
2. Measures image rates and, in `rgbd-inertial` mode, IMU rates for three seconds.
3. Checks exact infra1/depth/color header timestamp synchronization.
4. Confirms camera information and depth extrinsics were received.
5. Starts MCAP recording with the `fastwrite` preset.
6. Displays a five-second stationary preparation timer (configurable).
7. Displays `3`, `2`, `1`, `GO` before motion should begin.
8. On `Ctrl+C`, safely stops rosbag2 and prints metadata message counts.

`/tf` and `/tf_static` are recorded but produce warnings rather than blocking the
recording when no message arrives during the health window.

Useful options:

Health checks use each stream's first-to-last reception interval:
`(message_count - 1) / (last_arrival - first_arrival)`. At least two messages
and the configured minimum rate are required. There is no subscription warmup,
freshness/span classification, or `--check-only` option. A short burst can
therefore appear healthy; this is a basic check, not a sustained-rate guarantee.
The default 20 Hz threshold does not certify 60 Hz operation.
Image QoS remains RELIABLE / KEEP_LAST(10). The stationary timer after recording
starts is separate and remains available.

Current recommendation: explicitly use `--raw-depth` for recording and batch
processing, preserving synchronization. See the
[compressedDepth issue](../../../docs/repo_state.md#compresseddepth-實機發布異常).
The tool's default remains compressed depth for compatibility.

To compare the stamp checker with the collector's compressed depth input:

```bash
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash
python3 ~/umi_ORB_SLAM3/ros2_ws/src/umi_dataset_tools/umi_dataset_tools/check_rgbd_stamps.py \
  --depth-topic /camera/camera/depth/image_rect_raw/compressedDepth
```

The checker buffers unmatched arrivals and counts each matched pair once.
`pending_rgb` and `pending_depth` are still waiting for their counterpart;
they are not immediately counted as unmatched. Raw depth remains its default.

```bash
ros2 run umi_dataset_tools collect_rgbd_bag \
  --raw-depth \
  --output-root ~/umi_ORB_SLAM3/data/bags \
  --health-seconds 3 \
  --stationary-seconds 5 \
  --countdown 3 \
  --max-cache-size 1000000000 \
  --min-image-hz 20 \
  --min-imu-hz 50
```

Use `--force` to continue after a failed health check and `--yes` to skip the
interactive confirmation. Existing bag directories are never overwritten.

## Compressed Depth

To reduce bag size, first check that the following topic exists:

```text
/camera/camera/depth/image_rect_raw/compressedDepth
```

```bash
ros2 topic list | grep compressedDepth
```

If the topic is already listed after starting RealSense, no additional package
installation is needed. Only when it is missing, install/check the transport:

```bash
sudo apt install ros-humble-image-transport ros-humble-compressed-depth-image-transport
```

Compressed depth remains the software default, but raw depth is currently recommended:

```bash
ros2 run umi_dataset_tools collect_rgbd_bag \
  --raw-depth \
  --prefix data0914
```

The collector validates the compressed topic frequency and exact header
timestamp synchronization. Only PNG `compressedDepth` is supported; RVL is not.
Use `--raw-depth` for the current recommended `sensor_msgs/Image` depth workflow.
