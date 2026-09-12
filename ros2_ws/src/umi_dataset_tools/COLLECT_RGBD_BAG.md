# UMI RGB-D Bag Collector

## Build

```bash
cd ~/umi_ORB_SLAM3/ros2_ws
colcon build --packages-select umi_dataset_tools --symlink-install
source install/setup.bash
```

Start the RealSense camera and Xsens driver before running the collector.

## Automatic Naming

The command can run from any directory after sourcing the workspace:

```bash
ros2 run umi_dataset_tools collect_rgbd_bag
```

The default prefix is the current date in `dataMMDD` format. Existing directories
are scanned, so `data0902_01` and `data0902_02` cause the next bag to be named
`data0902_03`.

The default output is `~/umi_ORB_SLAM3/data/bags`. Set `UMI_DATA_ROOT` to put
all dataset data on another disk.

Specify a prefix or an exact name when needed:

```bash
ros2 run umi_dataset_tools collect_rgbd_bag --prefix data0902
ros2 run umi_dataset_tools collect_rgbd_bag --name calibration_test
```

## Recording Sequence

The tool performs these steps:

1. Verifies all required topics exist.
2. Measures image and IMU rates for three seconds.
3. Checks exact infra1/depth/color header timestamp synchronization.
4. Confirms camera information and depth extrinsics were received.
5. Starts MCAP recording with the `fastwrite` preset.
6. Displays a five-second stationary timer for ORB-SLAM3/IMU initialization.
7. Displays `3`, `2`, `1`, `GO` before motion should begin.
8. On `Ctrl+C`, safely stops rosbag2 and prints metadata message counts.

`/tf` and `/tf_static` are recorded but produce warnings rather than blocking the
recording when no message arrives during the health window.

Useful options:

```bash
ros2 run umi_dataset_tools collect_rgbd_bag \
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
