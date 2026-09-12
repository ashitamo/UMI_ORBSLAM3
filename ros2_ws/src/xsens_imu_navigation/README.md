# xsens_imu_navigation

ROS 2 Humble diagnostic inertial odometry using:

- `/filter/free_acceleration` (`geometry_msgs/msg/Vector3Stamped`)
- `/filter/quaternion` (`geometry_msgs/msg/QuaternionStamped`)
- `/imu/angular_velocity` (`geometry_msgs/msg/Vector3Stamped`)

The estimator performs initial stationary bias estimation, low-pass filtering,
stationary detection, zero-velocity updates (ZUPT), online bias adaptation and
trapezoidal integration.

## Important limitation

This is short-horizon dead reckoning, not an absolute navigation system. With
only IMU/AHRS data, position and velocity remain unobservable during arbitrary
motion and will drift. The program is primarily intended to diagnose whether
Xsens free acceleration produces a plausible short trajectory.

Free Acceleration is treated as already gravity-compensated and expressed in
the Xsens local Earth frame. Do not apply another gravity subtraction or rotate
it again with the quaternion.

## Verify topic types

```bash
ros2 topic type /filter/free_acceleration
ros2 topic type /filter/quaternion
ros2 topic type /imu/angular_velocity
```

Expected:

```text
geometry_msgs/msg/Vector3Stamped
geometry_msgs/msg/QuaternionStamped
geometry_msgs/msg/Vector3Stamped
```

## Install

Copy the package into a ROS 2 workspace:

```bash
cd ~/orbslam3_ros2_ws/src
cp -r /path/to/xsens_imu_navigation .

cd ~/orbslam3_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select xsens_imu_navigation
source install/setup.bash
```

## Run

Start the Xsens driver first. Keep the sensor completely still, then run:

```bash
ros2 launch xsens_imu_navigation imu_navigation.launch.py
```

The first 5 seconds are used to estimate residual Free Acceleration bias.
Movement during calibration restarts the calibration window.

## Outputs

```text
/imu_nav/odometry
/imu_nav/path
/imu_nav/stationary
/imu_nav/filtered_acceleration
/imu_nav/accel_bias
TF: imu_nav -> imu_dead_reckoning
```

## RViz

Set Fixed Frame to `imu_nav`, then add:

- Path: `/imu_nav/path`
- Odometry: `/imu_nav/odometry`
- TF

## Reset

Reset position and velocity while preserving the learned bias:

```bash
ros2 service call /imu_nav/reset std_srvs/srv/Trigger {}
```

Restart stationary calibration:

```bash
ros2 service call /imu_nav/recalibrate std_srvs/srv/Trigger {}
```

## First test

1. Keep still for calibration.
2. Move approximately 1 m in one direction.
3. Stop completely for at least 1 second so ZUPT can trigger.
4. Return to the starting point.
5. Stop again.

Inspect:

```bash
ros2 topic echo /imu_nav/stationary
ros2 topic echo /imu_nav/odometry
```

If the estimated endpoint is already far away after a short, slow motion, log
`/filter/free_acceleration`, `/filter/quaternion`, `/imu/angular_velocity` and
`/imu_nav/*` for analysis.

## Useful tuning

- Stationary state never becomes true: increase
  `stationary_accel_threshold` gradually from `0.08` to `0.12`, or
  `stationary_gyro_threshold` from `0.025` to `0.04`.
- Slow real motion is suppressed: lower `accel_deadband` from `0.025`.
- Path is noisy: lower `low_pass_cutoff_hz` from `8.0` to `4.0`.
- Timestamp warnings appear despite a stable 100 Hz stream: inspect message
  stamps first. As a temporary test only, set `use_fixed_dt: true`.
- Ground vehicle: set `planar_mode: true`.

Do not enable `velocity_leak_per_second` when evaluating sensor quality; it is a
non-physical visualization aid.
