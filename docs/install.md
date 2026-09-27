# 系統安裝與重建

本文件說明如何在另一台電腦重建 D405 + Xsens + ORB-SLAM3
RGB-D-Inertial 系統。

## 1. 目標環境

```text
OS：Ubuntu 22.04
ROS 2：Humble
Camera：Intel RealSense D405
IMU：Xsens
Compiler：支援 C++17 的 GCC/G++
```

其他 Ubuntu 或 ROS 版本尚未驗證。

## 2. ROS 2 與基礎工具

先依 ROS 2 官方流程安裝 Humble Desktop，並確認：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

安裝基礎相依套件：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git git-lfs pkg-config \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-numpy python3-scipy python3-matplotlib python3-yaml \
  libeigen3-dev libopencv-dev libboost-all-dev libssl-dev \
  libgl1-mesa-dev libglew-dev
```

```bash
sudo rosdep init       # 已初始化過可略過
rosdep update
```

## 3. 感測器 Driver

### RealSense：使用 apt 安裝

目前使用 ROS 2 Humble 的 apt 套件，不需要在本 workspace clone 或編譯
RealSense ROS wrapper。官方 GitHub 已移至
[RealSenseAI/realsense-ros](https://github.com/realsenseai/realsense-ros)，
安裝選項請參考該 repository 的 README。

完成 ROS 2 apt 軟體來源設定後執行：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-camera-msgs \
  ros-humble-realsense2-description
source /opt/ros/humble/setup.bash
ros2 pkg prefix realsense2_camera
ros2 pkg xml realsense2_camera | grep '<version>'
```

apt 安裝的 package prefix 應為 `/opt/ros/humble`。目前開發機的 wrapper
版本為 `4.58.3`，不是要求 apt 永遠安裝此版本；可用以下指令記錄實際版本：

```bash
dpkg-query -W 'ros-humble-realsense2*'
```

避免 source 含有舊 RealSense 原始碼安裝的 overlay，否則可能蓋過 apt
版本；載入其他 workspace 後也應重新確認 package prefix。
目前建議使用 raw depth，並保留同步；compressedDepth 的降頻與 color
停止問題見 [repo_state.md](repo_state.md#compresseddepth-實機發布異常)。

### Xsens：使用 repository 內的 driver

Xsens 不會隨上述 RealSense apt 指令安裝。本 repository 已包含
`ros2_ws/src/Xsens_MTi_ROS_Driver_and_Ntrip_Client/`，其中有
`xsens_mti_ros2_driver` 與 `ntrip` packages，隨 `ros2_ws` 一起編譯。
不需另外 clone 或 source legacy driver workspace；避免同時加入同名 package。
原始 driver 說明與授權保留在該目錄。NTRIP 帳密為範例占位值，正式憑證
請另存本機，不要提交到 Git。

在後續 rosdep 步驟安裝相依套件；亦可明確安裝：

```bash
sudo apt install -y ros-humble-nmea-msgs ros-humble-mavros-msgs
```

完成 apt 安裝、本 workspace 編譯及載入環境後，必須能找到：

```text
realsense2_camera
realsense2_camera_msgs
xsens_mti_ros2_driver
```

不再使用外部 driver workspace。請從新的 terminal 依本文流程建置，
完成後只載入 `/opt/ros/humble/setup.bash` 與本專案的
`ros2_ws/install/setup.bash`。若 shell 啟動設定仍會自動載入舊 driver，
請移除該設定，並清除以前設定的 `LEGACY_ROS2_WS` 環境變數：

```bash
unset LEGACY_ROS2_WS
```

## 4. Clone Repository

```bash
git clone --recurse-submodules \
  https://github.com/ashitamo/UMI_ORBSLAM3.git \
  ~/umi_ORB_SLAM3
cd ~/umi_ORB_SLAM3
git submodule status
```

若忘記下載 submodules：

```bash
git submodule update --init --recursive
```

## 5. 檢查環境與資料目錄

```bash
cd ~/umi_ORB_SLAM3
./scripts/bootstrap.sh
```

這一步不編譯，只檢查 ROS/source 並建立 `data/bags`、
`data/trajectories`、`data/processed` 與 `runtime/orbslam`。

## 6. ROS Dependencies

```bash
source /opt/ros/humble/setup.bash
cd ~/umi_ORB_SLAM3
rosdep install \
  --from-paths ros2_ws/src \
  --ignore-src \
  --rosdistro humble \
  -r -y
```

若找不到 Xsens package，先確認 repository 內的 driver 目錄完整，並用
`colcon list` 確認可發現 `xsens_mti_ros2_driver`；不要移除 dependency。

## 7. 編譯

本系統不是只執行一次 `colcon build`。正確順序是：

```text
1. core_ws/src/Pangolin
2. core_ws/src/ORB_SLAM3
3. ros2_ws（ORB_SLAM3_ROS2、bringup、dataset tools）
```

其中 ORB-SLAM3 核心位於：

```text
~/umi_ORB_SLAM3/core_ws/src/ORB_SLAM3
```

`core_ws` 主要用來放置 C++ 核心相依程式，不是以 `colcon build` 直接編譯的
ROS 2 workspace。`scripts/bootstrap.sh` 會按照上述順序處理：

```bash
cd ~/umi_ORB_SLAM3
./scripts/bootstrap.sh --build
```

腳本會：

1. 使用 CMake 編譯 `core_ws/src/Pangolin`。
2. 檢查 `core_ws/src/ORB_SLAM3/lib/libORB_SLAM3.so`。
3. 若 ORB-SLAM3 library 不存在，執行 `core_ws/src/ORB_SLAM3/build.sh`。
4. 設定 `ORB_SLAM3_ROOT_DIR`，讓 ROS wrapper 找到 ORB-SLAM3 headers/library。
5. 以 `colcon build --symlink-install` 編譯 `ros2_ws`。

若要明確地手動重新編譯 ORB-SLAM3 核心：

```bash
cd ~/umi_ORB_SLAM3/core_ws/src/ORB_SLAM3
chmod +x build.sh
./build.sh
```

成功後應存在：

```text
~/umi_ORB_SLAM3/core_ws/src/ORB_SLAM3/lib/libORB_SLAM3.so
```

接著手動編譯 ROS 2 workspace：

```bash
source /opt/ros/humble/setup.bash
export ORB_SLAM3_ROOT_DIR=~/umi_ORB_SLAM3/core_ws/src/ORB_SLAM3

cd ~/umi_ORB_SLAM3/ros2_ws
colcon build \
  --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DPangolin_DIR=~/umi_ORB_SLAM3/core_ws/src/Pangolin/build
```

## 8. 載入環境

每個新 terminal：

```bash
source /opt/ros/humble/setup.bash
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash
```

不要 source 已刪除或垃圾桶裡的舊 `install/setup.bash`。若看到
`pythonpath_develop.sh not found`，請開新 terminal，只載入仍存在的 workspace；
必要時清除本專案 `build/install/log` 後重建。

## 9. 建置驗證

```bash
ros2 pkg prefix orbslam3
ros2 pkg prefix umi_orbslam3_bringup
ros2 pkg prefix umi_dataset_tools
ros2 pkg prefix realsense2_camera
ros2 pkg prefix xsens_mti_ros2_driver
ros2 pkg executables orbslam3
ros2 pkg executables umi_dataset_tools
```

ORB-SLAM3 預期至少包含 `orbslam3 rgbd` 和 `orbslam3 rgbd-inertial`。

```bash
grep CMAKE_BUILD_TYPE \
  ~/umi_ORB_SLAM3/ros2_ws/build/orbslam3/CMakeCache.txt
```

## 10. 感測器驗證

啟動 RealSense 與 Xsens 後：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /imu/data
ros2 topic echo /camera/camera/extrinsics/depth_to_infra1 --once
ros2 topic echo /camera/camera/extrinsics/depth_to_color --once
```

Mapping 基準約為 30 Hz image + 100 Hz IMU；Locating 基準約為
60 Hz image + 200 Hz IMU。頻率必須與 ORB-SLAM3 settings YAML 一致。

## 11. 最小功能測試

```bash
ros2 run umi_dataset_tools collect_rgbd_bag --help
cd ~/umi_ORB_SLAM3
ros2_ws/scripts/run_orbslam_and_process_bags.sh --help
```

正式流程請回到 [README](../README.md)。

## 12. 更新程式

```bash
cd ~/umi_ORB_SLAM3
git pull
git submodule update --init --recursive
./scripts/bootstrap.sh --build
```

若 submodule 有本機修改，先 commit 或 stash，避免直接切換 submodule commit。
