# D405 + Xsens + ORB-SLAM3 RGB-D-Inertial

> 文件版本：2026-08-04  
> 目前正式基準：**D405 Infra1 + Depth + Xsens IMU**
>
> 已驗證工作流程：
>
> - **建圖：30 Hz Infra/Depth + 100 Hz IMU**
> - **再次啟動並定位：載入既有 Atlas，60 Hz Infra/Depth + 200 Hz IMU**
>
> 標記說明：
>
> - ✅ 已由目前程式、執行輸出或測試確認
> - 🧪 已完成實驗，但結論仍受場景與測試條件限制
> - ⚠️ 可運作，但仍有工程限制或尚未完成定量驗證
> - 📌 尚待完成
> - 🔎 需重新檢查目前原始碼後才能確認

---

## 1. 專案概述

本專案在 Ubuntu 22.04 與 ROS 2 Humble 上整合：

- Intel RealSense D405
- Xsens IMU
- ORB-SLAM3
- 自訂 ROS 2 wrapper

目前穩定 SLAM 輸入為：

```text
影像：/camera/camera/infra1/image_rect_raw
深度：/camera/camera/depth/image_rect_raw
IMU： /imu/data
```

雖然 ORB-SLAM3 executable 名稱為 `rgbd-inertial`，目前 `camera/rgb` 實際 remap 到 D405 Infra1，因此本專案目前的正式基準應稱為：

> **Infra–Depth–Inertial ORB-SLAM3**

### 1.1 目前已完成的核心功能

✅ 可從空 Atlas 建立新地圖  
✅ 可完成 IMU 初始化  
✅ 可完成 VIBA 1、VIBA 2  
✅ 已恢復 IMU prediction 與 inertial optimization  
✅ 可儲存 Atlas 並在下一次執行載入  
✅ 載入 Atlas 後可成功進行 visual relocalization  
✅ 可在 relocalization 後蒐集本次 session 的靜態 IMU  
✅ 可建立新的 live KeyFrame anchor  
✅ 可建立第一個 live inertial prior，之後恢復一般 inertial tracking  
✅ 已驗證「30/100 建圖、60/200 再定位」可行  
✅ 已發布 `/orbslam3/pose`  
✅ 已發布 `/orbslam3/map_odometry`  
✅ 已發布 `map -> camera_infra1_optical_frame` TF  
✅ `Ctrl+C` 關閉時可自動儲存 CameraTrajectory 與 KeyFrameTrajectory  
✅ 可保存完整執行 log

### 1.2 目前尚未完成的部分

⚠️ 目前載入 Atlas 後仍會新增 KeyFrame 與 MapPoint，還不是嚴格的 localization-only  
🧪 彩色 RGB + aligned depth + IMU 已能啟動與追蹤，但未改善重複紋理的根本問題，因此不是目前正式基準  
📌 尚未重新執行最終 Kalibr  
📌 尚未完成 ground truth、ATE、RPE 等定量精度評估  
📌 尚未完成長時間部署與 Nav2 唯一定位源驗證

---

## 2. 軟硬體環境

### 2.1 已知環境

```text
OS：Ubuntu 22.04
ROS 2：Humble
Camera：Intel RealSense D405
IMU：Xsens
ORB-SLAM3 sensor mode：RGB-D-Inertial
穩定影像來源：Infra1 rectified
深度來源：Depth rectified
```

### 2.2 已驗證的兩組操作模式

#### 模式 A：建圖基準

```text
Infra1：30 Hz
Depth：30 Hz
IMU：100 Hz
解析度：848 × 480
ORB features：1500
用途：從空 Atlas 建圖、完成 VIBA、儲存 Atlas
```

#### 模式 B：載入 Atlas 後再次定位

```text
Infra1：60 Hz
Depth：60 Hz
IMU：200 Hz
解析度：848 × 480
ORB features：1500
用途：載入 30/100 建立的 Atlas，進行 relocalization 與後續追蹤
```

實測頻率：

```text
/imu/data：約 200 Hz
/camera/camera/infra1/image_rect_raw：約 59.9 Hz
```

在正常同步狀態下，60 Hz image interval 約為：

```text
image_dt ≈ 0.016696 s
```

200 Hz IMU 搭配 60 Hz影像時，每張影像區間通常可取得：

```text
imu_count = 3 或 4
```

這比 100 Hz IMU 搭配 60 Hz影像時穩定，後者常只有 `imu_count=1`，會被目前同步邏輯丟棄。

---

## 3. 資料夾與重要檔案

```text
/home/lab606/
├── slam_ws/
│   └── src/
│       └── ORB_SLAM3/
│           ├── Vocabulary/
│           │   └── ORBvoc.txt
│           ├── Examples/
│           │   └── RGB-D-Inertial/
│           │       ├── D405_rgbd_inertial_mapping.yaml
│           │       ├── D405_rgbd_inertial_locating_200_60.yaml
│           │       └── ...
│           ├── include/
│           ├── src/
│           │   ├── Tracking.cc
│           │   ├── LocalMapping.cc
│           │   ├── Optimizer.cc
│           │   └── ...
│           └── Thirdparty/
│
├── orbslam3_ros2_ws/
│   ├── src/
│   │   └── ORB_SLAM3_ROS2/
│   │       ├── src/
│   │       │   └── rgbd-inertial/
│   │       ├── include/
│   │       └── config/
│   │           └── mask_left.png
│   ├── build/
│   ├── install/
│   └── log/
│
├── orbslam3_logs/
│   └── rgbd_imu_YYYYMMDD_HHMMSS.log
│
├── CameraTrajectory.txt
└── KeyFrameTrajectory.txt
```

Atlas 名稱在 YAML 中通常不寫副檔名，ORB-SLAM3 儲存時會建立：

```text
<AtlasName>.osa
```

例如：

```text
D405_rgbd_imu_atlas_map.osa
```

### 3.1 封版時建議保存

```text
ORB_SLAM3 commit hash
ORB_SLAM3_ROS2 commit hash
建圖 YAML
定位 YAML
mask_left.png
RealSense 啟動參數
Xsens 輸出設定
曝光與 gain
穩定 Atlas
範例 log
CameraTrajectory.txt
KeyFrameTrajectory.txt
Kalibr 輸出
README.md
```

---

## 4. 編譯與環境載入

### 4.1 載入 ROS 2

```bash
source /opt/ros/humble/setup.bash
```

### 4.2 編譯 ROS 2 wrapper

```bash
cd ~/orbslam3_ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select orbslam3 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source ~/orbslam3_ros2_ws/install/setup.bash
```

### 4.3 確認執行檔

```bash
ros2 pkg executables orbslam3
```

預期至少包含：

```text
orbslam3 rgbd
orbslam3 rgbd-inertial
```

### 4.4 確認目前 build type

```bash
grep CMAKE_BUILD_TYPE \
  ~/orbslam3_ros2_ws/build/orbslam3/CMakeCache.txt
```

建議使用：

```text
CMAKE_BUILD_TYPE:STRING=Release
```

---

## 5. 啟動感測器與切換頻率

本專案已驗證兩組頻率配置：

| 用途 | D405 Infra1 / Depth | Xsens IMU | ORB-SLAM3 YAML |
|---|---:|---:|---|
| 建圖 | 30 Hz | 100 Hz | `D405_rgbd_inertial_mapping.yaml` |
| 載入 Atlas 再定位 | 60 Hz | 200 Hz | `D405_rgbd_inertial_locating_200_60.yaml` |

> **重要：** RealSense 與 Xsens 的真實輸出頻率，必須在感測器端或 driver 啟動參數中設定。YAML 內的 `Camera.fps` 與 `IMU.Frequency` 只是在告訴 ORB-SLAM3 應使用的頻率，不能單獨改變感測器輸出。

建議使用四個終端機，依序啟動：

```text
Terminal 1：RealSense D405
Terminal 2：Xsens IMU
Terminal 3：檢查 topic 與頻率
Terminal 4：ORB-SLAM3
```

### 5.1 啟動 D405：30 Hz 建圖模式

```bash
source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=false \
  enable_sync:=true \
  align_depth.enable:=false \
  depth_module.depth_profile:=848x480x30 \
  depth_module.infra_profile:=848x480x30
```

啟動後應存在：

```text
/camera/camera/infra1/image_rect_raw
/camera/camera/depth/image_rect_raw
/camera/camera/infra1/camera_info
/camera/camera/depth/camera_info
```

確認實際設定：

```bash
ros2 param get /camera/camera depth_module.depth_profile
ros2 param get /camera/camera depth_module.infra_profile
```

確認頻率：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

預期約為：

```text
Infra1：約 30 Hz
Depth：約 30 Hz
```

### 5.2 啟動 D405：60 Hz 再定位模式

```bash
source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=false \
  enable_sync:=true \
  align_depth.enable:=false \
  depth_module.depth_profile:=848x480x60 \
  depth_module.infra_profile:=848x480x60
```

確認頻率：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

目前實測：

```text
/camera/camera/infra1/image_rect_raw：約 59.9 Hz
```

若 driver 回報 profile 不支援或沒有套用，先查本機 launch 參數：

```bash
ros2 param describe /camera/camera depth_module.depth_profile
ros2 param describe /camera/camera depth_module.infra_profile
```

也可列出相機支援的 stream profile：

```bash
rs-enumerate-devices -s
```

### 5.3 修改 D405 頻率

D405 頻率主要由以下兩個 launch 參數控制：

```text
depth_module.depth_profile
depth_module.infra_profile
```

格式為：

```text
寬度 × 高度 × FPS
```

本專案目前使用：

```text
建圖：848x480x30
定位：848x480x60
```

修改解析度時，不能只改 launch 指令，還必須同步更新 ORB-SLAM3 YAML 中的：

```yaml
Camera.width: <新的寬度>
Camera.height: <新的高度>
Camera.fx: <對應解析度的 fx>
Camera.fy: <對應解析度的 fy>
Camera.cx: <對應解析度的 cx>
Camera.cy: <對應解析度的 cy>
```

相機內參應從對應 topic 重新讀取：

```bash
ros2 topic echo /camera/camera/infra1/camera_info --once
```

只修改 FPS、解析度仍保持 `848 × 480` 時，可維持同一組內參，但 ORB-SLAM3 YAML 中的 `Camera.fps` 必須同步改成實際 FPS。

### 5.4 設定 Xsens 為 100 Hz 或 200 Hz

Xsens 真實輸出頻率應在裝置的 Output Configuration 中設定，而不是只修改 ORB-SLAM3 YAML。

建議流程：

1. 先停止 Xsens ROS driver。
2. 開啟 MT Manager。
3. 連接 Xsens 裝置。
4. 進入 `Device Settings` → `Output Configuration`。
5. 將 ORB-SLAM3 所需的 `Acceleration` 與 `Rate of Turn` 設為相同頻率。
6. 建圖時設為 `100 Hz`；再次定位時設為 `200 Hz`。
7. 按下 `Apply`，讓設定寫入裝置。
8. 關閉 MT Manager，釋放序列埠。
9. 再啟動 ROS 2 Xsens driver。

ORB-SLAM3 使用 `/imu/data` 中的：

```text
angular_velocity
linear_acceleration
```

因此至少必須讓 Xsens 輸出：

```text
Rate of Turn
Acceleration
```

設定完成後，啟動 Xsens driver：

```bash
source /opt/ros/humble/setup.bash
source ~/orbslam3_ros2_ws/install/setup.bash

ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
```

若本機 package 或 launch file 名稱不同，可先查詢：

```bash
ros2 pkg executables xsens_mti_ros2_driver
ros2 pkg prefix --share xsens_mti_ros2_driver

find "$(ros2 pkg prefix --share xsens_mti_ros2_driver)" \
  -maxdepth 3 -type f \
  | grep -E 'launch|yaml'
```

確認 topic：

```bash
ros2 topic list | grep -E '^/imu|^/filter'
ros2 topic echo /imu/data --once
```

確認頻率：

```bash
ros2 topic hz /imu/data
```

預期：

```text
建圖模式：約 100 Hz
再次定位模式：約 200 Hz
```

目前 200 Hz 實測平均值約為：

```text
average rate: 200 Hz
```

`ros2 topic hz` 顯示的是 ROS message 到達間隔。Xsens USB/driver 可能出現 burst，因此還應確認 `header.stamp` 是否依設定頻率規律前進。ORB-SLAM3 預積分使用的是 message timestamp。

### 5.5 ORB-SLAM3 YAML 必須與感測器頻率一致

#### 建圖 YAML

檔案：

```text
D405_rgbd_inertial_mapping.yaml
```

設定：

```yaml
Camera.fps: 30.0
IMU.Frequency: 100.0
```

#### 再定位 YAML

檔案：

```text
D405_rgbd_inertial_locating_200_60.yaml
```

設定：

```yaml
Camera.fps: 60.0
IMU.Frequency: 200.0
```

修改 Xsens 或 D405 的實際頻率後，必須同步修改對應 YAML。若兩者不一致，可能造成：

```text
IMU 預積分時間錯誤
影像區間 IMU 數量判斷錯誤
初始化或 tracking 不穩定
不必要的 image drop
```

### 5.6 啟動後的完整檢查

```bash
ros2 topic list | grep -E 'infra1|depth|imu'

ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /imu/data

ros2 topic echo /camera/camera/infra1/camera_info --once
ros2 topic echo /imu/data --once
```

預期 topic：

```text
/camera/camera/infra1/image_rect_raw
/camera/camera/depth/image_rect_raw
/imu/data
```

---

## 6. 相機曝光設定

### 6.1 一般使用基準

```bash
ros2 param set /camera/camera \
  depth_module.enable_auto_exposure false

ros2 param set /camera/camera \
  depth_module.exposure 6000

ros2 param set /camera/camera \
  depth_module.gain 64
```

### 6.2 暗環境、中低速候選設定

```bash
ros2 param set /camera/camera \
  depth_module.enable_auto_exposure false

ros2 param set /camera/camera \
  depth_module.exposure 16000

ros2 param set /camera/camera \
  depth_module.gain 16
```

確認：

```bash
ros2 param get /camera/camera depth_module.enable_auto_exposure
ros2 param get /camera/camera depth_module.exposure
ros2 param get /camera/camera depth_module.gain
```

目前經驗：

- `6000 / 64`：一般使用較平衡。
- `16000 / 16`：暗環境與中低速可提高影像品質，但高速旋轉可能增加 motion blur。
- 60 Hz 時 frame period 約 16.67 ms，曝光不應長到逼近或超過 frame period。

---

# 7. 工作流程一：30 Hz / 100 Hz 建圖

## 7.1 建圖 YAML 必要設定

建圖 YAML 應使用：

```yaml
Camera.fps: 30.0
IMU.Frequency: 100.0
```

Atlas 設定：

```yaml
# 不載入舊地圖
# System.LoadAtlasFromFile: "..."

System.SaveAtlasToFile: "D405_rgbd_imu_atlas_map"
```

相機內參、解析度、DepthMapFactor、`IMU.T_b_c1`、ORB Vocabulary 必須使用目前已驗證版本。

## 7.2 啟動建圖

先依第 5.1 節啟動 `848 × 480 @ 30 Hz` 的 D405，並依第 5.4 節將 Xsens 設為 `100 Hz`、啟動 `/imu/data`。確認三個 topic 頻率正確後，再啟動 ORB-SLAM3。

```bash
source /opt/ros/humble/setup.bash
source ~/orbslam3_ros2_ws/install/setup.bash

ros2 run orbslam3 rgbd-inertial \
  /home/lab606/slam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/lab606/slam_ws/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_rgbd_inertial_mapping.yaml \
  true \
  --ros-args \
  -p imu_time_offset_sec:=0.0 \
  -p publish_pose:=true \
  -p publish_odometry:=true \
  -p publish_tf:=true \
  -p map_frame_id:=map \
  -p camera_frame_id:=camera_infra1_optical_frame \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw
```

其中：

```text
true
```

表示開啟 Viewer。效能測試時建議改為：

```text
false
```

## 7.3 初始化操作

從空 Atlas 建圖時：

1. 啟動後先確認 RGB-D 與 IMU 持續輸入。
2. 先短暫靜止。
3. 做適量 XYZ 平移。
4. 加入適量 roll、pitch、yaw。
5. 避免一開始就快速甩動。
6. 等待 `end VIBA 1`。
7. 等待 `end VIBA 2`。
8. 再開始正式建圖。

可能看到：

```text
Not enough motion for initializing
```

表示運動激勵不足。

可能看到：

```text
Fail to track local map!
```

表示視覺追蹤已進入高風險狀態，應降低速度或改善影像品質。

## 7.4 結束並保存

按：

```text
Ctrl+C
```

正常關閉流程應包含：

```text
[SHUTDOWN] Saving Atlas to D405_rgbd_imu_atlas_map.osa
Saving camera trajectory ...
Saving keyframe trajectory ...
ORB-SLAM3 shutdown completed.
```

輸出通常包括：

```text
D405_rgbd_imu_atlas_map.osa
~/orbslam3_ros2_ws/CameraTrajectory.txt
~/orbslam3_ros2_ws/KeyFrameTrajectory.txt
```

建議關閉後立即備份原始 Atlas：

```bash
mkdir -p ~/orbslam3_atlas_backup

cp D405_rgbd_imu_atlas_map.osa \
  ~/orbslam3_atlas_backup/D405_rgbd_imu_atlas_map_$(date +%Y%m%d_%H%M%S).osa
```

不要讓後續診斷測試覆寫唯一一份穩定地圖。

---

# 8. 工作流程二：載入 Atlas 後再次定位

## 8.1 已驗證架構

```text
建圖：30 Hz image + 100 Hz IMU
再次定位：60 Hz image + 200 Hz IMU
```

這套方式已成功完成：

```text
Atlas load
→ visual relocalization
→ 約 1.8～2 秒靜態 IMU 蒐集
→ 建立 live KeyFrame anchor
→ 建立 live inertial prior
→ 恢復一般 inertial tracking
```

## 8.2 定位 YAML 必要設定

```yaml
Camera.fps: 60.0
IMU.Frequency: 200.0

System.LoadAtlasFromFile: "D405_rgbd_imu_atlas_map"
```

目前若仍要保留本次 session 的地圖擴充，可另存：

```yaml
System.SaveAtlasToFile: "D405_rgbd_imu_atlas_locating"
```

若只想保護原始地圖，不要使用同名輸出覆寫：

```yaml
# System.SaveAtlasToFile: "D405_rgbd_imu_atlas_map"
```

## 8.3 啟動載入 Atlas 的定位

先依第 5.2 節啟動 `848 × 480 @ 60 Hz` 的 D405，並依第 5.4 節將 Xsens 設為 `200 Hz`、啟動 `/imu/data`。確認影像約 `60 Hz`、IMU 約 `200 Hz` 後，再執行下列指令。

```bash
source /opt/ros/humble/setup.bash
source ~/orbslam3_ros2_ws/install/setup.bash

ros2 run orbslam3 rgbd-inertial \
  /home/lab606/slam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/lab606/slam_ws/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_rgbd_inertial_locating_200_60.yaml \
  true \
  --ros-args \
  -p imu_time_offset_sec:=0.0 \
  -p publish_pose:=true \
  -p publish_odometry:=true \
  -p publish_tf:=true \
  -p map_frame_id:=map \
  -p camera_frame_id:=camera_infra1_optical_frame \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw
```

已驗證最小版指令也可使用：

```bash
ros2 run orbslam3 rgbd-inertial \
  /home/lab606/slam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/lab606/slam_ws/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_rgbd_inertial_locating_200_60.yaml \
  true \
  --ros-args \
  -p imu_time_offset_sec:=0.0 \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw
```

## 8.4 啟動時操作方式

載入 Atlas 後的前幾秒，不要立刻移動。

建議流程：

1. 將相機放在 Atlas 可辨識的區域。
2. 啟動程式。
3. 等待 visual relocalization 成功。
4. 保持設備靜止約 2 秒。
5. 等待靜態 IMU 判定完成。
6. 等待 live anchor 建立。
7. 等待 live bootstrap 成功。
8. 再開始移動與定位。

應看到類似：

```text
[ATLAS RELOCALIZATION] SUCCESS
[ATLAS STATIC IMU] READY
[ATLAS LIVE ANCHOR] CREATED
[ATLAS LIVE BOOTSTRAP] SUCCESS
```

已驗證範例：

```text
Atlas：111 KFs / 5788 MPs
Relocalized!!
靜態 IMU：362 samples / 1.805 s
Live anchor：KF=195
Live bootstrap：SUCCESS，inliers=540
```

## 8.5 正常定位期間的預期 log

正常 60/200：

```text
image_dt ≈ 0.016696 s
rgbd_diff = 0
imu_count = 3 或 4
```

常見 inliers：

```text
靜止或容易區域：600～860+
一般移動：400～700
困難重複紋理或高速區域：可能降到 100～300
```

## 8.6 目前「定位」仍會擴充地圖

目前測試中，載入 Atlas 後仍看到：

```text
[LOCAL MAPPING]
map-updated
inertial last keyframe
```

且結束時地圖由：

```text
111 KFs / 5788 MPs
```

增加到：

```text
146 KFs / 約 8951 MPs
```

因此目前狀態是：

> **載入 Atlas 後進行定位，並持續擴充地圖。**

不是嚴格的 localization-only。

若要真正停止 Local Mapping，需要在 live bootstrap 完成後呼叫：

```cpp
SLAM->ActivateLocalizationMode();
```

較合理的時機是：

```text
[ATLAS LIVE BOOTSTRAP] SUCCESS
```

之後再切換，否則可能破壞 live IMU chain 建立流程。

可先搜尋目前 wrapper 是否已有相關參數：

```bash
grep -R "ActivateLocalizationMode" \
  ~/orbslam3_ros2_ws/src/ORB_SLAM3_ROS2
```

---

## 9. Atlas 相容條件

建圖與再次定位可以使用不同 FPS 與 IMU frequency，但必須維持相同的：

```text
D405 實體相機
Infra1 optical frame
848 × 480 解析度
相機內參 fx、fy、cx、cy
rectified 影像模型
DepthMapFactor
Camera.bf / ThDepth
Camera–IMU 外參 IMU.T_b_c1
相機與 IMU 固定安裝關係
ORB Vocabulary
ORB descriptor 類型
mask 幾何位置
```

允許改變：

```text
Camera.fps：30 → 60
IMU.Frequency：100 → 200
是否載入 Atlas
是否儲存成新的 Atlas 名稱
Viewer 開關
```

不建議在載入既有 Atlas 時改變：

```text
影像解析度
相機裁切
相機光學 frame
內參
外參方向
rectified / raw 狀態
Vocabulary
```

---

## 10. ROS 2 輸出

目前已驗證發布：

```text
/orbslam3/pose
/orbslam3/map_odometry
TF: map -> camera_infra1_optical_frame
```

可檢查：

```bash
ros2 topic hz /orbslam3/pose
ros2 topic echo /orbslam3/pose --once

ros2 topic hz /orbslam3/map_odometry
ros2 topic echo /orbslam3/map_odometry --once

ros2 run tf2_ros tf2_echo \
  map camera_infra1_optical_frame
```

目前 pose、odometry、TF 只在 `Tracking::OK` 時發布，timestamp 使用影像時間。

若要接 Nav2，還需要正式定義：

```text
map
odom
base_link
camera_infra1_optical_frame
```

並決定由誰提供：

```text
map -> odom
odom -> base_link
base_link -> camera frame
```

目前不應直接把 `map -> camera` 當成已完成的 Nav2 frame tree。

---

## 11. Trajectory 輸出

目前在正常關閉時可自動儲存：

```text
CameraTrajectory.txt
KeyFrameTrajectory.txt
```

預期位置：

```text
/home/lab606/orbslam3_ros2_ws/CameraTrajectory.txt
/home/lab606/orbslam3_ros2_ws/KeyFrameTrajectory.txt
```

關閉 log 應包含：

```text
Saving camera trajectory ...
Saving keyframe trajectory ...
```

建議每次測試後立即另存：

```bash
mkdir -p ~/orbslam3_trajectories

STAMP=$(date +%Y%m%d_%H%M%S)

cp ~/orbslam3_ros2_ws/CameraTrajectory.txt \
  ~/orbslam3_trajectories/CameraTrajectory_${STAMP}.txt

cp ~/orbslam3_ros2_ws/KeyFrameTrajectory.txt \
  ~/orbslam3_trajectories/KeyFrameTrajectory_${STAMP}.txt
```

後續可用於：

```text
evo_ape
evo_rpe
閉環誤差
不同 FPS 比較
不同 IMU frequency 比較
不同 Atlas 比較
```

---

## 12. 完整 log 記錄方式

```bash
mkdir -p ~/orbslam3_logs

LOG=~/orbslam3_logs/rgbd_imu_$(date +%Y%m%d_%H%M%S).log

ros2 run orbslam3 rgbd-inertial \
  /home/lab606/slam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/lab606/slam_ws/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_rgbd_inertial_locating_200_60.yaml \
  true \
  --ros-args \
  -p imu_time_offset_sec:=0.0 \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw \
  2>&1 | tee "$LOG"

echo "Log saved to: $LOG"
```

目前自訂 log 包含：

```text
[TRACK DIAG]
[IMU STATE]
[TLM OPT MODE]
[RGB INPUT GAP]
[DEPTH INPUT GAP]
[RGBD PAIR REJECTED]
[RGBD-IMU SYNC]
[LOAD ATLAS]
[ATLAS RELOCALIZATION]
[ATLAS STATIC IMU]
[ATLAS LIVE ANCHOR]
[ATLAS LIVE BOOTSTRAP]
[SHUTDOWN]
[SAVE ATLAS]
```

效能測試時不要每 frame 印出所有 debug 訊息。大量 terminal I/O 會增加 jitter。建議正式版本改為每秒統計一次。

---

## 13. 已完成的重要修正

### 13.1 Camera–IMU 外參方向

✅ `IMU.T_b_c1` 的方向／矩陣定義已修正到可完成初始化、VIBA、Atlas relocalization 與 inertial tracking。

注意：

```text
不要再次對目前已驗證矩陣取 inverse。
```

但能運作不代表外參已達最終精度，仍需最終 Kalibr 驗證。

### 13.2 IMU integration period

目前使用：

```cpp
mImuPer = 1.0 / static_cast<double>(mImuFreq);
```

因此：

```text
100 Hz → 0.01 s
200 Hz → 0.005 s
```

runtime 已驗證：

```text
[IMU CONFIG] frequency=200 period=0.005
```

### 13.3 IMU timestamp offset

wrapper 參數：

```text
imu_time_offset_sec
```

定義：

```text
t_used = t_raw + imu_time_offset_sec
```

目前使用：

```text
0.0 s
```

早期固定 ±10 ms 測試沒有決定性改善，舊 Kalibr 約 ±88 ms 的 offset 也不能直接套用到目前資料鏈。

### 13.4 Atlas 再定位流程

目前 custom startup 已完成：

```text
1. 載入 Atlas
2. Visual relocalization
3. 暫時以 visual-only warmup
4. 蒐集約 2 秒靜態 IMU
5. 計算本次 session gyro bias
6. 建立 live KeyFrame anchor
7. 建立第一個 live inertial prior
8. 恢復一般 inertial tracking
```

這解決了「載入舊 Atlas 後，新的 IMU session 沒有連續 temporal chain」的問題。

### 13.5 Pose / Odometry / TF

✅ 已能發布 live pose、odometry 與 TF。  
⚠️ 尚未完成與 Nav2 的正式 frame tree 整合。

### 13.6 Trajectory 自動儲存

✅ `Ctrl+C` 時可自動輸出 CameraTrajectory 與 KeyFrameTrajectory。

---

## 14. 彩色 RGB-D-Inertial 分支測試

曾測試：

```text
Color rectified：/camera/camera/color/image_rect_raw
Aligned depth： /camera/camera/aligned_depth_to_color/image_raw
IMU：           /imu/data
```

純 RGB-D 啟動範例：

```bash
ros2 run orbslam3 rgbd \
  /home/lab606/slam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/lab606/slam_ws/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_color_aligned_rgbd_inertial.yaml \
  true \
  --ros-args \
  -r camera/rgb:=/camera/camera/color/image_rect_raw \
  -r camera/depth:=/camera/camera/aligned_depth_to_color/image_raw
```

使用 rectified color 時：

```yaml
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.k3: 0.0
```

測試結論：

- ✅ 彩色 RGB + aligned depth 可以正常輸入 ORB-SLAM3。
- ✅ 可建立 color 專用 Atlas。
- 🧪 加入 RGB-D 與 IMU 後，對重複紋理仍沒有根本解決。
- ⚠️ RGB-D pair、alignment、CPU 負載比 infra + native depth 更複雜。
- ✅ 目前正式 SLAM 基準仍採用 Infra1 + Depth + IMU。

---

## 15. 60 Hz 測試結論

### 15.1 60 Hz 建圖

60 Hz 影像端確實可穩定發布，但完整建圖時容易出現：

```text
RGB QUEUE Overflow
DEPTH QUEUE Overflow
RGBD RESYNC
image_dt = 0.033 / 0.050 / 0.066 s
```

原因不是單一相機頻率問題，而是：

```text
ORB extraction
Tracking
Local Mapping
KeyFrame 插入
Local BA
Loop Closing
Viewer
大量 debug log
```

共同造成的 real-time 壓力。

### 15.2 100 Hz IMU + 60 Hz image

平均每個影像區間只有：

```text
100 / 60 ≈ 1.67 筆 IMU
```

實際常出現：

```text
imu_count=1
Dropping image with insufficient IMU measurements
```

因此目前同步策略下不建議使用。

### 15.3 200 Hz IMU + 60 Hz image

平均：

```text
200 / 60 ≈ 3.33 筆 IMU
```

實際多為：

```text
imu_count=3 或 4
```

且載入 Atlas 後已能大部分時間保持：

```text
image_dt ≈ 0.016696 s
```

因此目前推薦：

```text
30/100 建圖
60/200 再定位
```

但要注意：提高頻率只能降低 frame-to-frame displacement，不能消除重複紋理的 perceptual aliasing。

---

## 16. 目前已知問題

### 16.1 重複紋理

RGB、Depth、IMU 與 60 Hz 都不能從根本上創造唯一視覺資訊。

目前觀察：

- 高頻率可降低相鄰影像位移。
- IMU 可提供短時間運動先驗。
- Depth 可提供 metric geometry。
- 但共面且週期性紋理仍可能造成 descriptor ambiguity。

因此要以同一路徑 A/B test 比較：

```text
30/100
60/200
```

而不是只比較是否完全失去重複紋理問題。

### 16.2 RGB-D 偶發相差一幀

仍可能出現：

```text
[RGBD PAIR REJECTED]
abs_diff ≈ 0.016696 s
```

這代表 infra 與 depth 剛好錯開一個 60 Hz frame。

目前不建議直接把同步容差放寬到 16.7 ms，否則可能把不同時間的影像與深度配成同一組。

### 16.3 目前定位仍新增地圖

尚未在 live bootstrap 後自動切換：

```cpp
ActivateLocalizationMode();
```

所以載入 Atlas 後仍會新增 KeyFrame 與 MapPoint。

### 16.4 大量 debug log

目前每 frame 或每三 frame輸出大量診斷訊息，可能影響 60 Hz real-time。

正式版建議改為每秒輸出統計：

```text
received
paired
dropped
imu_count distribution
tracking fps
tracking latency
mean inliers
min inliers
```

### 16.5 Mask

目前載入：

```text
/home/lab606/orbslam3_ros2_ws/src/ORB_SLAM3_ROS2/config/mask_left.png
```

已觀察遮罩比例約：

```text
100006 / 407040 = 24.57%
```

若相機安裝、裁切或解析度改變，必須重新製作 mask。

### 16.6 最終 Kalibr 尚未重做

最終校正必須使用：

```text
最終 Infra topic
最終解析度
最終 30/60 Hz 設定
最終 100/200 Hz IMU 設定
目前實際 timestamp pipeline
相機與 IMU 固定安裝
```

### 16.7 尚未完成定量精度評估

目前已驗證：

```text
能初始化
能重定位
能追蹤
能保存 Atlas
能輸出 trajectory
能在靜止時收斂
```

但尚未完成：

```text
ATE
RPE
長時間 drift
閉環誤差
重複路徑 repeatability
ground truth comparison
```

---

## 17. 使用前檢查清單

### 17.1 30/100 建圖

```text
[ ] ROS 2 Humble 已 source
[ ] wrapper workspace 已 source
[ ] Infra1 約 30 Hz
[ ] Depth 約 30 Hz
[ ] IMU 約 100 Hz
[ ] 使用建圖 YAML
[ ] YAML 沒有 LoadAtlasFromFile
[ ] SaveAtlasToFile 使用新名稱
[ ] auto exposure / exposure / gain 已確認
[ ] imu_time_offset_sec = 0.0
[ ] Viewer 是否開啟已確認
[ ] 初始化時有足夠平移與旋轉
[ ] 已看到 end VIBA 1
[ ] 已看到 end VIBA 2
[ ] 已看到 Tracking OK
[ ] Ctrl+C 後 Atlas 已保存
[ ] CameraTrajectory 已保存
[ ] KeyFrameTrajectory 已保存
[ ] 原始 Atlas 已備份
```

### 17.2 60/200 再定位

```text
[ ] Infra1 約 60 Hz
[ ] Depth 約 60 Hz
[ ] IMU 約 200 Hz
[ ] 使用 locating_200_60 YAML
[ ] LoadAtlasFromFile 指向穩定 Atlas
[ ] 相機內參、解析度、Tbc 與建圖時一致
[ ] 原始 Atlas 不會被同名覆寫
[ ] 啟動位置在舊 Atlas 可辨識區域
[ ] 啟動後先保持靜止約 2 秒
[ ] 已看到 ATLAS RELOCALIZATION SUCCESS
[ ] 已看到 ATLAS STATIC IMU READY
[ ] 已看到 ATLAS LIVE ANCHOR CREATED
[ ] 已看到 ATLAS LIVE BOOTSTRAP SUCCESS
[ ] 正常 imu_count 為 3 或 4
[ ] image_dt 多數接近 0.016696 s
[ ] 沒有連續 Tracking Lost
[ ] 知道目前仍會擴充地圖
```

---

## 18. 建議測試矩陣

### 18.1 同一路徑 A/B

```text
A：30 Hz image + 100 Hz IMU
B：60 Hz image + 200 Hz IMU
```

固定：

```text
同一 Atlas
同一曝光
同一 gain
同一路徑
同一移動速度
同一場景光線
同一 ORB features
同一 mask
```

記錄：

```text
Tracking Lost 次數
最低 inliers
低於 100 inliers 的時間
錯誤跳位次數
平均 image_dt
最大 image_dt
RGB-D rejected 次數
IMU insufficient 次數
CPU 使用率
pose latency
ATE / RPE
```

---

## 19. 待辦事項

### P0：封存目前成功版本

```text
30/100 建圖成功版本
60/200 Atlas 再定位成功版本
兩份 YAML
穩定 Atlas
兩份範例 log
README
```

建議版本名稱：

```text
d405_infra_depth_imu_atlas_v0.2
```

### P1：真正 localization-only

在：

```text
[ATLAS LIVE BOOTSTRAP] SUCCESS
```

後切換：

```cpp
ActivateLocalizationMode();
```

並驗證：

```text
KeyFrame 不再增加
MapPoint 不再增加
LocalMapping 停止
60 Hz tracking latency 是否下降
```

### P1：降低診斷輸出負載

將逐 frame log 改成每秒摘要。

### P1：定量 trajectory 評估

使用目前已可輸出的 trajectory 完成：

```text
evo_ape
evo_rpe
閉環誤差
固定路徑重複 5 次
30/100 與 60/200 對照
```

### P1：完整啟動指令封存

仍需補上：

```text
RealSense 30 Hz launch
RealSense 60 Hz launch
Xsens 100 Hz設定
Xsens 200 Hz設定
driver 版本
USB 設定
```

### P2：最終 Kalibr

### P2：正式 Nav2 frame tree

### P2：長時間穩定性測試

---

## 20. 目前正式結論

目前專案已不只是「能從空地圖啟動的 RGB-D-Inertial 原型」，而是已具備：

```text
D405 Infra1 + Depth + Xsens IMU
30/100 從空 Atlas 建圖
Atlas save
下一次執行 Atlas load
visual relocalization
本次 session 靜態 IMU初始化
live inertial anchor
live inertial bootstrap
60/200 持續追蹤
pose / odometry / TF發布
trajectory 自動輸出
```

目前最準確的系統定位是：

> **可儲存地圖並於後續 session 再次重定位與持續追蹤的研究原型。**

但仍須清楚標示：

> **目前載入 Atlas 後仍會擴充地圖，尚未完成嚴格 localization-only；重複紋理也尚未被根本解決。**
