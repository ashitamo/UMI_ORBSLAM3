# D405 + Xsens + ORB-SLAM3 RGB-D-Inertial System

本專案在 Ubuntu 22.04 與 ROS 2 Humble 上整合 Intel RealSense D405、Xsens
IMU、ORB-SLAM3，以及 UMI 資料收集與離線處理工具。

目前正式且已驗證的 SLAM 輸入為：

```text
影像：/camera/camera/infra1/image_rect_raw
深度：/camera/camera/depth/image_rect_raw（raw sensor_msgs/Image）
IMU： /imu/data
```

雖然執行檔名稱是 `rgbd-inertial`，`camera/rgb` 實際 remap 到 D405 Infra1，
因此目前系統基準是 **Infra–Depth–Inertial ORB-SLAM3**。

## 系統狀態

| 模式 | Infra1 / Depth | IMU | 用途 |
| --- | ---: | ---: | --- |
| Mapping | 30 Hz | 100 Hz | 從空 Atlas 建圖、完成 VIBA、儲存地圖 |
| Locating | 60 Hz | 200 Hz | 載入 Atlas、重定位、持續追蹤與地圖更新 |

目前 `locating` 不是 strict localization-only；Local Mapping 與 Loop Closing
會保持開啟。當操作範圍出現 Atlas 外的新特徵時，持續建圖有助於維持追蹤與
完整操作軌跡，這是目前正式設計，而不是待修正問題。

已完成的主要功能：

- 建立、儲存與重新載入 ORB-SLAM3 Atlas。
- RGB-D-Inertial 初始化、IMU prediction、VIBA 與 visual relocalization。
- 輸出 CameraTrajectory、KeyFrameTrajectory、pose、odometry 與 TF。
- 自動檢查 topics、倒數並錄製 MCAP bag。
- 輸出同步 color、逐幀彩色點雲、夾爪開度與 `T_BASE_EE(t)`。
- 檢查缺失 timestamp 並產生資料集說明與視覺化。

仍待完成：最終 Kalibr、ground truth/ATE/RPE 定量評估、長時間部署驗證，
以及可由命令列選擇 Atlas 的 map registry。

## 安裝

> **此 repository 不包含 RealSense 與 Xsens ROS 2 driver。** 首次建置前必須
> 自行安裝 `realsense2_camera`、`realsense2_camera_msgs` 與
> `xsens_mti_ros2_driver`，並 source driver 所在的 workspace。

完整的全新電腦安裝、submodule、相依套件、編譯與驗證流程：

**[系統安裝與重建指南](docs/install.md)**

目前完成項目、限制、已知問題與後續工作：

**[Repository 狀態與 Roadmap](docs/repo_state.md)**

快速檢查與建置：

```bash
cd ~/umi_ORB_SLAM3
./scripts/bootstrap.sh
./scripts/bootstrap.sh --build
source ros2_ws/install/setup.bash
```

## 專案結構

```text
umi_ORB_SLAM3/
├── core_ws/src/
│   ├── ORB_SLAM3/                 # 修改後的 ORB-SLAM3 fork
│   └── Pangolin/                  # viewer dependency
├── ros2_ws/src/
│   ├── ORB_SLAM3_ROS2/            # ROS 2 wrapper fork
│   ├── umi_orbslam3_bringup/      # Xsens 100/200 Hz 設定
│   ├── umi_dataset_tools/         # 收集、處理與視覺化工具
│   └── xsens_imu_navigation/
├── data/
│   ├── bags/                      # 原始 MCAP
│   ├── trajectories/              # ORB-SLAM3 軌跡
│   └── processed/                 # color、點雲、EE、夾爪輸出
├── runtime/orbslam/               # 可覆寫的 ORB-SLAM3 暫存
├── docs/
└── scripts/
```

`data/`、`runtime/`、build products、MCAP、PLY 與 Atlas 不加入 Git。若資料要
放在其他硬碟，可設定 `UMI_DATA_ROOT`；pipeline 也支援 `DATA_DIR`、
`BAGS_DIR`、`TRAJECTORY_DIR`、`PROCESSED_DIR` 與 `RUNTIME_DIR`。

## 啟動系統

RGB-D+IMU 使用四個 terminal：RealSense、Xsens、頻率檢查與 ORB-SLAM3。
純 RGB-D 使用三個 terminal：RealSense、頻率檢查與 ORB-SLAM3，不需要 Xsens。
每個 terminal 都要載入其需要的 ROS overlay；Mapping 與 Locating 擇一執行。

### Mapping：30 Hz Image + 100 Hz IMU

Terminal 1 — D405：

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

Terminal 2 — Xsens：

```bash
source /opt/ros/humble/setup.bash
# source ~/umi_legacy/ros2_ws/install/setup.bash  # driver 在此 workspace 時需要
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash

PARAM_FILE="$(ros2 pkg prefix --share umi_orbslam3_bringup)/config/xsens_mapping_100hz.yaml"
ros2 launch umi_orbslam3_bringup xsens.launch.py param_file:="$PARAM_FILE"
```

Terminal 3 — 頻率檢查：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /imu/data
```

Terminal 4 — ORB-SLAM3 Mapping：

```bash
source /opt/ros/humble/setup.bash
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash

ROOT=~/umi_ORB_SLAM3
mkdir -p "$ROOT/runtime/orbslam/rgbdi_mapping"

ros2 run orbslam3 rgbd-inertial \
  "$ROOT/core_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  "$ROOT/core_ws/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_rgbd_inertial_mapping.yaml" \
  true \
  "$ROOT/runtime/orbslam/rgbdi_mapping/CameraTrajectory.txt" \
  "$ROOT/runtime/orbslam/rgbdi_mapping/KeyFrameTrajectory.txt" \
  --ros-args \
  -p atlas_path:="$ROOT/maps/lab_b36_rgbdi/atlas.osa" \
  -p imu_time_offset_sec:=0.0 \
  -p publish_pose:=false \
  -p publish_odometry:=false \
  -p publish_tf:=false \
  -p map_frame_id:=map \
  -p camera_frame_id:=camera_infra1_optical_frame \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw
```

確認約為 30/30/100 Hz 後再啟動 Mapping。從空 Atlas 初始化時先短暫靜止，
再做適量 XYZ 與 roll/pitch/yaw 運動；等待 VIBA 1、VIBA 2 完成後才開始正式
建圖。以 `Ctrl+C` 正常結束，讓 Atlas 與軌跡寫完。

此命令將 RGB-D+IMU 地圖儲存到 `maps/lab_b36_rgbdi/atlas.osa`，不依賴目前
工作目錄。輸出 Atlas 已存在時預設覆寫；若要保護既有輸出，可加上
`-p atlas_overwrite:=false`。RGB-D 與 RGB-D+IMU 地圖分開存放。

### Locating：60 Hz Image + 200 Hz IMU

Terminal 1 — D405：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=false \
  enable_sync:=true \
  align_depth.enable:=false \
  pointcloud.enable:=false \
  depth_module.depth_profile:=848x480x60 \
  depth_module.infra_profile:=848x480x60 \
  depth_module.color_profile:=848x480x60
```

Terminal 2 — Xsens：

```bash
source /opt/ros/humble/setup.bash
source ~/umi_legacy/ros2_ws/install/setup.bash  # driver 在此 workspace 時需要
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash

PARAM_FILE="$(ros2 pkg prefix --share umi_orbslam3_bringup)/config/xsens_localization_200hz.yaml"
ros2 launch umi_orbslam3_bringup xsens.launch.py param_file:="$PARAM_FILE"
```

Terminal 3 — 檢查 topics 與頻率：

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /imu/data
```

確認約為 60/60/200 Hz 後，啟動 ORB-SLAM3 live locating。

Terminal 4 — ORB-SLAM3 Locating：

```bash
source /opt/ros/humble/setup.bash
# source ~/umi_legacy/ros2_ws/install/setup.bash  # driver 在此 workspace 時需要
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash

ROOT=~/umi_ORB_SLAM3
mkdir -p "$ROOT/runtime/orbslam/rgbdi_locating"

ros2 run orbslam3 rgbd-inertial \
  "$ROOT/core_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  "$ROOT/core_ws/src/ORB_SLAM3/Examples/RGB-D-Inertial/D405_rgbd_inertial_locating.yaml" \
  true \
  "$ROOT/runtime/orbslam/rgbdi_locating/CameraTrajectory.txt" \
  "$ROOT/runtime/orbslam/rgbdi_locating/KeyFrameTrajectory.txt" \
  --ros-args \
  -p atlas_path:="$ROOT/maps/lab_b36_rgbdi/atlas.osa" \
  -p imu_time_offset_sec:=0.0 \
  -p publish_pose:=false \
  -p publish_odometry:=false \
  -p publish_tf:=false \
  -p map_frame_id:=map \
  -p camera_frame_id:=camera_infra1_optical_frame \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw
```

ORB_SLAM3_ROS2 會依 remap 後的 topic 名稱與 publisher 型別自動選擇 depth
subscriber。`/compressedDepth` 使用 `sensor_msgs/msg/CompressedImage` 並在
wrapper 內解碼 16UC1 PNG；raw topic 則使用 `sensor_msgs/msg/Image`，不需要
額外參數。

上面的 `true` 會開啟 ORB-SLAM3 Viewer，適合收集資料前快速確認是否已在正確
地圖位置完成重定位。效能測試時可改成 `false`。

此命令載入 Mapping 儲存的 RGB-D+IMU Atlas：

```text
~/umi_ORB_SLAM3/maps/lab_b36_rgbdi/atlas.osa
```

定位期間仍持續更新地圖，結束時預設另存同目錄的 `atlas_locating.osa`。
再次定位會預設覆寫該定位輸出；若要保留，可用
`-p atlas_save_path:=<新的輸出路徑>` 另存，或用 `-p atlas_overwrite:=false`
拒絕覆寫。預設載入的 `atlas.osa` 不會被覆寫。
若沿用舊地圖，將 `atlas_path` 改為該 RGB-D+IMU Atlas 的實際位置即可。

相機應先放在 Atlas 可辨識的位置；visual relocalization 後繼續靜止約 2 秒，
等待：

```text
[ATLAS RELOCALIZATION] SUCCESS
[ATLAS STATIC IMU] READY
[ATLAS LIVE ANCHOR] CREATED
[ATLAS LIVE BOOTSTRAP] SUCCESS
```

完成後才開始移動。正常 60/200 狀態下，`image_dt` 約 0.0167 秒，每張影像
通常有 3–4 筆 IMU。

> RealSense/Xsens 的真實輸出頻率必須與 settings YAML 的 `Camera.fps` 和
> `IMU.Frequency` 一致；修改 YAML 不會改變 driver 的實際頻率。

### 純 RGB-D Mapping：30 Hz Image（無 IMU）

純 RGB-D 實際使用 Infra1 + Depth，解析度為 848×480；建圖 YAML 設定為
30 Hz，定位 YAML 設定為 60 Hz。RGB-D 與 RGB-D+IMU 請各自建立 Atlas。

Terminal 1 — D405：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=false \
  enable_sync:=true \
  align_depth.enable:=false \
  pointcloud.enable:=false \
  depth_module.depth_profile:=848x480x30 \
  depth_module.infra_profile:=848x480x30
```

Terminal 2 — 頻率檢查：

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

逐一執行，每次按 Ctrl+C 結束該項檢查；確認兩者約為 30 Hz。

Terminal 3 — ORB-SLAM3 RGB-D Mapping：

```bash
source /opt/ros/humble/setup.bash
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash
ROOT=~/umi_ORB_SLAM3

ros2 run orbslam3 rgbd \
  "$ROOT/core_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  "$ROOT/core_ws/src/ORB_SLAM3/Examples/RGB-D/D405_rgbd_mapping.yaml" \
  true \
  "$ROOT/runtime/orbslam/rgbd_mapping/CameraTrajectory.txt" \
  "$ROOT/runtime/orbslam/rgbd_mapping/KeyFrameTrajectory.txt" \
  --ros-args \
  -p atlas_path:="$ROOT/maps/lab_b36_rgbd/atlas.osa" \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw
```

確認 Viewer 已正常追蹤後，緩慢移動相機建立地圖；純 RGB-D 不需要等待 IMU
bias 或 VIBA。完成後按 Ctrl+C，等待 Atlas 與軌跡儲存完成。
地圖輸出為 `maps/lab_b36_rgbd/atlas.osa`，預設覆寫既有輸出。

### 純 RGB-D Locating：60 Hz Image（無 IMU）

Terminal 1 — D405：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=false \
  enable_sync:=true \
  align_depth.enable:=false \
  pointcloud.enable:=false \
  depth_module.depth_profile:=848x480x60 \
  depth_module.infra_profile:=848x480x60 \
  depth_module.color_profile:=848x480x60

```

若建圖後相機仍以 30 Hz 執行，請先停止該相機節點，再以上述 60 Hz 設定啟動。
此處為即時定位測試，Color 關閉；後續要轉出彩色影像、彩色點雲與夾爪量測時，
仍需啟用並錄製 Color。

Terminal 2 — 頻率檢查：

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

逐一確認兩者約為 60 Hz，無需檢查 `/imu/data`。

Terminal 3 — ORB-SLAM3 RGB-D Locating：

```bash
source /opt/ros/humble/setup.bash
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash
ROOT=~/umi_ORB_SLAM3

ros2 run orbslam3 rgbd \
  "$ROOT/core_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  "$ROOT/core_ws/src/ORB_SLAM3/Examples/RGB-D/D405_rgbd_locating.yaml" \
  true \
  "$ROOT/runtime/orbslam/rgbd_locating/CameraTrajectory.txt" \
  "$ROOT/runtime/orbslam/rgbd_locating/KeyFrameTrajectory.txt" \
  --ros-args \
  -p atlas_path:="$ROOT/maps/lab_b36_rgbd/atlas.osa" \
  -r camera/rgb:=/camera/camera/infra1/image_rect_raw \
  -r camera/depth:=/camera/camera/depth/image_rect_raw
```

先將相機放在已建地圖能辨識的位置，等待 `[ATLAS RELOCALIZATION] SUCCESS`，
並確認 Viewer 的位置正確後再移動。純 RGB-D 不等待靜止 IMU 或 IMU live bootstrap。
重定位後依視覺追蹤與新特徵的條件新增 KeyFrame、MapPoint，持續更新地圖；
靜止或未符合新增條件時，KeyFrame 數量不一定增加。

正常結束後，軌跡保存在 `runtime/orbslam/rgbd_locating/`；更新後的地圖預設
存成 `maps/lab_b36_rgbd/atlas_locating.osa` 並允許覆寫，載入的 `atlas.osa` 保留。

### 共用 Atlas 與輸出參數

兩個 executable（`rgbd`、`rgbd-inertial`）都支援上述五個位置參數與：

- `atlas_path`：Mapping 的輸出 Atlas；Locating 的輸入 Atlas。接受相對或絕對
  路徑，可包含 `.osa`。YAML 有非空 `System.LoadAtlasFromFile` 時視為 Locating。
- `atlas_save_path`：選用，指定另一個輸出 Atlas，需搭配 `atlas_path`。
  Locating 預設另存為同目錄的 `atlas_locating.osa`，並保持地圖更新。
- `atlas_overwrite`：預設 `true`，允許覆寫輸出 Atlas；若指定
  `-p atlas_overwrite:=false`，輸出已存在便拒絕啟動。此設定同時適用於 Mapping 與 Locating。

指定 `atlas_path` 後不必切換到特定工作目錄；wrapper 產生暫存 YAML，原始 YAML
保持不變。未提供此參數時沿用 YAML 的舊路徑行為。`true` 可改成 `false` 關閉
Viewer。Depth 可用 raw 或 16UC1 PNG compressedDepth，影像 QoS 為
RELIABLE / KEEP_LAST(10) / VOLATILE。純 RGB-D 也保留舊的兩個位置參數用法。

## 資料收集與處理

收集工具與批次腳本皆支援 `--sensor rgbd`，預設為 `rgbd-inertial`。
離線轉檔共用 `process_rgbd_bags.py`。

### 1. 收集資料

先啟動 RealSense 與 Xsens：

```bash
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash
ros2 run umi_dataset_tools collect_rgbd_bag --raw-depth --prefix data0912
```

工具會檢查 topics、頻率與 RGB-D timestamp，要求靜止 5 秒並顯示
`3、2、1、GO`。資料預設寫入 `data/bags/`。

詳細說明：[Bag 收集工具](ros2_ws/src/umi_dataset_tools/COLLECT_RGBD_BAG.md)

純 RGB-D 收集不需要 Xsens，但必須啟用 Color（將相機啟動參數設為
`enable_color:=true`），確認 `/camera/camera/color/image_raw/compressed` 與
Infra1、Depth 的 header timestamp 同步後執行：

```bash
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash
ros2 run umi_dataset_tools collect_rgbd_bag --raw-depth --sensor rgbd --prefix data0927
```

此模式不檢查或錄製 `/imu/data`，其餘健康檢查仍保留。預設保留 5 秒靜止準備
與 3 秒倒數；純 RGB-D 可用 `--stationary-seconds 0` 省略靜止準備，這個計時器
不代表已確認 SLAM 重定位成功。

目前建議明確加上 `--raw-depth` 錄製原始深度（工具未指定時仍預設 compressed depth）：

```bash
ros2 run umi_dataset_tools collect_rgbd_bag \
  --raw-depth \
  --prefix data0912
```

執行前確認 `/camera/camera/depth/image_rect_raw` 有資料。暫時保留
`enable_sync:=true` 並使用 raw depth；compressedDepth 訂閱造成 color 停止及
深度降頻的實機觀察，見 [Repository 狀態](docs/repo_state.md#compresseddepth-實機發布異常)。

### 2. ORB-SLAM3 與資料轉換

RGB-D+IMU（60 Hz 影像／200 Hz IMU）：

```bash
cd ~/umi_ORB_SLAM3
ros2_ws/scripts/run_orbslam_and_process_bags.sh \
  --raw-depth \
  --sensor rgbd-inertial \
  --atlas-path "$PWD/maps/lab_b36_rgbdi/atlas.osa" \
  --domain-id 42 \
  --bag-rate 0.4 \
  data0912_{01..20}
```

純 RGB-D（60 Hz 影像，不需要 IMU）：

```bash
cd ~/umi_ORB_SLAM3
ros2_ws/scripts/run_orbslam_and_process_bags.sh \
  --raw-depth \
  --sensor rgbd \
  --atlas-path "$PWD/maps/lab_b36_rgbd/atlas.osa" \
  --domain-id 42 \
  --bag-rate 0.4 \
  data0927_{01..20}
```

`--sensor` 決定 executable 與預設定位 YAML；純 RGB-D 不要求 IMU topic，
不執行啟動 IMU 檢查，也不等待 IMU 訂閱。影像 topic 與軌跡有效性檢查仍保留。
兩者都會執行相同的 `process_rgbd_bags`；bag 仍需包含 Color、內參與外參才能
完整轉檔。`--settings` 可覆蓋預設 YAML，請與錄製頻率一致。
`--atlas-path` 接受相對或絕對路徑；未指定時沿用 YAML 與舊工作目錄的路徑規則。
定位更新地圖預設另存同目錄的 `atlas_locating.osa`，每份 bag 可覆寫此定位輸出。

`--domain-id` 應使用目前沒有其他 ROS 2 nodes 的值。腳本會讓 rosbag playback
與 ORB-SLAM3 共用此 domain，避免實機 RealSense/Xsens publisher 的 timestamp
或訊息混入離線回放。執行 pipeline 的 terminal 不需要事先手動設定
`ROS_DOMAIN_ID`；請不要在同一個 domain 啟動實機感測器。

上述 pipeline 指令皆明確加入 `--raw-depth`，與目前建議的錄製格式一致。
處理既有 compressedDepth bag 時才移除此旗標；請勿混用輸入格式：

```bash
ros2_ws/scripts/run_orbslam_and_process_bags.sh \
  --raw-depth \
  --domain-id 42 \
  --sensor rgbd-inertial \
  --atlas-path "$PWD/maps/lab_b36_rgbdi/atlas.osa" \
  --bag-rate 0.4 \
  data0912_{01..20}
```

compressedDepth 的既有支援仍保留：ORB_SLAM3_ROS2 與逐幀點雲工具可直接
解碼 bag 中的 16UC1 PNG compressedDepth，RVL 尚未支援。這不代表實機
60 Hz 壓縮發布已通過驗證；目前日常指令以 raw depth 為主。

預設輸出到 `data/trajectories/<bag>/`、`data/processed/<bag>/` 與
`runtime/orbslam/`。

詳細輸出格式、座標系與裁切參數：
[RGB-D Bag 處理工具](ros2_ws/src/umi_dataset_tools/PROCESS_RGBD_BAGS.md)

### 3. 手動 ORB-SLAM3 指令

- `mapping.txt`：從空 Atlas 建圖的終端指令。
- `locating.txt`：載入 Atlas 後再次定位的終端指令。

執行前必須確認 settings YAML、Atlas 路徑、相機與 IMU 頻率一致。

## ROS 2 輸出

Tracking 為 `OK` 時可發布：

```text
/orbslam3/pose
/orbslam3/map_odometry
TF: map -> camera_infra1_optical_frame
```

```bash
ros2 topic hz /orbslam3/pose
ros2 topic echo /orbslam3/map_odometry --once
ros2 run tf2_ros tf2_echo map camera_infra1_optical_frame
```

這還不是完整 Nav2 frame tree；`map`、`odom`、`base_link` 與 camera frame 的
唯一發布責任尚未正式定義。

## 資料與座標系

- ORB 原始軌跡是 `T_WORLD_TRACKING(t)`；`TRACKING` 是
  `camera_infra1_optical_frame`。
- `trajectory_base_ee.csv` 是 `T_BASE_EE(t)`。
- `BASE` 是 UMI 把手底部中心座標系，不是 ORB-SLAM map 原點。
- 每份 PLY 位於該 timestamp 的 `camera_color_optical_frame`，不融合到 `WORLD`。
- Depth 點先用 bag 內 `T_COLOR_DEPTH` 轉到 color camera，再投影取色。

每個處理後資料夾的 `DATASET_INFO.md` 與 `camera_calibration.yaml` 會保存
frame、內外參、同步與輸出說明。

## Git 與 Submodules

本 repository 使用 `ashitamo/ORB_SLAM3`、`stevenlovegrove/Pangolin` 與
`ashitamo/ORB_SLAM3_ROS2` 三個 submodule。修改內層後，先在該 submodule
commit/push，再回根目錄提交新的 submodule commit。

## 歷史文件

- [D405 + Xsens + ORB-SLAM3 v2 紀錄](docs/previous_readme/README_D405_Xsens_ORB_SLAM3_v2.md)
- [D405 + Xsens + ORB-SLAM3 v4 紀錄](docs/previous_readme/README_D405_Xsens_ORB_SLAM3_v4.md)
- [D405 + Xsens + ORB-SLAM3 v5 完整紀錄](docs/previous_readme/README_D405_Xsens_ORB_SLAM3_v5.md)

歷史文件可能包含舊路徑與已更新的操作方式；目前應以本 README、
[安裝指南](docs/install.md)、[Repository 狀態](docs/repo_state.md) 與套件內
文件為準。
