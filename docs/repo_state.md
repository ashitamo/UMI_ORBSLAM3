# Repository 狀態與 Roadmap

本文件記錄目前 repository 已完成的功能、已驗證範圍、限制與後續工作。
日常啟動方式請看 [README](../README.md)，全新建置請看
[install.md](install.md)。

標記說明：

- ✅ 已由目前程式、執行輸出或測試確認
- 🧪 已完成實驗，但結論仍受場景與測試條件限制
- ⚠️ 可運作，但仍有工程限制或尚未完成定量驗證
- 📌 尚待完成
- 🔎 需重新檢查目前原始碼後才能確認

## compressedDepth 實機發布異常

- 🧪 使用者實機版本：`realsense2_camera 4.58.3`、
  `compressed_depth_image_transport 2.5.5`；以下為目前場景的觀察，非所有硬體的結論。
- 🧪 `enable_sync=true` 時，單獨訂閱 compressedDepth 即使獨立 color
  檢查器由約 60 Hz 降至 0，取消訂閱後恢復；depth 約 45–47 Hz。
  RELIABLE 與 BEST_EFFORT 訂閱都能重現，不需要 collector 或解壓縮。
- 🧪 `enable_sync=false` 時 color 可維持，但僅訂閱 compressedDepth
  仍約 45 Hz。raw depth 對照接近 60 Hz，另有少量缺幀，並非完全無損。
- ⚠️ 問題指向 compressedDepth 發布路徑及同步流程的交互影響；尚未確認
  是編碼效能、共用發布阻塞、DDS 或其他原因，不直接認定為特定版本 bug。
- ✅ README 暫以 raw depth 為主，收集與批次轉檔明確加 `--raw-depth`；
  影像接收 QoS 仍為 RELIABLE + KEEP_LAST(10)，壓縮功能未移除。
- ⚠️ 不以關閉同步作為正式解法：資料轉換要求影像 header timestamp 一致，
  關閉同步後必須重新驗證，不能只看 color 是否恢復。
- 📌 待測：訂閱 compressedDepth 期間的 raw depth 頻率、color raw 是否停止、
  相機完整設定與發布／編碼效能；修改後需重新驗證同步及長時間錄製。
- ✅ Collector 已撤回本次 Hz 診斷新增的 warmup、check-only 與狀態欄位，
  恢復原先健康檢查；錄製開始後的靜止準備與 GO 倒數仍保留。

## 1. 目前定位

本系統是可儲存 Atlas、在後續 session 重新定位並持續追蹤的研究原型：

```text
D405 Infra1 + native Depth + Xsens IMU
30/100 從空 Atlas 建圖
Atlas save/load
visual relocalization
live session 靜態 IMU 初始化
live inertial anchor/bootstrap
60/200 持續追蹤
pose / odometry / TF
trajectory 與資料集輸出
```

✅ 目前 `locating` 刻意保留 Local Mapping 與 Loop Closing，不是 strict
localization-only。這是目前正式設計：當操作範圍超出既有 Atlas，或環境出現
地圖外特徵時，系統可持續新增 KeyFrame 與 MapPoint，避免只依賴舊地圖而使
追蹤中斷，並提高完整操作軌跡保持正常的機會。

## 2. 已完成

### ORB-SLAM3

- ✅ 從空 Atlas 完成 RGB-D-Inertial 初始化、VIBA 1 與 VIBA 2。
- ✅ Atlas 儲存、載入與 visual relocalization。
- ✅ 載入 Atlas 後蒐集當次 session 的靜態 IMU。
- ✅ 建立 live KeyFrame anchor 與 inertial prior，恢復一般 inertial tracking。
- ✅ `Ctrl+C` 正常關閉時輸出 CameraTrajectory 與 KeyFrameTrajectory。
- ✅ 可由執行參數指定 viewer 與 trajectory 輸出路徑。

### ROS 2 Wrapper

- ✅ Infra1、raw/compressedDepth 與 IMU 輸入及同步診斷。
- ✅ ORB_SLAM3_ROS2 會依 remap topic 名稱或 publisher 型別自動選擇
  `sensor_msgs/Image` 或 `sensor_msgs/CompressedImage`。
- ✅ `/orbslam3/pose`、`/orbslam3/map_odometry` 與
  `map -> camera_infra1_optical_frame` TF。
- ✅ 可關閉 viewer，並可停用 pose、odometry、TF publisher。
- ✅ 相機 mask 與 ORB-SLAM3 source 路徑已改為可攜式建置設定。

### 資料工具

- 錄製前 topic 存在性、頻率、訊息數量與 timestamp 健康檢查。
- 自動 bag 命名、靜止倒數、MCAP 錄製與結束統計。
- 批次 rosbag playback、ORB-SLAM3 trajectory 與失敗 bag 跳過。
- 精確同步 Infra1/Depth/Color，並記錄缺失 timestamp。
- 使用 bag 內 depth-to-color 外參建立逐幀彩色點雲。
- 輸出 `T_WORLD_TRACKING(t)`、`T_BASE_EE(t)`、color、gripper 與校準資訊。
- 頭尾使用不同半徑裁切軌跡：預設 0.02 m / 0.10 m。
- 夾爪波形、軌跡速度、點雲/EE 等視覺化工具。

### Repository

- ORB-SLAM3、Pangolin、ORB_SLAM3_ROS2 使用 Git submodules。
- runtime/data 與 source/build 分離。
- 提供 `scripts/bootstrap.sh` 與安全的資料移轉腳本。
- `umi_dataset_tools` 與 `orbslam3` 已在目前電腦成功建置。

## 3. 已驗證基準

| 工作模式 | Image | IMU | 結果 |
| --- | ---: | ---: | --- |
| Mapping | 30 Hz | 100 Hz | 可初始化、建圖、VIBA、儲存 Atlas |
| Locating | 60 Hz | 200 Hz | 可載入 Atlas、重定位、bootstrap、持續追蹤 |
| 60 Hz image | 100 Hz | 高風險 | 常只有一筆 IMU，可能丟棄 image |
| 60 Hz mapping | 200 Hz | 有壓力 | Local BA、viewer、log 等可能造成 queue overflow |

推薦基準仍是 **30/100 mapping** 與 **60/200 locating**。

定位設定統一使用 `D405_rgbd_inertial_locating.yaml`（60 Hz 影像／200 Hz IMU）；
舊的 `D405_rgbd_inertial_locating_200_60.yaml` 保留相同內容以相容既有命令。
純 RGB-D 的 `D405_rgbd_locating.yaml` 同樣使用 60 Hz，建圖維持 30 Hz。

## 4. 已知問題

### 純 RGB-D 啟動介面驗證（2026-09-24）

- ✅ `rgbd` 支援與 `rgbd-inertial` 相同的五個位置參數：vocabulary、settings、
  viewer、CameraTrajectory、KeyFrameTrajectory；原本兩個位置參數仍可使用。
- ✅ 兩者共用 `atlas_path`、`atlas_save_path` 與 `atlas_overwrite` 路徑處理。
- ✅ 在獨立 ROS domain 以合成 Infra1 和 16UC1 PNG compressedDepth 建圖，
  產生 100 筆相機軌跡；重新載入 Atlas 後成功重定位並產生 99 筆軌跡。
- ✅ 修正 Atlas 重定位後無條件等待 IMU 的問題：僅慣性 sensor 進入靜止 IMU、
  bias、preintegrator 與 live-anchor 初始化；純 RGB-D 使用正常視覺 KeyFrame 條件。
- 🧪 合成平移影像測試確認同一張 map（ID 0）在定位後持續擴增：KeyFrame
  由 1 增至 40、MapPoint 由 1,502 增至 3,963，輸出 239 筆定位軌跡，
  並未出現等待靜止 IMU 的訊息。此測試不代表實機精度或漂移的驗證。
- ✅ 兩者 raw depth 訂閱均驗證為 RELIABLE / KEEP_LAST(10)，空輸入正常退出
  並回傳 2、跳過軌跡輸出；缺少 Atlas 會提早報錯。`atlas_overwrite` 預設為
  `true`；指定 `false` 時才拒絕覆寫既有輸出。
- ✅ 修正核心 `mptViewer` 未初始化造成關閉 Viewer 時退出崩潰的問題；
  此修正只初始化 thread 指標，沒有修改 SLAM 演算法。
- 🧪 上述是合成資料的流程測試；純 RGB-D 在真實場景的重定位率與漂移仍待測試。
- ✅ 收集工具支援 `--sensor rgbd`，略過 IMU 檢查及錄製；預設維持 `rgbd-inertial`。
- ✅ 批次腳本支援 `--sensor rgbd|rgbd-inertial` 與 `--atlas-path`；純 RGB-D
  不要求或等待 IMU，不執行 startup IMU 檢查，離線轉檔保持共用。

### P0 — Atlas 選擇與保護

- ✅ `rgbd` 與 `rgbd-inertial` 可透過 `atlas_path` 覆蓋 YAML 的 Atlas 路徑；
  定位預設另存 `_locating.osa`，既有輸出預設允許覆寫，可用 `atlas_overwrite:=false` 保護。
- 未提供 `atlas_path` 時，沿用 settings YAML 的 `System.LoadAtlasFromFile`。
- pipeline 尚未提供 `--map <name>` 與 `--save-map <name>`。
- 缺少只讀 master Atlas、session Atlas 與升級紀錄制度。
- 不應用 locating session 直接覆寫唯一一份穩定 master Atlas。

建議建立：

```text
maps/<map_name>/atlas.osa
maps/<map_name>/metadata.yaml
runtime/orbslam/generated_settings.yaml
```

由 wrapper 產生暫存 settings，不修改 ORB-SLAM3 核心。

### ⚠️ P1 — 重複紋理

共面、週期性紋理仍可能造成 descriptor ambiguity 與錯誤軌跡。提高 FPS、
IMU 頻率或加入 depth 可降低風險，但不能創造唯一視覺資訊。

### ⚠️ P1 — 偶發 RGB-D 錯幀

60 Hz 時偶爾出現約 0.0167 秒的 Infra/Depth 差異，代表錯開一個 frame。
不能直接把同步容差放寬到 16.7 ms，否則會配對不同時刻的資料。

### ⚠️ P1 — 診斷輸出負載

逐 frame debug log 可能增加 60 Hz tracking jitter。正式版本應改為每秒摘要：

```text
received / paired / dropped
IMU count distribution
tracking FPS and latency
mean/min inliers
```

### ⚠️ P1 — Mask 維護

目前 mask 針對 848×480 與現有相機安裝。若解析度、裁切或硬體安裝改變，
必須重新製作並驗證 mask。

## 5. 尚未完成的驗證

- 📌 使用最終 Infra topic、頻率與實際 timestamp pipeline 重新執行 Kalibr。
- 📌 Ground truth、ATE、RPE、閉環誤差與長時間 drift。
- 📌 固定路徑至少五次 repeatability 測試。
- 🧪 30/100 與 60/200 同路徑 A/B test。
- 📌 不同 Atlas 更新前後的 relocalization 成功率。
- 📌 長時間運作、USB 負載、CPU latency 與 bag drop 統計。
- 📌 正式 Nav2 frame tree 與唯一 localization source。
- 📌 在全新 Ubuntu 22.04 電腦完整執行 `docs/install.md`。

## 6. Roadmap

### P0

1. 實作 map registry、`--map`、`--save-map` 與 master Atlas 保護。
2. 封存穩定 Atlas、對應 YAML、範例 log 與硬體設定。

### P1

1. 將逐 frame log 改為每秒診斷摘要。
2. 建立 evo APE/RPE 與 repeatability 評估流程。
3. 記錄 locating session 新增 KeyFrame/MapPoint 與 loop closure。
4. 補齊 RealSense/Xsens driver 版本、USB 與硬體安裝資訊。

### P2

1. 執行最終 Kalibr。
2. 建立正式 Nav2 frame tree。
3. 完成長時間穩定性測試。

## 7. 維護規則

- 新功能完成或驗證結論改變時，更新本文件。
- 操作方式改變時，更新根目錄 README。
- 安裝依賴或建置方式改變時，更新 `docs/install.md`。
