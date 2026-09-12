# UMI RGB-D Dataset System

This repository contains the modified ORB-SLAM3 stack, ROS 2 wrapper, RGB-D bag
collector, trajectory conversion, point-cloud export, and visualization tools.

## Directory Layout

```text
umi_ORB_SLAM3/
├── core_ws/src/                 # ORB-SLAM3 and Pangolin source
├── ros2_ws/src/                 # ROS 2 packages only
├── data/
│   ├── bags/                    # recorded MCAP bags
│   ├── trajectories/            # per-bag ORB-SLAM3 trajectories
│   └── processed/               # exported datasets
├── runtime/orbslam/             # temporary ORB-SLAM3 output
└── scripts/                     # setup and migration helpers
```

`data/`, `runtime/`, ROS build products, maps, MCAP files, and generated point
clouds are ignored by Git. Set `UMI_DATA_ROOT=/path/on/another/disk` to change
the Python tools' data root. The pipeline shell script also accepts `DATA_DIR`,
`BAGS_DIR`, `TRAJECTORY_DIR`, `PROCESSED_DIR`, and `RUNTIME_DIR`.

## Existing Data Migration

The migration tool never overwrites a destination with the same name. Preview
the operations first, then execute them:

```bash
cd ~/umi_ORB_SLAM3
./scripts/migrate_runtime_data.sh
./scripts/migrate_runtime_data.sh --execute
```

## Build on Another Computer

Target environment: Ubuntu 22.04, ROS 2 Humble, a working RealSense ROS driver,
and the Xsens packages used by this project. Clone all Git submodules first:

```bash
git clone --recurse-submodules <ROOT_REPOSITORY_URL> ~/umi_ORB_SLAM3
cd ~/umi_ORB_SLAM3
./scripts/bootstrap.sh
./scripts/bootstrap.sh --build
source ros2_ws/install/setup.bash
```

The check-only invocation creates the data layout and reports missing source or
ROS prerequisites. `--build` builds Pangolin, ORB-SLAM3 when needed, and the ROS
2 workspace. System packages and camera/IMU drivers remain machine-specific and
must be installed before running it.

## Normal Workflow

Record a bag; its name is generated automatically:

```bash
source ~/umi_ORB_SLAM3/ros2_ws/install/setup.bash
ros2 run umi_dataset_tools collect_rgbd_bag --prefix data0909
```

Run locating and dataset export:

```bash
cd ~/umi_ORB_SLAM3
ros2_ws/scripts/run_orbslam_and_process_bags.sh --bag-rate 0.4 data0909_{01..20}
```

Default outputs are `data/trajectories`, `data/processed`, and
`runtime/orbslam`. Collector details are in
`ros2_ws/src/umi_dataset_tools/COLLECT_RGBD_BAG.md`; export details are in
`ros2_ws/src/umi_dataset_tools/PROCESS_RGBD_BAGS.md`.

## Publishing to Git

ORB-SLAM3, Pangolin, and ORB_SLAM3_ROS2 have independent history, so publish
your modified ORB-SLAM3 and ORB_SLAM3_ROS2 forks first, then track these source
directories as submodules in the root repository. Do not add them as ordinary
embedded repositories.

```bash
# In each modified nested repository: point origin at your fork, commit, push.
git remote -v
git status
git add <reviewed-files>
git commit -m "Describe local ORB-SLAM3 changes"
git push -u origin <branch>

# At the project root. Remove an empty placeholder only if `ls -A .git` is empty.
cd ~/umi_ORB_SLAM3
rmdir .git 2>/dev/null || true
git init
git branch -M main
git submodule add --force <YOUR_ORB_SLAM3_FORK_URL> core_ws/src/ORB_SLAM3
git submodule add --force <PANGOLIN_URL> core_ws/src/Pangolin
git submodule add --force <YOUR_ORB_SLAM3_ROS2_FORK_URL> ros2_ws/src/ORB_SLAM3_ROS2
git add .gitignore .gitmodules README.md core_ws ros2_ws scripts mapping.txt locating.txt
git status
git commit -m "Initial UMI RGB-D dataset system"
git remote add origin <ROOT_REPOSITORY_URL>
git push -u origin main
```

Before committing, verify that `git status` does not list `data/`, `runtime/`,
`build/`, `install/`, `log/`, `.mcap`, `.osa`, or `.ply` files. Calibration YAML
may contain machine-specific measurements; decide explicitly whether it belongs
in the repository.

## Future Map Selection

No ORB-SLAM3 core change is required. The recommended follow-up is a map
registry such as `maps/<map_name>/atlas.osa`, plus `--map <map_name>` and
`--save-map <map_name>` options in the wrapper. The wrapper can copy the selected
settings YAML into `runtime/orbslam/` and replace only
`System.LoadAtlasFromFile` or `System.SaveAtlasToFile`. This makes map selection
explicit while preserving the existing SLAM core; it is intentionally not
implemented yet.
