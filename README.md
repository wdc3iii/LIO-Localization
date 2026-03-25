# LIO-Localization

A ROS2 workspace for LiDAR-Inertial Odometry (LIO) based pointcloud map creation and localization within a fixed pointcloud map. Built on [SPARK Fast-LIO2](https://github.com/MIT-SPARK/spark-fast-lio) with support for Livox, Velodyne, and Ouster LiDARs.

## Prerequisites

- **ROS2 Foxy** (full desktop install)
- **Livox SDK2** — [Installation guide](https://github.com/Livox-SDK/Livox-SDK2)
- **Livox ROS2 Driver** — [Installation guide](https://github.com/Livox-SDK/livox_ros_driver2)

## Installation

Clone with submodules:

```bash
git clone --recursive https://github.com/wdc3iii/LIO-Localization.git
cd LIO-Localization
```

Install ROS2 dependencies via rosdep:

```bash
cd ws_lio_loc
rosdep install --from-paths src --ignore-src -r -y
```

This handles `libpcl-dev`, `pcl_conversions`, `yaml-cpp`, and all other ROS2 dependencies automatically.

Source the Livox ROS2 Driver workspace, then build:

```bash
source /path/to/livox_ros_driver2/install/setup.bash
cd ws_lio_loc
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> **Note:** `spark_fast_lio` already forces Release mode with `-O3` in its CMakeLists, but passing `--cmake-args -DCMAKE_BUILD_TYPE=Release` ensures the other packages (e.g. `relocalization_bringup`) also build optimized — important for PCL-heavy operations.

## Usage

### Recording offline

To record LiDAR data offline, simply source the `livox_ros_driver2`, launch the LiDAR unit, and record a rosbag:
```bash
source /path/to/livox_ros_driver2/install/setup.bash
ros2 launch livox_ros_driver2 mid360.launch.py
```

```bash
# Run from the repository root (LIO-Localization/)
source /path/to/livox_ros_driver2/install/setup.bash
ros2 bag record /livox/lidar /livox/imu -o ws_lio_loc/src/relocalization_bringup/bags
```

### Mapping

Launch the mapping node with a Livox Mid360:

```bash
ros2 launch relocalization_bringup mapping.launch.py
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Set `true` for bag replay |
| `config_file` | `mid360.yaml` | Sensor configuration file |
| `rviz` | `true` | Launch RViz visualization |

PCD scans are saved to `spark_fast_lio/PCD/` during mapping.

### Pointcloud Consolidation

After mapping, consolidate the individual PCD scans into a single map:

```bash
ros2 run relocalization_bringup consolidate_map
```

Configuration is in `relocalization_bringup/config/consolidate_map.yaml` — controls voxel filter sizes, output directory, and whether to delete source files after consolidation. Output maps are written to `relocalization_bringup/pcd/`.

To visualize a map, make sure `pcl_viewer` is installed `sudo apt install pcl-tools`, and run 

```bash
pcl_viewer src/relocalization_bringup/pcd/<name>/map.pcd
```

## Packages

| Package | Description |
|---|---|
| **spark_fast_lio** | SPARK Fast-LIO2 mapping engine (LiDAR-inertial SLAM) |
| **scan_lock** | Scan matching for localization within a prior map |
| **relocalization_bringup** | Launch files, sensor configs, and map consolidation tools |
