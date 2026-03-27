# LIO-Localization

A ROS2 workspace for LiDAR-Inertial Odometry (LIO) based pointcloud map creation and localization within a fixed pointcloud map. Built on [SPARK Fast-LIO2](https://github.com/MIT-SPARK/spark-fast-lio) with support for Livox, Velodyne, and Ouster LiDARs.

## Prerequisites

- **ROS2 Humble** (full desktop install)
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

### Upstream FAST_LIO

To run the upstream (unmodified) FAST_LIO with the Mid360, source the `lidar_odom_ws` and point to the config in this repo:

```bash
source ~/lidar_odom_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_path:=/home/$USER/repos/LIO-Localization/ws_lio_loc/src/relocalization_bringup/config \
  config_file:=mid360_fastlio.yaml
```

### Upstream spark-fast-lio

To run the upstream (unmodified) spark-fast-lio with the Mid360, source `spark_ws` only — do **not** source the local `ws_lio_loc` workspace, as the package names will conflict:

```bash
source ~/spark_ws/install/setup.bash
ros2 launch spark_fast_lio mapping_mit_campus.launch.yaml \
  config_path:=/home/$USER/repos/LIO-Localization/ws_lio_loc/src/relocalization_bringup/config/mid360_spark_upstream.yaml
```

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

## Docker

A Docker-based development environment is provided with two variants: **GPU** (NVIDIA runtime) and **No-GPU**. The container includes ROS2 Humble, Livox SDK2, the Livox ROS2 driver, and all PCL dependencies pre-installed.

### Prerequisites

- [Docker Engine](https://docs.docker.com/engine/install/) and [Docker Compose](https://docs.docker.com/compose/install/)
- For GPU support: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- For GUI forwarding (RViz, pcl_viewer): X11 display server on the host

### Environment Setup

The compose files read variables from `docker/.env`. Before building, set `LIO_ROOT` to the absolute path of this repository:

```bash
# Add to docker/.env
LIO_ROOT=/absolute/path/to/LIO-Localization
```

The default `.env` also sets `USER`, `UID`, and `GID` — adjust these if your host user doesn't match the defaults.

### Option 1: VS Code Dev Container (Recommended)

1. Install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension in VS Code.
2. Open this repository in VS Code.
3. Press `Ctrl+Shift+P` → **Dev Containers: Reopen in Container**.
4. Select one of the available configurations:
   - **LIO GPU Dev Container** — uses `docker-compose.yml` with NVIDIA runtime
   - **LIO No-GPU Dev Container** — uses `docker-compose-no-gpu.yml`
5. VS Code will build the image (first time only), start the container, and attach with recommended extensions pre-installed.

### Option 2: Terminal

**Build the image:**

```bash
cd docker
docker compose build                  # GPU
docker compose -f docker-compose-no-gpu.yml build  # No-GPU
```

**Start the container:**

```bash
# Allow X11 forwarding for GUI apps
xhost +local:docker

# GPU
cd docker
docker compose up -d
docker compose exec lio_localization bash

# No-GPU
cd docker
docker compose -f docker-compose-no-gpu.yml up -d
docker compose -f docker-compose-no-gpu.yml exec lio_localization bash
```

**Stop the container:**

```bash
cd docker
docker compose down                   # GPU
docker compose -f docker-compose-no-gpu.yml down   # No-GPU
```

### Building the Workspace Inside the Container

Once inside the container, build the ROS2 workspace as usual:

```bash
cd $LIO_ROOT/ws_lio_loc
source /opt/ros/humble/setup.bash
source ~/livox_driver_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## tmux

A quick reference for common tmux operations. All commands start with the **prefix**: `Ctrl+b`

### Starting and Stopping

```bash
tmux                           # Create new session (unnamed)
tmux new-session -s name       # Create named session
tmux kill-session -t name      # Kill a session
tmux kill-server               # Kill all sessions
```

### Attaching and Detaching

```bash
tmux list-sessions             # List all sessions
tmux attach-session -t name    # Attach to session
```

Once inside tmux:
- `Prefix d` — detach from session (session keeps running)
- `Prefix :kill-session` — kill current session

### Splitting Panes

- `Prefix "` — split horizontally (top/bottom)
- `Prefix %` — split vertically (left/right)
- `Prefix x` — close current pane (will prompt y/n)
- `Prefix arrow key` — navigate between panes
- `Prefix z` — toggle zoom (fullscreen) on current pane

### Other Basics

- `Prefix c` — create new window
- `Prefix n` / `Prefix p` — next/previous window
- `Prefix [` — enter scroll/copy mode (exit with `q`)
- `Prefix :` — enter command mode
