# relocalization_bringup

Bringup package for mapping and relocalization with two LIO backends: `fast_lio` (FAST-LIO2) and `spark_fast_lio` (SPARK Fast-LIO2).

## Package Structure

```
relocalization_bringup/
├── bags/          # Bag files for replay
├── pcd/           # Consolidated maps from consolidate_map
├── config/        # Sensor and scan_lock configuration files
└── launch/        # Launch files
```

## Configuration Files

| File | Used by | Description |
|---|---|---|
| `mid360.yaml` | `mapping.launch.py` | FAST-LIO2 mapping config (MID360) |
| `mid360_spark.yaml` | `mapping_spark.launch.py` | SPARK Fast-LIO2 mapping config (MID360) |
| `mid360_relocalization.yaml` | `relocalization.launch.py` | FAST-LIO2 relocalization config (MID360) |
| `mid360_relocalization_spark.yaml` | `relocalization_spark.launch.py` | SPARK Fast-LIO2 relocalization config (MID360) |
| `hesaiJT128.yaml` | `mapping.launch.py`, `relocalization.launch.py` | FAST-LIO2 config for Hesai JT128 |
| `hesaiJT128_driver.yaml` | `hesaiJT128.launch.py` | Hesai ROS driver config (UDP, correction file paths) |
| `scan_lock.yaml` | `relocalization.launch.py` | scan_lock config for FAST-LIO2 pipeline |
| `scan_lock_spark.yaml` | `relocalization_spark.launch.py` | scan_lock config for SPARK pipeline |
| `consolidate_map.yaml` | `consolidate_map` | Map consolidation settings |

## Lidar Drivers

The lidar driver must be launched in a separate terminal before running mapping or relocalization.

**Livox MID360:**

```bash
ros2 launch relocalization_bringup mid360.launch.py
```

**Hesai JT128:**

```bash
ros2 launch relocalization_bringup hesaiJT128.launch.py
```

> Before running the Hesai driver, verify that `correction_file_path` and `firetimes_path` in `config/hesaiJT128_driver.yaml` point to the correct calibration files on your machine.

## Usage

### Mapping

First launch the lidar driver (see above), then in a second terminal:

**MID360 + FAST-LIO2:**

```bash
ros2 launch relocalization_bringup mapping.launch.py
```

**MID360 + SPARK Fast-LIO2:**

```bash
ros2 launch relocalization_bringup mapping_spark.launch.py
```

**Hesai JT128 + FAST-LIO2:**

```bash
ros2 launch relocalization_bringup mapping.launch.py config_file:=hesaiJT128.yaml
```

To launch without RViz:

```bash
ros2 launch relocalization_bringup mapping.launch.py rviz:=false
```

#### Replay a bag file

In one terminal, launch mapping with sim time:

```bash
ros2 launch relocalization_bringup mapping.launch.py use_sim_time:=true
```

In a second terminal, play the bag with `--clock`:

```bash
ros2 bag play bags/<your_bag> --clock
```

#### Mapping Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock (for bag replay) |
| `config_path` | `<package>/config` | Path to config directory |
| `config_file` | `mid360.yaml` / `mid360_spark.yaml` | Config file name |
| `rviz` | `true` | Launch RViz |
| `rviz_cfg` | `fastlio.rviz` | RViz config file path |

### Relocalization

First launch the lidar driver (see above), then in a second terminal:

Relocalization launches a LIO backend, a body frame TF broadcaster, and `scan_lock` for localization within a prior pointcloud map.

**MID360 + FAST-LIO2:**

```bash
ros2 launch relocalization_bringup relocalization.launch.py robot_name:=go2
```

**MID360 + SPARK Fast-LIO2:**

```bash
ros2 launch relocalization_bringup relocalization_spark.launch.py robot_name:=go2
```

**Hesai JT128 + FAST-LIO2:**

```bash
ros2 launch relocalization_bringup relocalization.launch.py robot_name:=go2 lio_config_file:=hesaiJT128.yaml
```

In RViz, use the **2D Pose Estimate** tool to provide an initial pose guess. `scan_lock` will refine the estimate via ICP registration.

#### Relocalization Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock (for bag replay) |
| `robot_name` | `default` | Robot platform (`default`, `g1`, `go2`, `stick`) |
| `lio_config_path` | `<package>/config` | LIO config directory |
| `lio_config_file` | `mid360_relocalization.yaml` / `mid360_relocalization_spark.yaml` | LIO config file |
| `scan_lock_config_path` | `<package>/config` | scan_lock config directory |
| `scan_lock_config_file` | `scan_lock.yaml` / `scan_lock_spark.yaml` | scan_lock config file |
| `rviz` | `true` | Launch RViz |
| `rviz_cfg` | `scanlock.rviz` | RViz config file path |

### Consolidate PCD Map

After mapping, consolidate saved PCD scans into a single voxel-filtered map:

```bash
# Run from ws_lio_loc/
ros2 run relocalization_bringup consolidate_map --lio fast_lio
ros2 run relocalization_bringup consolidate_map --lio spark
```

The `--lio` argument selects which backend's PCD directory to read from. Output maps are written to `relocalization_bringup/pcd/` and copied to `scan_lock/pcd/`.

To verify setup without processing:

```bash
ros2 run relocalization_bringup consolidate_map --lio fast_lio --dry-run
```

#### Configuration (`consolidate_map.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `output_name` | `"lidar_map"` | Folder name prefix (timestamp appended) |
| `intermediate_filter_enable` | `false` | Enable voxel filter on each scan during accumulation |
| `intermediate_voxel_size` | `0.01` | Leaf size (m) for intermediate filter |
| `final_filter_enable` | `true` | Enable voxel filter on the consolidated cloud |
| `voxel_size` | `0.05` | Leaf size (m) for final filter |
| `delete_source_files` | `true` | Delete source PCD files after consolidation |

#### CLI Options

| Option | Description |
|---|---|
| `--lio {fast_lio,spark}` | Select LIO backend PCD source directory |
| `--config PATH` | Path to config YAML |
| `--source PATH` | Override source PCD directory |
| `--output PATH` | Override output base directory |
| `--dry-run` | Validate setup without processing |

#### Visualizing the Map

```bash
sudo apt install pcl-tools  # one-time install
pcl_viewer src/relocalization_bringup/pcd/lidar_map_*/map.pcd
```

## Notes

- PCD saving is enabled in the default mapping configs. FAST-LIO2 saves to `FAST_LIO/PCD/` and SPARK Fast-LIO2 saves to `spark_fast_lio/PCD/`.
- The `bags/` directory is provided as a convenient location to store bag files for replay.
- Body frame TF broadcasting is only launched during relocalization, not mapping.
