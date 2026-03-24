# relocalization_bringup

Bringup package for relocalization with `spark_fast_lio`.

## Package Structure

```
relocalization_bringup/
├── bags/          # Bag files for replay
├── pcd/           # PCD output from spark_fast_lio
├── config/        # Sensor configuration files
└── launch/        # Launch files
```

## Usage

### Launch spark_fast_lio

```bash
ros2 launch relocalization_bringup mapping.launch.py
```

To launch without rviz:

```bash
ros2 launch relocalization_bringup mapping.launch.py rviz:=false
```

### Replay a bag file

In one terminal, launch spark_fast_lio with sim time enabled:

```bash
ros2 launch relocalization_bringup mapping.launch.py use_sim_time:=true
```

In a second terminal, play the bag with `--clock` so it publishes the `/clock` topic:

```bash
ros2 bag play bags/<your_bag> --clock
```

### Launch Arguments

| Argument       | Default          | Description                          |
|----------------|------------------|--------------------------------------|
| `use_sim_time` | `false`          | Use simulation clock (for bag replay)|
| `config_path`  | `<package>/config` | Path to config directory           |
| `config_file`  | `mid360.yaml`    | Config file name                     |
| `rviz`         | `true`           | Launch rviz                          |
| `rviz_cfg`     | `fastlio.rviz`   | RViz config file path                |

### Consolidate PCD Map

After running spark_fast_lio, consolidate the saved PCD scans into a single voxel-filtered map:

```bash
cd ~/repos/ws_spark
./install/relocalization_bringup/lib/relocalization_bringup/consolidate_map
```

This uses the default config at `src/relocalization_bringup/config/consolidate_map.yaml`. To specify a different config:

```bash
./install/relocalization_bringup/lib/relocalization_bringup/consolidate_map --config path/to/config.yaml
```

The tool will:
1. Create a timestamped folder in `src/relocalization_bringup/pcd/` (e.g., `lidar_map_2026-03-24_135658/`)
2. Copy all PCD files from `spark_fast_lio/PCD/` into that folder
3. Load each scan, optionally apply an intermediate voxel filter, and accumulate into a single cloud
4. Optionally apply a final voxel filter
5. Save the result as `map.pcd`
6. Optionally delete the source PCD files

To verify setup without processing:

```bash
./install/relocalization_bringup/lib/relocalization_bringup/consolidate_map --dry-run
```

#### Configuration (`consolidate_map.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `output_name` | `"lidar_map"` | Folder name prefix (timestamp appended) |
| `intermediate_filter_enable` | `true` | Enable voxel filter on each scan during accumulation |
| `intermediate_voxel_size` | `0.01` | Leaf size (m) for intermediate filter |
| `final_filter_enable` | `true` | Enable voxel filter on the consolidated cloud |
| `voxel_size` | `0.05` | Leaf size (m) for final filter |
| `delete_source_files` | `true` | Delete source PCD files after consolidation |
| `verbose` | `true` | Print detailed progress |
| `source_pcd_dir` | `src/spark-fast-lio/spark_fast_lio/PCD` | Source PCD directory (relative to CWD) |
| `output_base_dir` | `src/relocalization_bringup/pcd` | Output directory (relative to CWD) |

#### CLI Options

| Option | Description |
|---|---|
| `--config PATH` | Path to config YAML (default: `src/relocalization_bringup/config/consolidate_map.yaml`) |
| `--source PATH` | Override source PCD directory |
| `--output PATH` | Override output base directory |
| `--dry-run` | Validate setup without processing |

#### Visualizing the Map

```bash
sudo apt install pcl-tools  # one-time install
pcl_viewer src/relocalization_bringup/pcd/lidar_map_*/map.pcd
```

## Notes

- PCD saving is enabled in the default config (`mid360.yaml`). spark_fast_lio saves PCD files to `<spark_fast_lio_source>/PCD/` (hardcoded at compile time), not to this package's `pcd/` directory.
- The `bags/` directory is provided as a convenient location to store bag files for replay.
