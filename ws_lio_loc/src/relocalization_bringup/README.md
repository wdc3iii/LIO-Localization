# relocalization_bringup

Bringup package for mapping and relocalization with two LIO backends: `fast_lio` (FAST-LIO2) and `spark_fast_lio` (SPARK Fast-LIO2).

## Package Structure

```
relocalization_bringup/
├── bags/          # Bag files for replay
├── pcd/           # Consolidated maps from consolidate_map
├── glim_maps/     # GLIM session dumps and exported maps
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
| `glim_mid360/` | `run_glim_bag.py`, `run_glim_chunked.py` | GLIM config dir for Mid360 — Iridescence GUI + RViz publisher (default) |
| `glim_mid360_rviz/` | (same, `--rviz-only`) | RViz publisher only — no native GUI window |
| `glim_mid360_headless/` | (same, `--headless`) | No viewers at all — dump file only |

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

### GLIM Mapping (optional, desktop-only)

[GLIM](https://koide3.github.io/glim/) is an optional third LIO/SLAM backend, useful when FAST-LIO2 struggles with very long bags or large environments. Install it by setting `WITH_GLIM_CUDA=true` (or `WITH_GLIM=true` for the CPU build) in `docker/.env` before building the image — see the top-level README's Docker section. The helper scripts and configs live in this package and are built/installed unconditionally; only the GLIM runtime is gated.

Three helper executables ship via `ros2 run relocalization_bringup`:

| Script | Purpose |
|---|---|
| `run_glim_bag.py` | Map a single bag in one shot (no chunking) |
| `run_glim_chunked.py` | Map a long bag in time-bounded chunks (for VRAM-bounded GPU runs) |
| `ply_to_pcd` | Convert GLIM's offline_viewer PLY export to a binary PCD for `scan_lock_node` |

A Mid360-specific GLIM config dir lives at [config/glim_mid360/](config/glim_mid360/). Customizations from stock GLIM:
- `config_ros.json` — points/IMU topics set to `/livox/points` and `/livox/imu`
- `config_sensors.json` — `T_lidar_imu = [-0.011, -0.02329, 0.04412, 0, 0, 0, 1]`
- `config.json` — global mapping defaults to the CPU variant (safer for long bags; flip the comment to switch to GPU)

#### End-to-end: bag → map.pcd → scan_lock

The full GLIM workflow takes a recorded bag and produces a `map.pcd` in `scan_lock/pcd/` ready for relocalization. Four steps:

**1. Record (or already have) a bag.** Bag should publish `/livox/lidar` (CustomMsg) and `/livox/imu`:

```bash
ros2 launch relocalization_bringup mid360.launch.py    # T1: lidar driver
ros2 bag record /livox/lidar /livox/imu -o bags/my_recording   # T2: record
```

The helper scripts handle the CustomMsg → PointCloud2 relay automatically when replaying.

**2. Run GLIM on the bag.** Two variants depending on bag length:

```bash
# Whole-bag (single shot). Best for bags ≲ 15 min on a 4080 with GPU mapping,
# or anything length on CPU global mapping.
ros2 run relocalization_bringup run_glim_bag.py \
    --bag /path/to/bag \
    --output-dir src/relocalization_bringup/glim_maps/my_run

# Chunked (long bags). Splits into ~15 min chunks with 60 s overlap; each
# chunk gets its own dump so VRAM stays bounded.
ros2 run relocalization_bringup run_glim_chunked.py \
    --bag /path/to/bag \
    --output-dir src/relocalization_bringup/glim_maps/my_run_chunks \
    --chunk-duration 900 --overlap 60
```

Both scripts spawn the CustomMsg relay, the `glim_rosnode`, and `ros2 bag play` themselves. They expect the calling shell to have sourced `/opt/ros/humble/setup.bash`, the Livox driver workspace, and (for runtime) `~/glim_ws/install/setup.bash`. Inside the Docker container with `WITH_GLIM_CUDA=true`, all three are added to `~/.bashrc` automatically — just open a shell and run.

On `Ctrl-C` (or natural end of bag), the script SIGINTs GLIM so it can write its dump (`graph.bin` + per-submap PCDs + an unmerged session) to `--output-dir`. **Never `kill -9`** — only graceful shutdown triggers the save.

**3. Export a single PLY from GLIM's offline_viewer.** GLIM dumps a session, not a finished map. Open the session and export:

```bash
ros2 run glim_ros offline_viewer --ros-args -p init_dump_path:=src/relocalization_bringup/glim_maps/my_run
```

In the viewer GUI: `File → Export → Points` and save as e.g. `src/relocalization_bringup/glim_maps/my_run/map.ply`. The viewer writes PLY regardless of extension, so a `.pcd` extension here will silently be PLY-format and fail downstream — name it `.ply`.

For chunked runs, either merge sessions inside `offline_viewer` first (`File → Merge`) and then export, or export each chunk separately and align them outside GLIM (see GLIM's [docs/merge.md](https://github.com/koide3/glim/blob/master/docs/merge.md)).

**4. Convert PLY → PCD and copy into scan_lock.** This is the part that closes the loop with the rest of this repo:

```bash
ros2 run relocalization_bringup ply_to_pcd \
    src/relocalization_bringup/glim_maps/my_run/map.ply \
    src/relocalization_bringup/glim_maps/my_run/map.pcd \
    --voxel 0.05 \
    --copy-to src/scan_lock/pcd/
```

That writes a binary PCD as `PointXYZI` (16 B/pt — half the size of `PointXYZINormal`; `scan_lock_node` loads it just the same, zero-padding the absent normal/curvature fields it never reads anyway), voxel-downsamples at 0.05 m (matching `consolidate_map.yaml`'s default), reports point-count and file-size compression, and copies the result into `src/scan_lock/pcd/map.pcd`. From there, `relocalization.launch.py` (or `relocalization_spark.launch.py`) can lock onto it. Pass `--xyzin` if you specifically need `PointXYZINormal` output to match `consolidate_map`'s layout.

`--copy-to` mirrors `consolidate_map.yaml`'s `copy_dir` field: if the argument is an existing directory, the PCD is copied into it with its original basename; otherwise the argument is treated as the target file path. Drop the flag entirely if you don't want the copy.

Other useful `ply_to_pcd` options:

```bash
ply_to_pcd in.ply out.pcd --voxel 0.1       # coarser downsampling (smaller map, less detail)
ply_to_pcd in.ply out.pcd --no-voxel        # no downsampling, full resolution
ply_to_pcd in.ply out.pcd --ascii           # write ASCII PCD (debugging only — 3x larger)
```

#### Viewer variants

GLIM has two viewer extension modules: `libstandard_viewer.so` (native Iridescence window) and `librviz_viewer.so` (publishes ROS topics for RViz). Three sibling config directories pre-select different combinations, picked via flags on the runner scripts:

| Variant | Config dir | Iridescence GUI | RViz topics | Pick via |
|---|---|---|---|---|
| **default** | `glim_mid360/` | ✓ | ✓ | (no flag) |
| **rviz-only** | `glim_mid360_rviz/` | ✗ | ✓ | `--rviz-only` |
| **headless** | `glim_mid360_headless/` | ✗ | ✗ | `--headless` |

```bash
# Default: opens the Iridescence window + publishes RViz topics
ros2 run relocalization_bringup run_glim_bag.py \
    --bag /path/to/bag --output-dir src/relocalization_bringup/glim_maps/run

# RViz-only: useful inside the docker container or over SSH where the
# native viewer is awkward but you still want to see what's happening.
# Launch your own RViz against /glim/* topics in another terminal.
ros2 run relocalization_bringup run_glim_bag.py \
    --bag /path/to/bag --output-dir src/relocalization_bringup/glim_maps/run --rviz-only

# Headless: no viewer state, only the dump on disk. Cheapest CPU-wise,
# safe over SSH with no display, fine for CI / batch reprocessing.
ros2 run relocalization_bringup run_glim_bag.py \
    --bag /path/to/bag --output-dir src/relocalization_bringup/glim_maps/run --headless
```

Same three flags apply to `run_glim_chunked.py`. The flags are a no-op if you also pass `--config /some/explicit/path` — that path wins, since you've taken responsibility for picking a config dir.

Under the hood the variant dirs just symlink to the default's sub-configs (`config_sensors.json`, `config_global_mapping_*.json`, …) and override only `config_ros.json`. Editing the default config (e.g. switching between `_cpu`/`_gpu` global mapping) automatically propagates to all three variants — there's no fork to keep in sync.

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
