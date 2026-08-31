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
| `waypoint_dropper.yaml` | `waypoint_dropper.launch.py` | Waypoint dropper: map, ground estimation, levels |
| `consolidate_map.yaml` | `consolidate_map` | Map consolidation settings |
| `glim_mid360/` | `run_glim_bag.py`, `run_glim_chunked.py` | GLIM config dir for Mid360 — Iridescence GUI + RViz publisher, outdoor-tuned (default) |
| `glim_mid360_rviz/` | (same, `--rviz-only`) | RViz publisher only — no native GUI window |
| `glim_mid360_headless/` | (same, `--headless`) | No viewers at all — dump file only |
| `glim_mid360_indoor/` | (same, `--indoor`) | Short-range preprocessing, finer voxels, tighter loop search — for indoor mapping |

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

### Recording a bag

To record a bag for later offline mapping (FAST-LIO2, SPARK, or GLIM), record the raw Livox driver topics. Launch the lidar driver in one terminal:

```bash
ros2 launch relocalization_bringup mid360.launch.py
```

Then start recording in a second terminal (from `ws_lio_loc/`):

```bash
ros2 bag record /livox/lidar /livox/imu -o src/relocalization_bringup/bags/<recording_name>
```

Stop with `Ctrl-C` when done. The two topics are:

| Topic | Type | Rate |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 10 Hz (driver `publish_freq`) |
| `/livox/imu` | `sensor_msgs/msg/Imu` | ~200 Hz |

Notes:

- Record the **driver topics only** — don't record LIO output topics (`/Odometry`, `/cloud_registered`, TF, etc.) if the goal is to re-run a LIO backend on the bag later; recording them wastes space and replaying TF can conflict with the live pipeline.
- `/livox/lidar` is a Livox `CustomMsg`, not a `PointCloud2`. Any machine playing back the bag needs `livox_ros_driver2` sourced (e.g. `source ~/repos/ws_livox/install/setup.bash`) so the message type resolves. The GLIM helper scripts relay CustomMsg → `/livox/points` (`PointCloud2`) automatically during replay.
- Before recording, sanity-check that both topics are alive: `ros2 topic hz /livox/lidar` and `ros2 topic hz /livox/imu`.
- Do **not** launch mapping or relocalization while recording (it isn't needed, and PCD saving will churn the disk). The bag only needs the driver running.

To replay, see [Replay a bag file](#replay-a-bag-file) below (mapping with `use_sim_time:=true` + `ros2 bag play --clock`), or feed the bag to GLIM via `run_glim_bag.py` / `run_glim_chunked.py` (see the GLIM section).

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

The tool only supplies x/y/yaw; the height comes from the map, as a low percentile of the z values in a box around x/y. On a multi-storey map that box spans every floor and the percentile lands on the lowest one, so first click the floor you are on with the **Publish Point** tool. `scan_lock` snaps the click to the nearest floor it detected in the map and confines its height search to that storey — see [Multi-storey maps: levels](#multi-storey-maps-levels). The same click crops `pose_prompt`'s fine cloud to that storey, so one click sets up both. Level parameters live under `initial_guess:` in `scan_lock.yaml` (`use_clicked_point_level`, `level_band_below`, `level_band_above`, `level_snap_tolerance`) and under `pose_prompt:` in `pose_prompt.yaml` (`level_crop_enable`, `level_band_below`, `level_band_above`).

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

### Dropping waypoints

`waypoint_dropper` loads a PCD map, lets you lay out a route of waypoints in RViz, and writes them to `outputs/waypoints_<timestamp>.yaml` on `Ctrl-C`.

```bash
ros2 launch relocalization_bringup waypoint_dropper.launch.py
```

- **2D Pose Estimate** drops a waypoint. The tool supplies x/y/yaw; the height is estimated from the map, as a low percentile of the z values in a box around x/y (so furniture and walls don't lift it off the floor).
- **Left-click** a marker to set the insertion point; **right-click** for the menu (Insert Before / Insert After / Snap to Active Level / Delete).
- Insertion point can also be set directly: `ros2 param set /waypoint_dropper waypoint_dropper.insert_index <N>` (`-1` = append).
- Load an existing route to edit with `waypoint_dropper.input_waypoint_file` (relative paths resolve under `outputs/`).

#### Multi-storey maps: levels

In a single-storey map the height under (x, y) is unambiguous. In a building it isn't: the column above a point holds every floor, ceiling and slab, and a low percentile always returns the *lowest* storey. A **level** fixes that — a named horizontal plane at height z, where only map points within `[z - band_below, z + band_above]` count as that floor. Ground estimation inside that band cannot fall through to the storey below.

Workflow:

1. **Start the dropper.** It peak-picks a z-histogram of the map, logs the floor heights it found, and creates a level per floor (`L0` upwards from the bottom). No level is active yet, so behaviour is unchanged until you pick one.
2. **Publish Point** on the floor you want. That level becomes active, and `map_cloud_level` is republished with just that storey — turn the full `map_cloud` display off to see it cleanly.
3. **2D Pose Estimate** as usual. Waypoints now land on the active floor and record which level they belong to.
4. `Ctrl-C` saves the levels alongside the waypoints.

Two things worth knowing:

- **Click from an orbit view, not top-down.** *Publish Point* raycasts onto the topmost surface under the cursor, which from above is a ceiling. Orbit into the storey, or click on the already-sliced `map_cloud_level`. You can also skip clicking entirely: `ros2 param set /waypoint_dropper levels.active L1` (and `""` to go back to a whole-column search).
- **Clicking a table or a step is fine.** The clicked height snaps to the nearest known floor within `levels.merge_tolerance` (1 m by default), which covers everything you might realistically click while aiming at a floor.

Waypoints on other levels are dimmed rather than hidden (set `levels.hide_other_levels: true` to hide them), and their labels carry the level name. **Snap to Active Level** on a marker's right-click menu moves an existing waypoint onto the active floor — useful for repairing a route saved before levels existed.

The output gains a `levels:` block and a `level:` key per waypoint; every other field is unchanged, so existing consumers keep working:

```yaml
levels:
  - name: L0
    z: 0.0
  - name: L1
    z: 3.3
waypoints:
  - x: 5.0
    y: 5.0
    z: 3.3
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
    level: L1
```

Key parameters (`config/waypoint_dropper.yaml`, under `levels:`):

| Parameter | Default | Description |
|---|---|---|
| `enable` | `true` | `false` always searches the full z column (pre-levels behaviour) |
| `band_below` | `0.25` | How far below a level's height still counts as its floor. Must stay inside the slab between a storey's ceiling and the floor above it (~0.3 m), or the ceiling below wins the search |
| `band_above` | `2.5` | How far above; keep it under the floor-to-ceiling height |
| `merge_tolerance` | `1.0` | A click within this of a known level selects it instead of making a new one |
| `auto_detect` | `true` | Seed the level list from the map's z-histogram at startup |
| `detect_min_fraction` | `0.02` | Share of map points in a 10 cm slab needed to count as a floor |
| `hide_other_levels` | `false` | Hide (rather than dim) markers that aren't on the active level |
| `active` | `""` | Active level at startup; settable at runtime |
| `names` / `heights` | (unset) | Levels known ahead of time, instead of relying on detection |

### Cropping a map to a waypoint route

A map covering a whole campus is mostly wasted on a route that visits one building. `crop_map` trims a PCD down to the region a waypoint file actually visits:

```bash
# From ws_lio_loc/
ros2 run relocalization_bringup crop_map \
    waypoints_20260830_120000.yaml \
    caltech_0/map.pcd \
    src/relocalization_bringup/pcd/caltech_0/map_cropped.pcd
```

The kept region is the bounding box of every waypoint, padded by **40 m in x/y** and **5 m in z** by default. The x/y footprint is squared up, so the padding is *at least* the requested buffer on every side (the longer axis gets exactly it, the shorter one gets more); `--rect` pads the bounding box directly instead. z is padded but never squared.

Both input paths are used as given if they exist, and otherwise resolve under the package's `outputs/` and `pcd/` directories respectively — so a bare waypoint filename and a bare map name both work, matching `waypoint_dropper`'s conventions. `$ENV` variables are expanded in both.

Every field of the input survives: the cloud is filtered as a raw `PCLPointCloud2`, so a `PointXYZINormal` map from `consolidate_map` comes out `PointXYZINormal`, and a `PointXYZI` map from `ply_to_pcd` comes out `PointXYZI`.

| Option | Default | Description |
|---|---|---|
| `--xy-buffer M` | `40` | Minimum padding (m) around the waypoints in x and y |
| `--z-buffer M` | `5` | Padding (m) below the lowest and above the highest waypoint |
| `--rect` | (off) | Pad the bounding box by exactly `--xy-buffer`; don't square the footprint |
| `--dry-run` | (off) | Report the crop box and surviving point count without writing |
| `--ascii` | (off) | Write ASCII PCD instead of binary |
| `--copy-to PATH` | — | Also copy the result to PATH (a directory keeps the basename) |

The tool prints the waypoint bounds, the map's own bounds and the crop box before writing, which is the quickest way to catch a waypoint file paired with the wrong map — if the two bounds barely overlap you'll see it immediately, and a crop that keeps nothing is an error rather than an empty file.

Cropping also sharpens [level detection](#multi-storey-maps-levels): `levels.detect_min_fraction` is a share of *all* map points, so trimming away far-off terrain makes a building's floors stand out in the z-histogram.

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

**1. Record (or already have) a bag.** The bag should contain `/livox/lidar` (CustomMsg) and `/livox/imu` — see [Recording a bag](#recording-a-bag) above. The helper scripts handle the CustomMsg → PointCloud2 relay automatically when replaying.

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
ros2 run glim_ros offline_viewer src/relocalization_bringup/glim_maps/my_run
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

#### Config variants

Four sibling config directories ship in [config/](config/), each pre-tuned for a different scenario. Both runner scripts accept matching flags to pick the variant:

| Variant | Config dir | Iridescence GUI | RViz topics | Tuning | Pick via |
|---|---|---|---|---|---|
| **default** | `glim_mid360/` | ✓ | ✓ | outdoor / mid-range | (no flag) |
| **rviz-only** | `glim_mid360_rviz/` | ✗ | ✓ | outdoor / mid-range | `--rviz-only` |
| **headless** | `glim_mid360_headless/` | ✗ | ✗ | outdoor / mid-range | `--headless` |
| **indoor** | `glim_mid360_indoor/` | ✓ | ✓ | indoor (short-range, fine voxels) | `--indoor` |

The four flags are mutually exclusive — combining e.g. `--indoor --headless` would need a 5th compound dir, which isn't built. If you need a combination, pass `--config /path/to/your/dir` explicitly and the flags drop out.

```bash
# Default (outdoor, full viz): opens Iridescence + publishes RViz topics
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

# Indoor: short-range preprocessing, finer voxels, tighter loop search.
# Use for buildings / labs / corridors. Defaults to GUI + RViz viz.
ros2 run relocalization_bringup run_glim_bag.py \
    --bag /path/to/bag --output-dir src/relocalization_bringup/glim_maps/run --indoor
```

All four flags apply to `run_glim_chunked.py` too. They're a no-op if you also pass `--config /some/explicit/path` — that path wins, since you've taken responsibility for picking a config dir.

##### What the indoor variant changes

| File | Param | Default (outdoor) | Indoor | Why |
|---|---|---|---|---|
| `config_preprocess.json` | `distance_far_thresh` | 100.0 m | **50.0 m** | Rooms return at 5–20 m, but corridors don't: side walls, floor and ceiling all run parallel to travel and constrain nothing about distance covered — the end wall is the only geometry that does. A 30 m cut hid it for most of a 40–60 m hallway (along-corridor scale drift). The Mid360 reaches 40 m @ 10% reflectivity, so painted walls return well past 30 m |
| `config_preprocess.json` | `downsample_resolution` | 1.0 m | **0.25 m** | Preserve doorway/furniture detail (1 m voxels lose chairs entirely) |
| `config_sub_mapping_*.json` | `submap_voxel_resolution` | 0.5 m | **0.25 m** | Match indoor scale |
| `config_sub_mapping_*.json` | `keyframe_update_interval_trans` | 1.0 m / 0.1 m | **0.5 m / 0.05 m** | ⚠️ **No-op as configured.** `SubMapping` consumes this only in the `DISPLACEMENT` branch; `keyframe_update_strategy` is `OVERLAP`, so keyframe insertion is decided by `max_keyframe_overlap` (0.6, same in both variants). Overlap is scale-relative so it adapts on its own; to actually densify keyframes indoors, raise `max_keyframe_overlap`. The `_passthrough` value is doubly inert — `config.json` selects `config_sub_mapping_gpu.json`, so that module never loads |
| `config_sub_mapping_gpu/cpu.json` | `keyframe_voxel_resolution` | 0.25 m | **0.1 m** | Finer registration target |
| `config_global_mapping_gpu.json` | `submap_voxel_resolution_{max,dmin,dmax}` | 1.0 / 5.0 / 20.0 m | **0.5 / 2.0 / 10.0 m** | Adaptive voxel scaling kicks in at indoor distances |
| `config_global_mapping_*.json` | `max_implicit_loop_distance` | 100.0 m | **30.0 m** | Indoor loops happen at short range; wider search wastes pose-graph factors |

Odometry (`config_odometry_*.json`), IMU noise (`config_sensors.json`), the topic / viewer choice (`config_ros.json`), and the optimizer settings are unchanged between variants — only what's listed above differs.

Two values in the shared `glim_mid360/config_odometry_gpu.json` were retuned for indoor work but, being shared, now apply to **every** variant:

| Param | Was | Now | Why |
|---|---|---|---|
| `full_connection_window_size` | 2 | **4** | Stair climbing pitches the sensor hard between frames. GLIM's own docs call for 3–5 on aggressive motion; connecting the latest pose to more of the recent window is what keeps odometry from breaking on stairs. Costs GPU time, no accuracy downside outdoors |
| `num_threads` | 2 | **8** | 2 threads bottlenecks offline bag replay on a many-core host (`preprocess.num_threads` in the indoor config was raised to 8 for the same reason) |

**Watch item for multi-floor stairwells:** `max_implicit_loop_distance` of 30 m spans roughly eight floors, and a stairwell is vertically self-similar. `GlobalMapping::find_overlapping_submaps` gates on Euclidean distance and then `min_implicit_loop_overlap` (0.2), which is loose for self-similar geometry. Overlap is evaluated at the *current* pose estimate, so well-separated floors score near zero — the risk only bites once Z drift brings them together, which is exactly when a false closure folds the building. If a multi-floor map comes out with floors collapsed, raise `min_implicit_loop_overlap` to ~0.35 before changing anything else.

#### Symlink layout

Under the hood each variant dir overrides only the files it needs and symlinks the rest back to canonical `glim_mid360/`:

- `glim_mid360_rviz/` / `glim_mid360_headless/` → real `config_ros.json` (viewer modules differ), everything else symlinked
- `glim_mid360_indoor/` → real `config_preprocess.json`, `config_sub_mapping_{cpu,gpu,passthrough}.json`, `config_global_mapping_{cpu,gpu}.json` (6 files), everything else symlinked

Editing the canonical config (e.g. switching `config_global_mapping` between `_cpu` / `_gpu` in `config.json`, or bumping `T_lidar_imu` in `config_sensors.json`) automatically propagates to all variants — there's no fork to keep in sync.

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
