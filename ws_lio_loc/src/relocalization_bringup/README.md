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

## Notes

- PCD saving is enabled in the default config (`mid360.yaml`). spark_fast_lio saves PCD files to `<spark_fast_lio_source>/PCD/` (hardcoded at compile time), not to this package's `pcd/` directory.
- The `bags/` directory is provided as a convenient location to store bag files for replay.
