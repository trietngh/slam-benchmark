# Frame Saver ROS2 Package

A ROS2 package designed to capture and save images from ROS2 topics. It supports both asynchronous (capture all) and synchronous (capture time-aligned) modes, with configurable frame rates.

## Dual Operation Modes
  - **Async Mode**: Saves images from all specified topics independently, optionally rate-limited.
  - **Sync Mode**: Saves images from all topics roughly synchronized by time (within a timeout), driven by a master timer.

## Dependencies

- ROS2 (tested on Jazzy)
- Python 3
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `opencv-python`

## Installation

Assuming you are in your ROS2 workspace (e.g., `ros2_ws`):

```bash
cd ros2_ws/src
# Clone or copy the package here
git clone <repository_url> frame_saver  # if applicable

cd ..
colcon build --packages-select frame_saver
source install/setup.bash
```

## Usage

### 1. using Launch File (Recommended)

The launch file loads default parameters from `config/frame_saver.yaml`.

```bash
ros2 launch frame_saver frame_saver.launch.py
```

**Override parameters via command line:**

```bash
ros2 launch frame_saver frame_saver.launch.py \
    topics:='["/camera/rgb", "/camera/depth"]' \
    save_dir:="/data/dataset" \
    mode:="sync" \
    save_rate:=10.0
```

### 2. Running the Node Directly

```bash
ros2 run frame_saver frame_saver_node --ros-args \
    -p topics:='["/image_raw"]' \
    -p save_dir:="/tmp/test"
```

## Configuration

Default configuration is stored in `config/frame_saver.yaml`:

```yaml
/**:
  ros__parameters:
    image_types: ["rgb", "depth"]  # List of image groups
    rgb:
      topics: ["/camera/image_raw"] # Topics for this group
      encoding: "bgr8"              # Encoding for this group (e.g., bgr8)
    depth:
      topics: ["/camera/depth/image_raw"]
      encoding: "32FC1"             # Depth format. Saved as .tiff files.
    save_rate: 0.0                 # Hz. 0.0 = save all (async) or max speed
    save_dir: "."                  # Base directory for saving runs
    mode: "async"                  # "async" or "sync"
    timeout: 0.1                   # Max time difference (s) for sync mode validation
    use_sim_time: false            # Set to true if running with bag files/sim
```

### Parameter Details

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `image_types` | string[] | `[]` | List of image group names. Each group must have `<group>.topics` and `<group>.encoding` defined. |
| `save_rate`    | double   | `0.0`     | **Async**: Rate limit per topic (Hz). 0=No limit.<br>**Sync**: Frequency of the sync timer (Hz).  |
| `save_dir` | string | `.` | Absolute or relative path to the base save directory. |
| `mode`         | string   | `async`   | `async`: Topics processed independently.<br>`sync`: Topics synchronized via buffer.               |
| `timeout`      | double   | `0.1`     | **Sync only**: Max allowed age (seconds) of a message relative to `now()` to be considered valid. |
| `use_sim_time` | bool     | `false`   | Synchronize with simulation/bag time.                                                             |

## Output Structure

The package creates a structured hierarchy:

```text
save_dir/
└── 260211_001/             # Run directory: YYMMDD_NNN
    ├── 1/                  # Topic 1 (first in list)
    │   ├── 1700000000.123456789.png
    │   └── ...
    ├── 2/                  # Topic 2
    │   └── ...
    └── ...
```

## Troubleshooting

- **"Base directory is not writable"**: Check permissions of `save_dir`.
- **Images not saving in Sync mode**:
  - Check if `timeout` is too small for your system latency.
  - Ensure all topics are publishing.
  - Verify `use_sim_time` is set correctly if using bag files.

## License

MIT

