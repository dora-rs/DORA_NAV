# ourrosbag

A high-performance ROSbag player for the [dora](https://github.com/dora-rs/dora) dataflow runtime.  
Reads ROS 1 (`.bag`) and ROS 2 (`metadata.yaml`) bags, parses sensor messages through a C++ hot path, and publishes them as Arrow IPC batches — or records them to Parquet for offline inspection.

---

## Features

| Capability | What it does |
|---|---|
| **C++ hot path** | Image decoding and PointCloud2 parsing run in a compiled pybind11 module (`ourrosbag_cpp`). Falls back to pure Python automatically if the `.pyd`/`.so` is missing. |
| **Arrow output** | Messages are serialised as Arrow IPC record-batches and sent via `dora.Node.send_output()`. Measured **23× faster** than JSON encoding. |
| **Parquet recording** | `--record` flag dumps every message to per-topic, zstd-compressed Parquet files — **~8× smaller** than JSONL. Buffered writes (flush every 500 rows) keep memory bounded. |
| **Stats tracker** | Every run prints a session summary: total messages, throughput (msg/sec), per-type message count, data volume, and average parse latency. |
| **Inspector** | CLI tool that loads a recorded Parquet session and produces dark-themed matplotlib visualisations — IMU charts, odometry path, single image view, and an interactive image scrubber with play/pause + keyboard controls. |
| **Dual bag format** | Transparent support for ROS 1 v2 (`.bag`) and ROS 2 v3 (directory with `metadata.yaml`), auto-detected at load time. |
| **Config-driven** | Single YAML file controls bag path, topic filtering, playback speed, looping, time-range trimming, and output format. |

---

## Supported Message Types

| ROS message type | Parsed fields | C++ accelerated |
|---|---|:---:|
| `sensor_msgs/msg/Image` | width, height, encoding, pixel data | ✅ |
| `sensor_msgs/msg/PointCloud2` | x, y, z float arrays, n_points | ✅ |
| `sensor_msgs/msg/Imu` | orientation, angular velocity, linear acceleration | — |
| `nav_msgs/msg/Odometry` | position, orientation | — |
| `sensor_msgs/msg/NavSatFix` | lat, lon, alt, status | — |
| _any other type_ | raw string fallback | — |

---

## Quick Start

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure

Edit `configs/default.yml`:

```yaml
bag_path: "data/race_1.bag"
topics:
  - /camera/fisheye2/image_raw
  - /camera/odom/sample
  - /camera/imu
playback:
  speed: 10.0
  loop: false
  start_time: 0.0
  end_time: null          # null = play until end
output:
  format: arrow           # "arrow" or "json"
  prefix: rosbag
```

Leave `topics` empty (`[]`) to subscribe to **all** topics in the bag.

### 3. Discover topics

```bash
python main.py --discover
```

Prints every topic and its message type without playing.

### 4. Play

```bash
# standalone (dry-run, no dora runtime)
python main.py

# inside dora
dora start dataflow.yml
```

When running outside dora, the publisher prints message summaries to stdout.  
When running inside dora, messages are sent as Arrow IPC on the outputs defined in `dataflow.yml`:

```
image · imu · gps · odometry · pointcloud
```

### 5. Pause / Resume

During playback, press **`p`** to toggle pause and resume:

```
[ourrosbag] Press 'p' to pause/resume playback.
[Player] Paused. Press 'p' to resume.
[Player] Resumed.
```

- Pausing blocks the playback generator — no messages are published while paused.
- Resuming adjusts the wall-clock anchor so messages don't burst-fire to catch up.
- Works on both Windows (`msvcrt`) and Unix/macOS (`termios`).
- `Ctrl+C` still stops the entire run at any time.

### 6. Record to Parquet

```bash
python main.py --record
```

Creates per-topic Parquet files in `output/parquet_session/`:

```
output/parquet_session/
  image.parquet
  imu.parquet
  odometry.parquet
```

### 7. Inspect a recorded session

```bash
# IMU angular velocity & linear acceleration charts
python -m ourrosbag.inspector --imu

# Odometry 2D path (color-coded by time)
python -m ourrosbag.inspector --path

# Single image frame (default frame 0)
python -m ourrosbag.inspector --image --frame 42

# Interactive image scrubber (←/→ to step, Space to play/pause)
python -m ourrosbag.inspector --scrub

# Everything at once
python -m ourrosbag.inspector --all

# Custom session directory
python -m ourrosbag.inspector --session output/my_session --imu
```

---

## Architecture

```
main.py                          entry point / CLI
ourrosbag/
├── config.py                    YAML config → BagConfig dataclass
├── reader.py                    opens bag (v2/v3), filters topics, yields raw messages
├── parser.py                    dispatches per-type parsers, tracks stats, uses C++ when available
├── player.py                    wall-clock synchronised playback (speed, loop, time-range, pause/resume)
├── publisher.py                 Arrow IPC serialisation → dora send_output (or dry-run stdout)
├── recorder.py                  buffered Parquet writer (per-topic, zstd compression)
├── inspector.py                 matplotlib visualisations (IMU, path, image, scrubber)
├── stats.py                     per-type message count, throughput, parse latency tracker
├── ourrosbag_cpp.*.pyd/.so      compiled C++ extension (auto-loaded)
└── __init__.py                  public API exports
cpp/
├── parser.cpp                   pybind11 module: parse_pointcloud2(), decode_image()
└── CMakeLists.txt               build config (C++17, pybind11)
configs/
└── default.yml                  default playback configuration
dataflow.yml                     dora node definition
tests/                           unit tests (parser, player, publisher, reader)
examples/                        dora dataflow pipeline examples (navigation, SLAM)
docs/                            documentation
```

### Data flow

```
┌──────────┐     ┌────────┐     ┌────────┐     ┌───────────┐
│ BagReader│────▶│ Player │────▶│ Parser │────▶│ Publisher  │──▶ dora outputs
│ (v2/v3)  │     │ (sync) │     │ (C++)  │     │ (Arrow)   │
└──────────┘     └────────┘     └────────┘     └───────────┘
                                    │
                                    ▼
                              ┌──────────┐
                              │ Recorder │──▶ .parquet files
                              │ (zstd)   │
                              └──────────┘
                                    │
                                    ▼
                              ┌───────────┐
                              │ Inspector │──▶ matplotlib plots
                              └───────────┘
```

---

## Building the C++ Module

The C++ extension is **optional** — everything works in pure Python without it, just slower for images and point clouds.

```bash
cd cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release --target install
```

This compiles `ourrosbag_cpp` and copies the `.pyd`/`.so` into `ourrosbag/`.  
Requires: CMake ≥ 3.15, Python dev headers, pybind11.

---

## Running Tests

```bash
python -m pytest tests/ -v
```

All tests use `unittest` + mocks — no real bag files or dora runtime required.

Run a single module:

```bash
python -m pytest tests/test_parser.py -v
python -m pytest tests/test_player.py -v
python -m pytest tests/test_publisher.py -v
python -m pytest tests/test_reader.py -v
```

---

## Config Reference

| Key | Type | Default | Description |
|---|---|---|---|
| `bag_path` | string | `""` | Path to `.bag` file or ROS 2 bag directory |
| `topics` | list | `[]` | Topic filter. Empty = all topics |
| `playback.speed` | float | `1.0` | Playback speed multiplier (e.g. `10.0` = 10× realtime) |
| `playback.loop` | bool | `false` | Loop playback when bag ends |
| `playback.start_time` | float | `0.0` | Skip messages before this bag-time (seconds) |
| `playback.end_time` | float | `null` | Stop after this bag-time. `null` = play to end |
| `output.format` | string | `"arrow"` | `"arrow"` for Arrow IPC, `"json"` for JSON stdout |
| `output.prefix` | string | `"rosbag"` | Output ID prefix (reserved for future use) |

---

## Roadmap

| Step | Status | Description |
|:---:|:---:|---|
| 0 | ✅ | **C++ module wired** — pybind11 fast path for image decode + PointCloud2 parsing |
| 1 | ✅ | **Stats tracker** — per-type message count, throughput, avg parse latency |
| 2 | ✅ | **Record mode** — per-topic Parquet output (zstd, ~8× smaller than JSONL) |
| 3 | ✅ | **Arrow output** — IPC serialisation for dora (23× speedup proven over JSON) |
| 4 | ✅ | **Inspector** — IMU charts, odometry path, image viewer, interactive scrubber |
| 5 | ✅ | **ROSbag v3 support** — metadata.yaml validation, storage plugin detection (SQLite3/MCAP), duration + message count logging |
| 6 | ✅ | **Runtime pause/resume** — `threading.Event` gate in player, `p` key toggle, wall-clock correction on resume |
| 7 | ✅ | **Tests** — unit tests for parser, player, publisher, reader (all mock-based, no bag files needed) |
| 8 | ✅ | **Example pipelines** — navigation & SLAM dora dataflow examples |
| 9 | ✅ | **README** — this file |
