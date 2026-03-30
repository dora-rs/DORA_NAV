# DORA_NAV



DORA_NAV is an open-source autonomous navigation framework built on the [DORA](https://github.com/dora-rs/dora) dataflow orchestration framework. It supports differential-drive, omnidirectional, and Ackermann-type mobile robot chassis. The framework integrates sensor drivers, mapping, localization, perception, planning, and control into a complete point-to-point navigation pipeline.



Directory Structure

```
~/dora          dora repository source code
~/dora_project  dora projects (e.g. ~/dora_project/dora_nav)
```

![Rerun Navigation Demo](images/dora_nav1.gif)

The Rerun viewer displays the navigation pipeline in real-time:
- **Red points** — Pre-built PCD point cloud map
- **Cyan line** — Global waypoint path
- **Blue line** — Planned local trajectory (from routing planner)
- **Green points** — Live LiDAR pointcloud (transformed to map frame)
- **Yellow box** — Robot body with heading arrow
- **Orange trail** — Robot trajectory history

---

## Architecture

The system is organized as a DORA dataflow graph where each node runs as an independent process communicating through the DORA runtime.

```
┌─────────────────────────────────────────────────────────────────────┐
│                        DORA Runtime (v0.3.12)                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐                                                   │
│  │  Sensor Input │   tick (10ms)                                    │
│  │  ────────────│──────────────┐                                   │
│  │  mujoco_sim  │              │                                    │
│  │  (or real    │    ┌─────────▼──────────┐                        │
│  │   LiDAR+IMU) │    │   pointcloud       │                        │
│  │              ├────►   imu_msg           │                        │
│  │              │    │   ground_truth_pose │                        │
│  └──┬───────┬───┘    └──┬──────────┬───────┘                        │
│     │       │           │          │                                │
│     │  SteeringCmd      │     ┌────▼─────────────────┐             │
│     │  TrqBreCmd        │     │  hdl_localization     │             │
│     │   (from control)  │     │  (NDT + UKF)          │             │
│     │                   │     │  ─────────────────    │             │
│     │                   │     │  IN: pointcloud,      │             │
│     │                   │     │      imu_msg          │             │
│     │                   │     │  OUT: cur_pose        │             │
│     │                   │     └────────┬──────────────┘             │
│     │                   │              │                            │
│  ┌──▼───────────┐  ┌───▼──────┐  ┌────▼──────────────────┐        │
│  │  Rerun       │  │ pub_road │  │ road_lane_publisher   │        │
│  │  Visualizer  │  │ (static  │  │ (pose + lane → frenet)│        │
│  │  ──────────  │  │  lanes)  │  │                       │        │
│  │  pointcloud  │  └───┬──────┘  └────────┬──────────────┘        │
│  │  raw_path    │      │                  │                        │
│  │  cur_pose    │      │    road_lane     │  cur_pose_all          │
│  └──────────────┘      │                  │                        │
│                   ┌────▼──────────────────▼───────────────┐        │
│                   │  task_pub_node ──► routing_planning    │        │
│                   │  (road attrs)     (A* + Frenet)       │        │
│                   │                   ────────────────     │        │
│                   │                   OUT: raw_path,       │        │
│                   │                        Request         │        │
│                   └──────┬─────────────────┬──────────────┘        │
│                          │                 │                        │
│                   ┌──────▼──────┐   ┌──────▼──────┐                │
│                   │ lat_control  │   │ lon_control  │               │
│                   │ (Pure       │   │ (PID speed   │               │
│                   │  Pursuit)   │   │  control)    │               │
│                   │ OUT:        │   │ OUT:         │               │
│                   │ SteeringCmd │   │ TrqBreCmd    │               │
│                   └─────────────┘   └──────────────┘               │
│                          │                 │                        │
│                          └────────┬────────┘                        │
│                                   │  (fed back to mujoco_sim       │
│                                   │   or real chassis)              │
│                                   ▼                                 │
│                          ┌────────────────┐                        │
│                          │  Vehicle/Sim   │                        │
│                          └────────────────┘                        │
└─────────────────────────────────────────────────────────────────────┘
```

### Node Summary

| Node | Algorithm | Key Libraries | Rate | Description |
|------|-----------|---------------|------|-------------|
| **mujoco_sim** | MuJoCo physics + raycasting | MuJoCo 3.5+, OpenMP | 1kHz sim, 10Hz LiDAR | Simulated LiDAR, IMU, ground truth pose |
| **hdl_localization** | NDT scan matching + UKF | PCL, Eigen3, NDT-OMP | Real-time | Pose estimation against pre-built PCD map |
| **pub_road** | Static publisher | DORA C API | 5Hz | Publishes pre-loaded road lane geometry |
| **road_lane_publisher** | Coordinate transform | Eigen3 | Event-driven | Transforms lanes to vehicle-local Frenet frame |
| **task_pub_node** | File reader | DORA C API | 50Hz | Publishes road attributes (speed limits, stops) |
| **routing_planning** | A* + Frenet planning | Eigen3, nlohmann/json | Event-driven | Generates reference trajectory and speed requests |
| **lat_controller** | Pure Pursuit | Eigen3 | 50Hz | Computes steering angle from path |
| **lon_controller** | PID | DORA C API | Event-driven | Computes torque/brake from speed request |
| **rerun** | Rerun streaming | Rerun SDK, PCL, Eigen3 | 10Hz | 3D visualization of full pipeline state |

### Shared Data Structures (`include/`)

```cpp
// SlamPose.h — Localization output
struct Pose2D_h { float x, y, theta; };

// Planning.h — Planning output
struct Request_h {
    uint8_t reques_type;   // FORWARD(0), BACK(1), STOP(2), AEB(3)
    float run_speed;
    float stop_distance;
    float aeb_distance;
};

// Localization.h — Full pose with Frenet coordinates
struct CurPose_h {
    double x, y, theta;   // Cartesian pose
    double s, d;           // Frenet coordinates along lane
};
```

---

## Directory Structure

```
dora-nav/
├── build_all.sh                  # Build all nodes
├── dataflow_full_sim.yml         # Full simulation pipeline (run this)
├── run.yml                       # Real hardware pipeline
├── Waypoints.txt                 # Global path waypoints
├── cmake/                        # Shared CMake configuration
│   └── dora_config.cmake
├── include/                      # Shared C/C++ headers
│   ├── SlamPose.h
│   ├── Planning.h
│   ├── Localization.h
│   └── imu_msg.h
├── data/
│   ├── map.pcd                   # Pre-built point cloud map
│   └── path/trajectory.txt       # Recorded trajectory
├── images/
│   └── localization.png          # Rerun visualization screenshot
├── simulation/mujoco_bridge/     # MuJoCo physics simulation
│   ├── src/mujoco_sim_bridge.cpp
│   └── models/robot_warehouse.xml
├── localization/dora-hdl_localization/  # NDT+UKF localization
├── mapping/ndt_mapping/          # Offline NDT map building
├── map/
│   ├── pub_road/                 # Static lane publisher
│   └── road_line_publisher/      # Lane-to-Frenet transformer
├── planning/
│   ├── mission_planning/task_pub/ # Road attribute publisher
│   └── routing_planning/          # A* + Frenet path planner
├── control/vehicle_control/
│   ├── lat_controller/           # Pure Pursuit steering
│   └── lon_controller/           # PID speed control
├── rerun/                        # Rerun 3D visualization
│   ├── src/points_to_rerun.cpp
│   └── data/map.pcd
├── driver/                       # Sensor drivers (LiDAR, IMU, GNSS)
├── peception/                    # Perception (ground filter, clustering, YOLO)
└── vehicle/                      # Chassis drivers (Adora series)
```

---

## Deployment Guide (Simulation Demo)

### Prerequisites

| Dependency | Version | Install |
|------------|---------|---------|
| Ubuntu | 22.04+ | - |
| CMake | 3.16+ | `sudo apt install cmake` |
| Clang/GCC | C++17 | `sudo apt install clang build-essential` |
| PCL | Latest | `sudo apt install libpcl-dev` |
| Eigen3 | 3.x | `sudo apt install libeigen3-dev` |
| Boost | Latest | `sudo apt install libboost-all-dev` |
| yaml-cpp | Latest | `sudo apt install libyaml-cpp-dev` |
| Python3-dev | 3.8+ | `sudo apt install python3-dev` |
| MuJoCo | 3.5+ | `pip install mujoco` |
| Rust/Cargo | Latest | `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \| sh` |

### Step 1: Install DORA Framework

```bash
# Install DORA CLI
pip install dora-rs-cli

# Clone and build DORA C API library
cd ~/Public
git clone https://github.com/dora-rs/dora.git
cd dora
cargo build -p dora-node-api-c --release
# This produces: ~/Public/dora/target/release/libdora_node_api_c.a
# Headers are in: ~/Public/dora/apis/c/node/
```

### Step 2: Install Rerun Viewer

```bash
# Install Rerun SDK (C++ library)
pip install rerun-sdk

# Download and build the C++ SDK
cd /tmp
wget https://github.com/rerun-io/rerun/releases/download/0.29.2/rerun_cpp_sdk.zip
mkdir -p rerun_cpp_sdk && cd rerun_cpp_sdk
unzip ../rerun_cpp_sdk.zip
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Step 3: Clone and Build DORA_NAV

```bash
cd ~/Public
git clone https://github.com/RuPingCen/DORA_NAV.git dora-nav
cd dora-nav
```

Build all pipeline nodes:

```bash
./build_all.sh
```

Or build individual nodes manually:

```bash
# Example: build the localization node
cd localization/dora-hdl_localization
mkdir -p build && cd build
cmake .. \
  -DCMAKE_PREFIX_PATH="$HOME/Public/dora-nav/cmake" \
  -DDORA_INCLUDE_DIR="$HOME/Public/dora/apis/c/node" \
  -DDORA_OPERATOR_DIR="$HOME/Public/dora/apis/c/operator" \
  -DDORA_LIB_PATH="$HOME/Public/dora/target/release/libdora_node_api_c.a" \
  -DDORA_NAV_ROOT="$HOME/Public/dora-nav"
make -j$(nproc)
```

Build the MuJoCo simulation bridge:

```bash
cd ~/Public/dora-nav/simulation/mujoco_bridge
mkdir -p build && cd build
cmake .. \
  -DCMAKE_PREFIX_PATH="$HOME/Public/dora-nav/cmake" \
  -DDORA_INCLUDE_DIR="$HOME/Public/dora/apis/c/node" \
  -DDORA_OPERATOR_DIR="$HOME/Public/dora/apis/c/operator" \
  -DDORA_LIB_PATH="$HOME/Public/dora/target/release/libdora_node_api_c.a" \
  -DDORA_NAV_ROOT="$HOME/Public/dora-nav"
make -j$(nproc)
```

Build the Rerun visualizer:

```bash
cd ~/Public/dora-nav/rerun
mkdir -p build && cd build
cmake .. \
  -DCMAKE_PREFIX_PATH="$HOME/Public/dora-nav/cmake" \
  -DDORA_INCLUDE_DIR="$HOME/Public/dora/apis/c/node" \
  -DDORA_OPERATOR_DIR="$HOME/Public/dora/apis/c/operator" \
  -DDORA_LIB_PATH="$HOME/Public/dora/target/release/libdora_node_api_c.a" \
  -DDORA_NAV_ROOT="$HOME/Public/dora-nav"
make -j$(nproc)
```

### Step 4: Verify Builds

Confirm all required binaries exist:

```bash
ls -la simulation/mujoco_bridge/build/mujoco_sim_bridge
ls -la localization/dora-hdl_localization/build/hdl_localization
ls -la map/pub_road/build/pubroad
ls -la map/road_line_publisher/build/road_lane_publisher_node
ls -la planning/mission_planning/task_pub/build/task_pub_node
ls -la planning/routing_planning/build/routing_planning_node
ls -la control/vehicle_control/lat_controller/build/lat_controller_node
ls -la control/vehicle_control/lon_controller/build/lon_controller_node
ls -la rerun/build/to_rerun
```

### Step 5: Run the Simulation Demo

**Important**: Run all `dora start` commands from the project root directory (`dora-nav/`) so that relative data file paths resolve correctly.

```bash
cd ~/Public/dora-nav

# Start the DORA coordinator and daemon
dora up

# Launch the full simulation pipeline with Rerun visualization
dora start dataflow_full_sim.yml --attach
```

The Rerun viewer window will open automatically, showing:
- The pre-built point cloud map (red)
- The global waypoint path (cyan)
- Live simulated LiDAR scans (green) transformed by robot pose
- The robot body (yellow box) with heading arrow
- The robot's trajectory trail (orange)
- The planned local path (blue)

### Step 6: Stop the Demo

Press `Ctrl+C` in the terminal, then:

```bash
dora destroy
```

---

## Dataflow Configurations

| Dataflow YAML | Description | Sensor Source |
|---------------|-------------|---------------|
| `dataflow_full_sim.yml` | Full pipeline with ground truth pose | MuJoCo simulation |
| `simulation/mujoco_bridge/dataflow_mujoco_sim.yml` | Full pipeline with NDT localization | MuJoCo simulation |
| `run.yml` | Full pipeline for real robot | Physical LiDAR + IMU |

---

## References

- [DORA Framework](https://github.com/dora-rs/dora) — Dataflow-Oriented Robotic Architecture
- [Rerun](https://www.rerun.io/) — Visualization SDK for robotics
- [MuJoCo](https://mujoco.org/) — Physics simulation engine
- [PCL](https://pointclouds.org/) — Point Cloud Library
- [HDL Localization](https://github.com/koide3/hdl_localization) — 3D LiDAR localization using NDT
- DORA and Rerun installation guide: [doc/dora_and_rerun_install.md](doc/dora_and_rerun_install.md)
