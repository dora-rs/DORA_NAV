# ROSbag Reader Node for DORA_NAV
### GSoC 2026 Project #9
**Author:** U.Kaushik Teja ([@kaushikteja26](https://github.com/kaushikteja26))

A complete ROSbag reader node for DORA_NAV that allows
replaying existing ROS2 datasets into the DORA dataflow
pipeline without requiring a physical robot or ROS2
installation.

## Features

- Read ROS2 bag files (v8/v9 format)
- Detect available topics automatically
- Multi-sensor parsing (IMU, GPS, Odometry, Image, PointCloud2)
- Timed playback with speed scaling (0.5x, 1x, 2x)
- Loop playback support
- DORA API integration with Apache Arrow
- YAML configuration support
- 3 example pipelines
- 11 unit tests — all passing

## Why This Matters
WITHOUT this node:

Researcher wants to test SLAM
-  needs expensive robot hardware
-  OR full ROS2 + Gazebo setup

WITH this node:
-  download any ROS2 dataset (e.g. KITTI)
-  run: python3 dora_rosbag_node.py
-  DORA gets real sensor data instantly!

## File Structure
apps/rosbag_reader/
├── rosbag_reader.py        # Topic inspection + sensor extraction
├── playback.py             # Timed playback with speed control
├── dora_rosbag_node.py     # Main DORA node (publishes via API)
├── config_reader.py        # YAML config based runner
├── config.yaml             # Example configuration
├── create_test_bag.py      # Test bag generator
├── printer_node.py         # Data printer node
├── slam_stub_node.py       # SLAM stub for testing
├── nav_stub_node.py        # Navigation stub for testing
├── test_rosbag_reader.py   # 11 unit tests
└── example_pipelines/
    ├── dataset_replay.yml      # Pipeline 1
    ├── slam_testing.yml        # Pipeline 2
    └── navigation_testing.yml  # Pipeline 3


## Installation
# Clone the repo
git clone https://github.com/dora-rs/DORA_NAV.git
cd DORA_NAV

# Create virtual environment
python3 -m venv ~/dora-env
source ~/dora-env/bin/activate

# Install dependencies
pip install dora-rs rosbags pyarrow pyyaml numpy

## Quick Start

### 1. Inspect a ROSbag file
python3 apps/rosbag_reader/rosbag_reader.py /path/to/bag/

### 2. Play a ROSbag file
# Normal speed
python3 apps/rosbag_reader/playback.py /path/to/bag/

# Double speed
python3 apps/rosbag_reader/playback.py /path/to/bag/ --speed 2.0

# Half speed with loop
python3 apps/rosbag_reader/playback.py /path/to/bag/ --speed 0.5 --loop --loops 3

### 3. Run as DORA node
BAG_PATH=/path/to/bag/ SPEED=1.0 LOOP=false \
python3 apps/rosbag_reader/dora_rosbag_node.py

### 4. Run with YAML config
# Edit config.yaml with your settings
python3 apps/rosbag_reader/config_reader.py config.yaml

## Configuration (config.yaml)
'''yaml
bag:
  path: /path/to/your/bag/   # ROSbag folder path
  speed: 1.0                  # 0.5=slow, 1.0=normal, 2.0=fast
  loop: false                 # loop playback
  loops: 1                    # number of loops

topics:
  - /imu/data                 # IMU sensor
  - /gps/fix                  # GPS sensor
  - /odom                     # Odometry
  - /camera/image_raw         # Camera
  - /pointcloud               # LiDAR PointCloud

output:
  format: arrow               # Apache Arrow for DORA
  verbose: true               # print to terminal
'''
## Supported Sensor Types

| Sensor | ROS Message Type | DORA Output |
| IMU | sensor_msgs/msg/Imu | `imu_data` |
| GPS | sensor_msgs/msg/NavSatFix | `gps_data` |
| Odometry | nav_msgs/msg/Odometry | `odom_data` |
| Image | sensor_msgs/msg/Image | `image_data` |
| PointCloud | sensor_msgs/msg/PointCloud2 | `pointcloud_data` |

## Example Pipelines

### Pipeline 1 — Dataset Replay
dora start apps/rosbag_reader/example_pipelines/dataset_replay.yml
ROSbag Reader - Printer Node

### Pipeline 2 — SLAM Testing
dora start apps/rosbag_reader/example_pipelines/slam_testing.yml
ROSbag Reader - SLAM Node - Printer Node
(LiDAR + IMU feeds into SLAM algorithm)

### Pipeline 3 — Navigation Testing
dora start apps/rosbag_reader/example_pipelines/navigation_testing.yml
ROSbag Reader - Navigation Node - Printer Node
(GPS + Odometry feeds into nav stack)


## Running Tests
cd apps/rosbag_reader
python3 test_rosbag_reader.py

Expected output:
Results: 11/11 passed
All tests passed!


## Compatible Datasets

This node works with any ROS2 bag file including:

| Dataset | Sensors | Use Case |
| KITTI | LiDAR, Camera, IMU, GPS | SLAM, Object Detection |
| Your own recording | Any ROS2 sensors | Custom testing |

### Download KITTI Sample Bag
wget https://urserver.kaist.ac.kr/publicdata/patchwork++/kitti_00_sample.bag

## Architecture
┌─────────────────────────────────────────┐
│           ROSbag File                   │
│  /imu/data, /gps/fix, /pointcloud...    │
└──────────────┬──────────────────────────┘
               │
 
┌─────────────────────────────────────────┐
│         dora_rosbag_node.py             │
│  1. Read messages with timestamps       │
│  2. Apply speed scaling (timed delay)   │
│  3. Convert ROS - Apache Arrow          │
│  4. Publish via DORA API                │
└──────┬──────────┬──────────┬────────────┘
       │          │          │
 
  imu_data    gps_data  pointcloud_data
       │          │          │
 
┌──────────┐ ┌─────────┐ ┌──────────────┐
│  SLAM    │ │   Nav   │ │  Perception  │
│  Node    │ │  Node   │ │    Node      │
└──────────┘ └─────────┘ └──────────────┘


## GSoC 2026 — Project #9

This is a proof of concept for GSoC 2026 Project #9:
ROSbag Reader Node for Dora.

**Mentors:** Jia Li, Zhen Tian

**PR:** https://github.com/dora-rs/DORA_NAV/pull/3
