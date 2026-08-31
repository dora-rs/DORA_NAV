// arrow_schemas.h
// Canonical Apache Arrow C++ schemas for DORA_NAV inter-node communication.
// Replaces the undocumented raw byte wire format used across the pipeline.
//
// GSoC 2026 — Project #7: SLAM and Localization Package
// Author: Leonardo Galgano (https://github.com/ilGalghi)

#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// ArrowPointCloud
// Replaces the custom 16-byte header + pointer arithmetic in bytes2cloud().
// Each point: x, y, z, intensity (Float32), packed at 16 bytes/point.
// Header metadata (per-batch, not per-point):
//   seq    — UInt32, sequence counter
//   stamp  — UInt64, microseconds
// frame_id is always "rslidar" by convention and is NOT on the wire.
// ---------------------------------------------------------------------------
struct ArrowPointCloud {
    float x;
    float y;
    float z;
    float intensity;
    // Per-batch metadata (carried in the Arrow RecordBatch metadata map):
    //   "seq"   -> uint32_t
    //   "stamp" -> uint64_t
};

// ---------------------------------------------------------------------------
// ArrowPose2D
// Replaces raw Pose2D_h / slampose_h / GroundTruthPose (all float).
// Fields: x, y (meters), theta (radians)
// NOTE: No timestamp in the original structs; if needed, add it in the
//       Arrow metadata map rather than changing the struct layout.
// ---------------------------------------------------------------------------
struct ArrowPose2D {
    float x;
    float y;
    float theta;
};

// ---------------------------------------------------------------------------
// ArrowCurPose
// Replaces CurPose_h (double precision) used by planning / routing nodes.
// Fields: x, y (meters), theta (radians), s, d (Frenet coordinates)
// ---------------------------------------------------------------------------
struct ArrowCurPose {
    double x;
    double y;
    double theta;
    double s;
    double d;
};

// ---------------------------------------------------------------------------
// ArrowIMU
// Replaces reinterpret_cast<canslam::imu_msg_h *>(data).
// Layout matches imu_msg_h: stamp (double, seconds) followed by two
// Vector3 groups (linear_acceleration, angular_velocity) as 3×float each.
// ---------------------------------------------------------------------------
struct ArrowIMU {
    double stamp;
    float acc_x, acc_y, acc_z;        // linear_acceleration (m/s²)
    float gyro_x, gyro_y, gyro_z;     // angular_velocity   (rad/s)
};

// ---------------------------------------------------------------------------
// ArrowOccupancyGrid
// 2D occupancy grid for Map Server, SLAM, and Localization operators.
// Compatible with ROS2 nav2 PGM+YAML map format.
// Grid cells are transmitted as a separate Arrow Int8Array (row-major):
//   0 = free, 100 = occupied, -1 = unknown
// This struct carries only the metadata; the cell data lives in the
// accompanying Arrow array within the same RecordBatch.
// ---------------------------------------------------------------------------
struct ArrowOccupancyGrid {
    uint32_t width;       // pixels
    uint32_t height;      // pixels
    float resolution;     // m/px
    float origin_x;       // meters
    float origin_y;       // meters
};