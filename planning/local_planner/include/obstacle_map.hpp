#pragma once
#include <vector>
#include <cstdint>

/// A 2D point obstacle in the robot's local frame
struct Obstacle {
    float x;  // metres, robot-frame
    float y;  // metres, robot-frame
};

/// Converts raw LiDAR bytes (from mujoco_sim/pointcloud) into a list of 2D obstacles.
/// Each point in the raw stream is: [seq(4B)] [stamp(8B)] [pad(4B)] then N×[x,y,z,intensity](16B each)
/// Only points within max_range and below max_height are kept.
class ObstacleMap {
public:
    ObstacleMap() = default;

    /// Parse raw DORA pointcloud bytes and extract obstacle positions.
    /// @param data   raw byte pointer from dora_input
    /// @param size   number of POINTS (not bytes) — (data_len - 16) / 16
    /// @param max_range   ignore points farther than this (metres)
    /// @param max_height  ignore points higher than this (metres, z-filter)
    void update(const char* data, int32_t size,
                float max_range = 20.0f, float max_height = 0.5f);

    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
    void clear() { obstacles_.clear(); }

private:
    std::vector<Obstacle> obstacles_;
};
