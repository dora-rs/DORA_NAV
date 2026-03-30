#include "obstacle_map.hpp"
#include <cmath>
#include <cstring>
#include <cstdio>

void ObstacleMap::update(const char* data, int32_t size,
                         float max_range, float max_height)
{
    obstacles_.clear();
    if (size <= 0 || data == nullptr) return;

    // Raw format from mujoco_sim: 16-byte header, then N × 16-byte points
    // Each point: [x(4B)][y(4B)][z(4B)][intensity(4B)]
    for (int32_t i = 0; i < size; ++i) {
        const char* ptr = data + 16 + i * 16;
        float x, y, z;
        std::memcpy(&x, ptr,     sizeof(float));
        std::memcpy(&y, ptr + 4, sizeof(float));
        std::memcpy(&z, ptr + 8, sizeof(float));

        // Filter: ignore ground and ceiling, ignore far points
        if (z > max_height || z < -0.1f) continue;
        float dist = std::sqrt(x * x + y * y);
        // Ignore points closer than 0.45m (the robot's own chassis/wheels)
        if (dist > max_range || dist < 0.45f) continue;

        obstacles_.push_back({x, y});
    }
}
