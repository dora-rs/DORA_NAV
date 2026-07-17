#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>
#include "types.hpp"

namespace dwa_planner {

// =====================================================================
//  点云处理器配置参数
// =====================================================================
struct PointCloudProcessorConfig {
    // 降采样参数
    float voxel_size = 0.15f;  // 体素网格大小 [m]（稍大以减少计算量）

    // 高度过滤参数
    float z_min = -0.3f;   // 最小高度 [m]
    float z_max = 0.2f;    // 最大高度 [m]

    // 距离过滤参数
    float min_distance = 0.05f;  // 最小距离 [m]（过滤机器人本体和地面反射点）
    float max_distance = 3.0f;  // 最大距离 [m]
};

// =====================================================================
//  点云处理器
//  功能：降采样 → 高度过滤 → 距离过滤
// =====================================================================
class PointCloudProcessor {
public:
    explicit PointCloudProcessor(const PointCloudProcessorConfig& config = PointCloudProcessorConfig{})
        : config_(config) {}

    // 完整处理流程：降采样 → 高度过滤 → 距离过滤
    Eigen::MatrixXf process(const Eigen::MatrixXf& cloud);

    // 单独的处理步骤（供调试使用）
    Eigen::MatrixXf downsample(const Eigen::MatrixXf& cloud);
    Eigen::MatrixXf filterHeight(const Eigen::MatrixXf& cloud);
    Eigen::MatrixXf filterDistance(const Eigen::MatrixXf& cloud);

private:
    // 体素网格哈希键
    struct VoxelKey {
        int x, y, z;

        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    // 哈希函数
    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey& k) const {
            return std::hash<int>()(k.x) ^
                   (std::hash<int>()(k.y) << 1) ^
                   (std::hash<int>()(k.z) << 2);
        }
    };

    // 将点转换为体素键
    VoxelKey pointToVoxel(float x, float y, float z) const;

    PointCloudProcessorConfig config_;
};

} // namespace dwa_planner
