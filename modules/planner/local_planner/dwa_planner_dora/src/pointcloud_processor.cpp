#include "dwa_planner/pointcloud_processor.hpp"
#include <iostream>
#include <vector>

namespace dwa_planner {

// =====================================================================
//  完整处理流程（优化版：单次遍历）
// =====================================================================
Eigen::MatrixXf PointCloudProcessor::process(const Eigen::MatrixXf& cloud)
{
    if (cloud.rows() == 0) {
        return cloud;
    }

    const float min_dist_sq = config_.min_distance * config_.min_distance;
    const float max_dist_sq = config_.max_distance * config_.max_distance;

    std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHash> voxel_map;
    voxel_map.reserve(cloud.rows() / 50);

    // 单次遍历：边降采样边过滤
    for (int i = 0; i < cloud.rows(); ++i) {
        float x = cloud(i, 0);
        float y = cloud(i, 1);
        float z = cloud(i, 2);

        // 高度过滤
        if (z < config_.z_min || z > config_.z_max) {
            continue;
        }

        // 距离过滤
        float dist_sq = x * x + y * y;
        if (dist_sq < min_dist_sq || dist_sq > max_dist_sq) {
            continue;
        }

        // 降采样：分配到体素
        VoxelKey key = pointToVoxel(x, y, z);
        voxel_map[key].push_back(i);
    }

    // 每个体素取平均值
    Eigen::MatrixXf result(voxel_map.size(), 3);
    int idx = 0;

    for (const auto& pair : voxel_map) {
        const auto& indices = pair.second;

        float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
        for (int i : indices) {
            sum_x += cloud(i, 0);
            sum_y += cloud(i, 1);
            sum_z += cloud(i, 2);
        }

        int count = indices.size();
        result(idx, 0) = sum_x / count;
        result(idx, 1) = sum_y / count;
        result(idx, 2) = sum_z / count;
        ++idx;
    }

    return result;
}

// =====================================================================
//  体素网格降采样
//  原理：将空间划分为体素网格，每个体素内的点取平均值
// =====================================================================
Eigen::MatrixXf PointCloudProcessor::downsample(const Eigen::MatrixXf& cloud) {
    if (cloud.rows() == 0) {
        return cloud;
    }

    // 使用哈希表存储每个体素内的点
    std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHash> voxel_map;

    // 将点分配到体素
    for (int i = 0; i < cloud.rows(); ++i) {
        VoxelKey key = pointToVoxel(cloud(i, 0), cloud(i, 1), cloud(i, 2));
        voxel_map[key].push_back(i);
    }

    // 每个体素取平均值
    Eigen::MatrixXf result(voxel_map.size(), 3);
    int idx = 0;

    for (const auto& pair : voxel_map) {
        const auto& indices = pair.second;

        float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
        for (int i : indices) {
            sum_x += cloud(i, 0);
            sum_y += cloud(i, 1);
            sum_z += cloud(i, 2);
        }

        int count = indices.size();
        result(idx, 0) = sum_x / count;
        result(idx, 1) = sum_y / count;
        result(idx, 2) = sum_z / count;
        ++idx;
    }

    return result;
}

// =====================================================================
//  高度过滤
//  保留 z ∈ [z_min, z_max] 的点
// =====================================================================
Eigen::MatrixXf PointCloudProcessor::filterHeight(const Eigen::MatrixXf& cloud) {
    if (cloud.rows() == 0) {
        return cloud;
    }

    // 统计满足条件的点数
    int valid_count = 0;
    for (int i = 0; i < cloud.rows(); ++i) {
        float z = cloud(i, 2);
        if (z >= config_.z_min && z <= config_.z_max) {
            ++valid_count;
        }
    }

    // 提取满足条件的点
    Eigen::MatrixXf result(valid_count, 3);
    int idx = 0;
    for (int i = 0; i < cloud.rows(); ++i) {
        float z = cloud(i, 2);
        if (z >= config_.z_min && z <= config_.z_max) {
            result.row(idx++) = cloud.row(i);
        }
    }

    return result;
}

// =====================================================================
//  距离过滤
//  保留距离在 [min_distance, max_distance] 范围内的点
//  注意：点云数据已经在机器人坐标系下，直接计算到原点的距离
// =====================================================================
Eigen::MatrixXf PointCloudProcessor::filterDistance(const Eigen::MatrixXf& cloud)
{
    if (cloud.rows() == 0) {
        return cloud;
    }

    const float min_dist_sq = config_.min_distance * config_.min_distance;
    const float max_dist_sq = config_.max_distance * config_.max_distance;

    // 统计满足条件的点数
    int valid_count = 0;
    for (int i = 0; i < cloud.rows(); ++i) {
        float x = cloud(i, 0);
        float y = cloud(i, 1);
        float dist_sq = x * x + y * y;

        if (dist_sq >= min_dist_sq && dist_sq <= max_dist_sq) {
            ++valid_count;
        }
    }

    // 提取满足条件的点
    Eigen::MatrixXf result(valid_count, 3);
    int idx = 0;
    for (int i = 0; i < cloud.rows(); ++i) {
        float x = cloud(i, 0);
        float y = cloud(i, 1);
        float dist_sq = x * x + y * y;

        if (dist_sq >= min_dist_sq && dist_sq <= max_dist_sq) {
            result.row(idx++) = cloud.row(i);
        }
    }

    return result;
}

// =====================================================================
//  辅助函数：点 → 体素键
// =====================================================================
PointCloudProcessor::VoxelKey PointCloudProcessor::pointToVoxel(
    float x, float y, float z) const
{
    return VoxelKey{
        static_cast<int>(std::floor(x / config_.voxel_size)),
        static_cast<int>(std::floor(y / config_.voxel_size)),
        static_cast<int>(std::floor(z / config_.voxel_size))
    };
}

} // namespace dwa_planner
