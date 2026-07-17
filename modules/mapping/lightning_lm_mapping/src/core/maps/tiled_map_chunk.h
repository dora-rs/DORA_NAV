//
// Created by xiang on 23-2-7.
//

#ifndef LIGHTNING_TILED_MAP_CHUNK_H
#define LIGHTNING_TILED_MAP_CHUNK_H

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace lightning {

/**
 * 一个地图区块
 *
 * TODO thread-safe
 */
struct MapChunk {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapChunk() = default;
    MapChunk(int id, Vec2i grid, std::string filename)
        : id_(id), grid_(std::move(grid)), filename_(std::move(filename)), cloud_(new PointCloudType()) {
        cloud_->reserve(50000);
    }

    ~MapChunk() = default;

    /**
     * 增加一个点
     * @param pt
     */
    void AddPoint(const PointType& pt);

    /// 加载地图
    void LoadCloud();

    /// 卸载本区块
    void Unload();

    int id_ = 0;                  // 该区块ID
    Vec2i grid_ = Vec2i::Zero();  // 网格中心
    std::string filename_;        // 地图文件名
    bool loaded_ = false;         // 该区域是否已经被载入

    std::mutex data_mutex_;     // 数据锁
    CloudPtr cloud_ = nullptr;  // 该区块的点云
};

}  // namespace lightning

#endif  // TEAPORT_TILED_MAP_CHUNK_H
