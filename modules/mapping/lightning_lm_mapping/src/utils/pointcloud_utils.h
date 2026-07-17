//
// Created by xiang on 25-4-22.
//

#ifndef LIGHTNING_POINTCLOUD_UTILS_H
#define LIGHTNING_POINTCLOUD_UTILS_H

#include "common/point_def.h"

namespace lightning {

/// 体素滤波
CloudPtr VoxelGrid(CloudPtr cloud, float voxel_size = 0.05);

/// 移除地面
void RemoveGround(CloudPtr cloud, float z_min = 0.5);

}  // namespace lightning

#endif  // LIGHTNING_POINTCLOUD_UTILS_H
