//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_MEASURE_GROUP_H
#define LIGHTNING_MEASURE_GROUP_H

#include <deque>

#include "common/imu.h"
#include "common/odom.h"
#include "common/point_def.h"

namespace lightning {

/// 同步获取的消息
struct MeasureGroup {
    double timestamp_ = 0;  // scan的开始时间
    double lidar_begin_time_ = 0;
    double lidar_end_time_ = 0;

    std::deque<IMUPtr> imu_;    // 两个scan之间的IMU测量
    std::deque<OdomPtr> odom_;  // 两个scan之间的odom测量

    CloudPtr scan_raw_ = nullptr;         // 原始传感器的扫描数据
    CloudPtr scan_ = nullptr;             // 选点、距离过滤之后的数据
    CloudPtr scan_undist_ = nullptr;      // 去畸变后的数据
    CloudPtr scan_undist_raw_ = nullptr;  // 去畸变后的原始数据
};

}  // namespace lightning

#endif  // LIGHTNING_MEASURE_GROUP_H
