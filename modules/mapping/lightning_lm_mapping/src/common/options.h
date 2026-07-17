//
// Created by xiang on 24-4-8.
//

#pragma once
#ifndef LIGHTNING_MODULE_OPTIONS_H
#define LIGHTNING_MODULE_OPTIONS_H

#include <string>

#include <common/constant.h>
#include <common/eigen_types.h>

// #include <rclcpp/rclcpp.hpp>

/// 配置参数
namespace lightning {

namespace debug {

/// debug and save
extern bool flg_exit;     // ctrl-c中断
extern bool flg_pause;    // 暂停
extern bool flg_next;     // 暂停后，放行单个消息(单步调试)
extern float play_speed;  // 播放速度

inline void SigHandle(int sig) {
    debug::flg_exit = true;
    // rclcpp::shutdown();
}

}  // namespace debug

namespace lo {
extern float lidar_time_interval;           // 雷达的扫描时间
extern bool use_dr_rotation;                // 姿态预测是否使用dr
extern int relative_pose_check_cloud_size;  // pose校验相对帧数,不可小于2
extern double parking_speed;                // 驻车判别速度阈值
extern double parking_count;                // 驻车判别心跳次数
extern double kf_pose_check_distance;       // 触发校验距离阈值
extern double kf_pose_check_angle;          // 触发校验角度阈值
extern double pose2d_roll_limit;            // 当前帧触发2d roll角度阈值
extern double pose2d_pitch_limit;           // 当前帧触发2d roll角度阈值
extern double pose2d_relative_z_limit;      // 当前帧触发2d 相邻帧z值波动阈值
}  // namespace lo

/// 地图配置
namespace map {
extern std::string map_path;  // 地图路径
extern Vec3d map_origin;      // 地图原点
}  // namespace map

/// PGO 配置
namespace pgo {

constexpr int PGO_MAX_FRAMES = 5;                               // PGO所持的最大帧数
constexpr int PGO_MAX_SIZE_OF_RELATIVE_POSE_QUEUE = 10000;      // PGO 相对定位队列最大长度
constexpr int PGO_MAX_SIZE_OF_RTK_POSE_QUEUE = 200;             // PGO RTK观测队列最大长度
constexpr double PGO_DISTANCE_TH_LAST_FRAMES = 2.5;             // PGO 滑窗时，最近两帧的最小距离
constexpr double PGO_ANGLE_TH_LAST_FRAMES = 10 * M_PI / 360.0;  // PGO 滑窗时，最近两帧的最小角度

/// 噪声参数
/// 宁松勿紧避矛盾
extern double lidar_loc_pos_noise;        // lidar定位位置噪声
extern double lidar_loc_ang_noise;        // lidar定位角度噪声
extern double lidar_loc_outlier_th;       // lidar定位异常阈值
extern double lidar_odom_pos_noise;       // LidarOdom相对定位位置噪声
extern double lidar_odom_ang_noise;       // LidarOdom相对定位角度噪声
extern double lidar_odom_outlier_th;      // LidarOdom异常值检测
extern double dr_pos_noise;               // DR相对定位位置噪声
extern double dr_ang_noise;               // DR相对定位角度噪声
extern double dr_pos_noise_ratio;         // DR位置噪声倍率
extern double pgo_frame_converge_pos_th;  // PGO帧位置收敛阈值
extern double pgo_frame_converge_ang_th;  // PGO帧姿态收敛阈值
extern double pgo_smooth_factor;          // PGO平滑因子
}  // namespace pgo

// ui
namespace ui {
extern int pgo_res_rows;  // pgo发送滑窗数据给ui的矩阵的行数
extern float opacity;     // 点云透明度
}  // namespace ui

// lidar_loc
namespace lidar_loc {
extern int grid_search_angle_step;      // 角度网格搜索步数（关键参数）
extern double grid_search_angle_range;  // 角度搜索半径(角度制，关键参数,左右各有)
}  // namespace lidar_loc

/// fasterlio 配置
namespace fasterlio {

/// fixed params
constexpr double INIT_TIME = 0.1;
constexpr int NUM_MATCH_POINTS = 5;      // required matched points in current
constexpr int MIN_NUM_MATCH_POINTS = 3;  // minimum matched points in current

/// configurable params
extern int NUM_MAX_ITERATIONS;      // max iterations of ekf
extern float ESTI_PLANE_THRESHOLD;  // plane threshold
}  // namespace fasterlio

}  // namespace lightning

#endif
