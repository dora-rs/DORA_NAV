//
// Created by xiang on 24-4-8.
//

#include <common/options.h>

namespace lightning {

namespace debug {

/// debug and save
bool flg_exit = false;   // ctrl-c中断
bool flg_pause = false;  // 暂停
bool flg_next = false;   // 暂停后，放行单个消息(单步调试)
float play_speed = 10.0;

}  // namespace debug

namespace lo {
float lidar_time_interval = 0.1;  // 雷达的扫描时间
bool use_dr_rotation = true;
int relative_pose_check_cloud_size = 2;
double parking_speed = 0.05;
double parking_count = 5;
double kf_pose_check_distance = 0.5;
double kf_pose_check_angle = 5.0;
double pose2d_roll_limit = 8.0;
double pose2d_pitch_limit = 10.0;
double pose2d_relative_z_limit = 0.25;
}  // namespace lo

namespace fasterlio {
int NUM_MAX_ITERATIONS = 8;
float ESTI_PLANE_THRESHOLD = 0.1;
}  // namespace fasterlio

namespace map {
std::string map_path = "";         // 地图路径
Vec3d map_origin = Vec3d::Zero();  // 地图原点
}  // namespace map

// ui
namespace ui {
int pgo_res_rows = 16;  // pgo发送滑窗数据给ui的矩阵的行数
float opacity = 0.2;    // 点云透明度
}  // namespace ui

// pgo
namespace pgo {

double lidar_loc_pos_noise = 0.3;                             // lidar定位位置噪声 // 0.3
double lidar_loc_ang_noise = 1.0 * constant::kDEG2RAD;        // lidar定位角度噪声
double lidar_loc_outlier_th = 30.0;                           // lidar定位异常阈值
double lidar_odom_pos_noise = 0.3;                            // LidarOdom相对定位位置噪声
double lidar_odom_ang_noise = 1.0 * constant::kDEG2RAD;       // LidarOdom相对定位角度噪声
double lidar_odom_outlier_th = 0.01;                          // LidarOdom异常值检测
double dr_pos_noise = 1.0;                                    // DR相对定位位置噪声 // 0.05
double dr_ang_noise = 0.5 * constant::kDEG2RAD;               // DR相对定位角度噪声
double dr_pos_noise_ratio = 1.0;                              // DR位置噪声倍率
double pgo_frame_converge_pos_th = 0.05;                      // PGO帧位置收敛阈值
double pgo_frame_converge_ang_th = 1.0 * constant::kDEG2RAD;  // PGO帧姿态收敛阈值
double pgo_smooth_factor = 0.01;                              // PGO帧平滑因子

}  // namespace pgo

// lidar_loc
namespace lidar_loc {
int grid_search_angle_step = 20;        // 角度网格搜索步数（关键参数）
double grid_search_angle_range = 20.0;  // 角度搜索半径(角度制，关键参数,左右各有)
}  // namespace lidar_loc

}  // namespace lightning