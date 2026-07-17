extern "C" {
#include "node_api.h"
}

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdint>

#include <nlohmann/json.hpp>
#include <rerun.hpp>

// PCL: 用于读取 .pcd 点云地图
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ==================== 数据结构 ====================

struct PoseData {
    float x, y, z;
    float qw, qx, qy, qz;
};

// 主线程与渲染线程之间共享的最新状态
struct SharedState {
    std::mutex mutex;

    PoseData  latest_pose{};
    bool      pose_updated = false;

    std::vector<rerun::Position3D> latest_path;
    bool      path_updated = false;

    // 实时点云（全局坐标系，已变换对齐地图）
    std::vector<rerun::Position3D> latest_live_cloud;
    bool      live_cloud_updated = false;

    // DWA预测轨迹（全局坐标系）
    std::vector<std::vector<rerun::Position3D>> latest_dwa_trajectories;
    bool      dwa_trajectories_updated = false;

    std::atomic<bool> running{true};
};

// ==================== JSON 解析 ====================

/**
 * @brief 解析定位节点发布的位姿 JSON
 * 格式：{ "header": {...}, "pose": { "position": {x,y,z}, "orientation": {w,x,y,z} } }
 */
bool parsePose(const std::string& json_str, PoseData& pose) {
    try {
        nlohmann::json j = nlohmann::json::parse(json_str);

        if (!j.contains("pose")) {
            std::cerr << "[rerun_visualizer] Missing 'pose' field in pose JSON" << std::endl;
            return false;
        }

        pose.x  = j["pose"]["position"]["x"].get<float>();
        pose.y  = j["pose"]["position"]["y"].get<float>();
        pose.z  = j["pose"]["position"]["z"].get<float>();
        pose.qw = j["pose"]["orientation"]["w"].get<float>();
        pose.qx = j["pose"]["orientation"]["x"].get<float>();
        pose.qy = j["pose"]["orientation"]["y"].get<float>();
        pose.qz = j["pose"]["orientation"]["z"].get<float>();

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[rerun_visualizer] Failed to parse pose JSON: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 解析 A* 规划节点发布的路径 JSON
 * 格式：{ "header": {...}, "poses": [ {x, y}, ... ] }
 * @return 路径点列表（全局坐标系，z=0），解析失败或路径为空时返回空列表
 */
std::vector<rerun::Position3D> parsePath(const std::string& json_str) {
    std::vector<rerun::Position3D> pts;
    try {
        nlohmann::json j = nlohmann::json::parse(json_str);

        // 规划失败时 A* 节点会带 "error": true
        if (j.value("error", false)) {
            std::cerr << "[rerun_visualizer] Received error path from planner, skipping." << std::endl;
            return pts;
        }

        if (!j.contains("poses") || !j["poses"].is_array()) {
            std::cerr << "[rerun_visualizer] Missing or invalid 'poses' field in path JSON" << std::endl;
            return pts;
        }

        const auto& poses = j["poses"];
        pts.reserve(poses.size());
        for (const auto& wp : poses) {
            pts.emplace_back(wp["x"].get<float>(), wp["y"].get<float>(), 0.0f);
        }
    } catch (const std::exception& e) {
        std::cerr << "[rerun_visualizer] Failed to parse path JSON: " << e.what() << std::endl;
        pts.clear();
    }
    return pts;
}

// ==================== 点云解析 ====================

/**
 * @brief 解析 DWA 规划器发布的预测轨迹（二进制格式）
 *
 * Binary 格式（小端）：
 *   [0..3]   uint32_t num_trajectories
 *   然后对每条轨迹：
 *     [0..3]   uint32_t num_points
 *     [4..]    N × 8 字节：float x, y（全局坐标系）
 *
 * @return 轨迹列表，每条轨迹是一个点列表
 */
std::vector<std::vector<rerun::Position3D>> parseDWATrajectories(const char* data, size_t len) {
    std::vector<std::vector<rerun::Position3D>> trajectories;

    if (!data || len < sizeof(uint32_t)) return trajectories;

    const char* ptr = data;

    // 读取轨迹数量
    uint32_t num_trajectories = 0;
    std::memcpy(&num_trajectories, ptr, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    trajectories.reserve(num_trajectories);

    for (uint32_t i = 0; i < num_trajectories; ++i) {
        if (ptr + sizeof(uint32_t) > data + len) break;

        uint32_t num_points = 0;
        std::memcpy(&num_points, ptr, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        std::vector<rerun::Position3D> trajectory;
        trajectory.reserve(num_points);

        for (uint32_t j = 0; j < num_points; ++j) {
            if (ptr + 2 * sizeof(float) > data + len) break;

            float x, y;
            std::memcpy(&x, ptr, sizeof(float));
            ptr += sizeof(float);
            std::memcpy(&y, ptr, sizeof(float));
            ptr += sizeof(float);

            if (std::isfinite(x) && std::isfinite(y)) {
                trajectory.emplace_back(x, y, 0.0f);
            }
        }

        if (!trajectory.empty()) {
            trajectories.push_back(std::move(trajectory));
        }
    }

    return trajectories;
}

/**
 * @brief 解析 livox_driver 发布的 binary 点云，提取本地坐标系下的有效点
 *
 * Binary 格式（小端，紧凑排列）：
 *   [0..7]   double   timestamp（秒，跳过不用）
 *   [8..11]  uint32_t num_points
 *   [12..]   N × 16 字节：float32 x, y, z, intensity（激光雷达本地坐标系）
 *
 * 坐标变换策略：
 *   点云数据保持激光雷达本地坐标系不变，存入 SharedState。
 *   渲染时通过在父实体 "lidar" 上 log Transform3D（平移 + 四元数旋转），
 *   由 Rerun 的实体树层级传播自动将子实体 "lidar/live_cloud" 变换到全局坐标系，
 *   无需手动对每个点做矩阵乘法。
 *
 * 高度过滤：
 *   在本地坐标系下过滤 z >= kZLocalMax 的点（激光雷达上方过高的点），
 *   过滤语义与地图加载时一致（地图过滤全局 z >= 3.0m ≈ 本地 z >= 2.55m）。
 */
std::vector<rerun::Position3D> parseLiveCloudLocal(const char* data, size_t len) {
    std::vector<rerun::Position3D> out;

    constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t);  // 12 bytes
    constexpr size_t POINT_SIZE  = 4 * sizeof(float);                  // 16 bytes
    // 本地 z 过滤阈值：对应地图全局 z_max(3.0) 减去安装高度补偿(0.45)
    constexpr float  kZLocalMax  = 2.55f;

    if (!data || len < HEADER_SIZE) return out;

    // 跳过 timestamp（8 字节），读取点数
    uint32_t num_points = 0;
    std::memcpy(&num_points, data + sizeof(double), sizeof(uint32_t));
    const char* ptr = data + HEADER_SIZE;

    if (len - HEADER_SIZE < static_cast<size_t>(num_points) * POINT_SIZE) return out;

    out.reserve(num_points);
    for (uint32_t i = 0; i < num_points; ++i, ptr += POINT_SIZE) {
        float vals[4];
        std::memcpy(vals, ptr, POINT_SIZE);
        const float x = vals[0], y = vals[1], z = vals[2];
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        if (z >= kZLocalMax) continue;  // 过滤过高点
        out.emplace_back(x, y, z);
    }
    return out;
}

// ==================== 初始化：静态场景 ====================

/**
 * @brief 一次性加载 PCD 点云地图并记录到 Rerun
 *
 * 使用 PCL 读取地图文件，转为 rerun::Points3D 写入 "map/point_cloud" 实体。
 * 调用 log_static() 而非 log()，表示该数据不随时间变化，只传输一次，
 * 避免每帧重复传输海量点云造成带宽浪费。
 *
 * 着色策略：将点的 Z 高度线性映射到灰度 [0x40, 0xD0]，体现地形起伏。
 * z_min / z_max 可按实际地图高度范围调整。
 *
 * 高度补偿：
 *   建图时激光雷达安装于距地面（车轮最低端）0.45m 处，因此 PCD 文件中所有
 *   点的 z 坐标以激光雷达为原点，相对于地面存在 +0.45m 的整体偏移。
 *   显示时需将每个点的 z 减去该偏移，使点云 z=0 对应实际地面，
 *   与车辆坐标系（原点在地面/车轮底端）保持一致。
 */
void loadAndLogPointCloudMap(rerun::RecordingStream& rec, const std::string& pcd_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) < 0) {
        std::cerr << "[rerun_visualizer] Failed to load PCD file: " << pcd_path << std::endl;
        return;
    }
    std::cout << "[rerun_visualizer] PCD map loaded: " << cloud->size() << " points" << std::endl;

    std::vector<rerun::Position3D> positions;
    std::vector<rerun::Color>      colors;
    positions.reserve(cloud->size());
    colors.reserve(cloud->size());

    // 激光雷达安装高度（相对地面/车轮最低端）。
    // 建图时以激光雷达为原点，地面点云的 z ≈ -0.45m；
    // 而定位输出的车辆位姿 z ≈ 0（以激光雷达高度为参考），导致点云整体
    // 比车辆模型低 0.45m，视觉上车辆悬空。
    // 修正方式：点云 z 加上安装高度，使地图与车辆坐标系对齐。
    constexpr float kLidarMountHeight = 0.45f;

    // 高度过滤阈值作用于补偿后的 z，保持过滤逻辑语义不变（≥ 3m 不显示）
    constexpr float z_filter_max = 3.0f;

    for (const auto& pt : *cloud) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;  // 跳过 NaN / Inf 无效点
        }
        // 点云 z 上移激光雷达安装高度，对齐车辆坐标系
        const float z_corrected = pt.z + kLidarMountHeight;
        if (z_corrected >= z_filter_max) {
            continue;  // 过滤高处点
        }
        positions.emplace_back(pt.x, pt.y, z_corrected);
        // 统一绿色显示
        colors.emplace_back(0x00, 0xCC, 0x44, 0xFF);
    }

    // log_static：地图不随时间变化，只发送一次
    rec.log_static(
        "map/point_cloud",
        rerun::Points3D(positions)
            .with_colors(colors)
            .with_radii({0.01f})
    );

    std::cout << "[rerun_visualizer] PCD map logged to Rerun ("
              << positions.size() << " valid points)." << std::endl;
}

/**
 * @brief 一次性定义车辆 Mesh3D 几何（静态部分）
 *
 * 车辆坐标系：X 轴朝前，Y 轴朝左，Z 轴朝上（ROS 标准右手系）。
 * 几何体分为三个子实体，均挂在 "robot/" 前缀下，
 * 位姿由父实体 "robot" 的 Transform3D 统一驱动（层级传播）。
 *
 * 子实体：
 *   "robot/body"   — 车体长方体 Mesh3D（蓝灰色，80% 不透明）
 *   "robot/arrow"  — 车顶朝向三角箭头 Mesh3D（橙色，指向 +X 即车头方向）
 *   "robot/wheels" — 四个车轮 Boxes3D（深灰色，实心）
 *
 * 参数（单位 m，按实车比例可调）：
 *   车长 4.6 m × 车宽 1.8 m × 车高 1.5 m
 *   轮宽 0.25 m，轮半径 0.35 m（Box 近似圆柱）
 */
void defineVehicleMesh(rerun::RecordingStream& rec) {
    // ---- 车体尺寸 ----
    constexpr float L           = 0.68f;
    constexpr float W           = 0.47f;
    constexpr float H           = 0.27f;
    constexpr float wheel_r     = 0.10f;  // 提前定义轮子半径，车体抬高需要用到
    constexpr float lh          = L / 2.f;
    constexpr float wh          = W / 2.f;
    // 车体底面抬高到轮子中心高度（wheel_r），
    // 这样轮子上半部嵌入车底，下半部自然露出贴地
    constexpr float z0          = wheel_r;        // 车底 = 轮子中心高度
    constexpr float z1          = wheel_r + H;    // 车顶

    // ---- 1. 车体 Mesh3D（8 顶点长方体，12 个三角面）----
    std::vector<rerun::Position3D> body_verts = {
        // 底面（z = z0）
        { lh,  wh, z0}, {-lh,  wh, z0}, {-lh, -wh, z0}, { lh, -wh, z0},
        // 顶面（z = z1）
        { lh,  wh, z1}, {-lh,  wh, z1}, {-lh, -wh, z1}, { lh, -wh, z1},
    };
    std::vector<rerun::components::TriangleIndices> body_tris = {
        {0, 1, 2}, {0, 2, 3},    // 底面
        {4, 6, 5}, {4, 7, 6},    // 顶面
        {0, 3, 7}, {0, 7, 4},    // 前面（+X，车头）
        {1, 5, 6}, {1, 6, 2},    // 后面（-X，车尾）
        {0, 4, 5}, {0, 5, 1},    // 左侧（+Y）
        {3, 2, 6}, {3, 6, 7},    // 右侧（-Y）
    };

    rec.log_static(
        "robot/body",
        rerun::Mesh3D(body_verts)
            .with_triangle_indices(body_tris)
            .with_albedo_factor(rerun::components::AlbedoFactor(
                rerun::datatypes::Rgba32(0x2F, 0x72, 0xB8, 0xCC)  // 蓝灰色，80% 不透明
            ))
    );

    // ---- 2. 朝向箭头三角形（车顶平面，指向 +X 车头方向）----
    constexpr float az = z1 + 0.05f;  // 稍高于车顶，避免 z-fighting
    std::vector<rerun::Position3D> arrow_verts = {
        { lh * 0.8f,        0.0f, az},   // 箭头尖（前）
        {-lh * 0.3f,  wh * 0.5f, az},   // 左后
        {-lh * 0.3f, -wh * 0.5f, az},   // 右后
    };
    std::vector<rerun::components::TriangleIndices> arrow_tris = {{0, 2, 1}};

    rec.log_static(
        "robot/arrow",
        rerun::Mesh3D(arrow_verts)
            .with_triangle_indices(arrow_tris)
            .with_albedo_factor(rerun::components::AlbedoFactor(
                rerun::datatypes::Rgba32(0xFF, 0x69, 0x00, 0xFF)  // 橙色
            ))
    );

    // ---- 3. 四轮（Cylinders3D）----
    // 轴向：绕 X 轴旋转 90°，圆柱轴向从 +Z 变为 +Y（轮轴方向）
    // Z方向：wheel_wz = wheel_r，车体底面在 z=wheel_r，轮子上半嵌入车底，下半露出贴地
    // Y方向：wheel_wy = wh - wheel_width/2，外侧面与车体侧面齐平，不从侧面突出
    constexpr float wheel_width  = 0.04f;
    constexpr float wheel_wx     = lh * 0.65f;              // 前后轴位置
    constexpr float wheel_wy     = wh - wheel_width * 0.5f; // 外侧面与车体侧面齐平
    constexpr float wheel_wz     = wheel_r;                  // 轮子中心 = 车底高度

    rec.log_static(
        "robot/wheels",
        rerun::Cylinders3D::from_lengths_and_radii(
            {wheel_width, wheel_width, wheel_width, wheel_width},
            {wheel_r, wheel_r, wheel_r, wheel_r}
        )
        .with_centers({
            { wheel_wx,  wheel_wy, wheel_wz},  // 前左
            { wheel_wx, -wheel_wy, wheel_wz},  // 前右
            {-wheel_wx,  wheel_wy, wheel_wz},  // 后左
            {-wheel_wx, -wheel_wy, wheel_wz},  // 后右
        })
        .with_rotation_axis_angles({
            rerun::RotationAxisAngle({1.0f, 0.0f, 0.0f}, rerun::Angle::degrees(90.0f)),
            rerun::RotationAxisAngle({1.0f, 0.0f, 0.0f}, rerun::Angle::degrees(90.0f)),
            rerun::RotationAxisAngle({1.0f, 0.0f, 0.0f}, rerun::Angle::degrees(90.0f)),
            rerun::RotationAxisAngle({1.0f, 0.0f, 0.0f}, rerun::Angle::degrees(90.0f)),
        })
        .with_fill_mode(rerun::components::FillMode::Solid)
        .with_colors({rerun::Rgba32(0x22, 0x22, 0x22, 0xFF)})
    );
}

// ==================== Rerun 渲染函数 ====================

/**
 * @brief 优化后的位姿渲染（原版对比分析）
 *
 * 原版问题：
 *   1. 手动从四元数提取 yaw，构造 2D 箭头方向向量 —— 丢失 pitch/roll 信息，
 *      且三角函数计算是冗余的，Rerun 原生支持四元数输入。
 *   2. 箭头原点和向量分开构造，代码啰嗦。
 *   3. 每帧都渲染独立箭头，无法复用车辆几何体。
 *
 * 新版方案：
 *   - 将完整 6-DOF 四元数直接写入父实体 "robot" 的 Transform3D，
 *     子实体 body / arrow / wheels 自动继承该变换（层级传播）。
 *   - 不再需要手动三角函数，Rerun 内部处理旋转合成。
 *   - 所有子实体几何只定义一次（log_static），每帧只更新一个 Transform3D，
 *     带宽占用极低。
 */
void pose2rerun(const PoseData& pose, rerun::RecordingStream& rec) {
    rec.log(
        "robot",
        rerun::Transform3D::from_translation_rotation(
            {pose.x, pose.y, pose.z},
            rerun::Rotation3D{
                rerun::datatypes::Quaternion::from_wxyz(pose.qw, pose.qx, pose.qy, pose.qz)
            }
        )
    );
}

/**
 * @brief 优化后的路径渲染
 *
 * 原版问题：
 *   - 只有折线 + 终点圆点，缺少起点标记和路径密度信息。
 *   - 终点仅用小圆点，俯视图下不够醒目。
 *
 * 新版方案：
 *   1. LineStrips3D 折线（API 已是最优，保持不变）
 *   2. Points3D 中间采样点（每 5 个路径点显示 1 个，辅助判断路径密度）
 *   3. Points3D 起点（绿色，带 "Start" 标签）
 *   4. Boxes3D 实心正方形终点标记（红色，俯视下比圆点更醒目，带 "Goal" 标签）
 */
void path2rerun(const std::vector<rerun::Position3D>& pts, rerun::RecordingStream& rec) {
    if (pts.size() < 2) return;

    // 1. 路径折线
    std::vector<rerun::components::LineStrip3D> strips = {rerun::components::LineStrip3D(pts)};
    rec.log(
        "planner/astar_path",
        rerun::LineStrips3D(strips)
            .with_colors(0x00BFFFFF)
            .with_radii({0.06f})
    );

    // 2. 起点标记（绿色圆，带标签）
    std::vector<rerun::Position3D> start_pt = {pts.front()};
    rec.log(
        "planner/start",
        rerun::Points3D(start_pt)
            .with_colors(0x00FF00FF)
            .with_radii({0.15f})
            .with_labels({"Start"})
    );

    // 3. 终点标记（红色小圆）
    std::vector<rerun::Position3D> goal_pt = {pts.back()};
    rec.log(
        "planner/goal",
        rerun::Points3D(goal_pt)
            .with_colors(0xFF0000FF)
            .with_radii({0.15f})
            .with_labels({"Goal"})
    );
}

/**
 * @brief 渲染 DWA 预测轨迹
 *
 * 将所有预测的轨迹显示为半透明的细线，便于观察算法的采样空间
 */
void dwaTrajectories2rerun(const std::vector<std::vector<rerun::Position3D>>& trajectories, rerun::RecordingStream& rec) {
    if (trajectories.empty()) return;

    std::vector<rerun::components::LineStrip3D> strips;
    strips.reserve(trajectories.size());

    for (const auto& traj : trajectories) {
        if (traj.size() >= 2) {
            strips.emplace_back(traj);
        }
    }

    if (!strips.empty()) {
        rec.log(
            "planner/dwa_trajectories",
            rerun::LineStrips3D(strips)
                .with_colors(rerun::Color(0xFF, 0xA5, 0x00, 0x60))  // 橙色，半透明
                .with_radii({0.02f})
        );
    }
}

// ==================== 渲染线程 ====================

/**
 * @brief 渲染线程：固定 30Hz 从共享状态读取最新数据并调用 rec.log()
 *
 * 与主线程分离后，主线程可以全速消费 Dora 事件队列，不会因为
 * rec.log() 的序列化开销而积压，从而消除位姿显示延迟。
 */
void renderLoop(SharedState& state, rerun::RecordingStream& rec) {
    constexpr auto kRenderInterval = std::chrono::milliseconds(33);  // ≈ 30 Hz

    while (state.running.load()) {
        PoseData  pose_to_render{};
        bool      has_pose = false;
        std::vector<rerun::Position3D> path_to_render;
        bool      has_path = false;
        std::vector<rerun::Position3D> cloud_to_render;
        bool      has_cloud = false;
        PoseData  cloud_pose{};  // 与点云同批拷贝的位姿，用于设置 Transform3D
        std::vector<std::vector<rerun::Position3D>> dwa_trajectories_to_render;
        bool      has_dwa_trajectories = false;

        // 加锁只做数据拷贝，尽量短暂
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            if (state.pose_updated) {
                pose_to_render = state.latest_pose;
                has_pose = true;
                state.pose_updated = false;
            }
            if (state.path_updated) {
                path_to_render = state.latest_path;
                has_path = true;
                state.path_updated = false;
            }
            if (state.live_cloud_updated) {
                cloud_to_render = state.latest_live_cloud;
                cloud_pose      = state.latest_pose;  // 取同一把锁内的最新位姿
                has_cloud = true;
                state.live_cloud_updated = false;
            }
            if (state.dwa_trajectories_updated) {
                dwa_trajectories_to_render = state.latest_dwa_trajectories;
                has_dwa_trajectories = true;
                state.dwa_trajectories_updated = false;
            }
        }

        // 锁外执行渲染，耗时不影响主线程写入
        if (has_pose) {
            pose2rerun(pose_to_render, rec);
        }
        if (has_path) {
            path2rerun(path_to_render, rec);
            std::cout << "[rerun_visualizer] Path updated: "
                      << path_to_render.size() << " waypoints" << std::endl;
        }
        if (has_cloud) {
            // 1. 更新父实体 "lidar" 的 Transform3D（本地→全局坐标系变换）
            //    子实体 "lidar/live_cloud" 的 Points3D 会自动继承此变换，
            //    无需对每个点手动做矩阵乘法
            rec.log(
                "lidar",
                rerun::Transform3D::from_translation_rotation(
                    {cloud_pose.x, cloud_pose.y, cloud_pose.z},
                    rerun::Rotation3D{rerun::datatypes::Quaternion::from_wxyz(
                        cloud_pose.qw, cloud_pose.qx, cloud_pose.qy, cloud_pose.qz
                    )}
                )
            );

            // 2. 发布实时点云（本地坐标，由父节点 Transform3D 自动变换到全局系）
            //    红色显示，与绿色地图点云形成视觉区分
            rec.log(
                "lidar/live_cloud",
                rerun::Points3D(cloud_to_render)
                    .with_colors(rerun::Color(0xFF, 0x30, 0x30, 0xFF))
                    .with_radii({0.02f})
            );
        }
        if (has_dwa_trajectories) {
            dwaTrajectories2rerun(dwa_trajectories_to_render, rec);
        }

        std::this_thread::sleep_for(kRenderInterval);
    }
}

// ==================== 主事件循环 ====================

int main() {
    std::cout << "[rerun_visualizer] Starting..." << std::endl;

    void* dora_context = init_dora_context_from_env();
    if (!dora_context) {
        std::cerr << "[rerun_visualizer] Failed to initialize dora context" << std::endl;
        return -1;
    }

    // 初始化 Rerun，spawn() 自动启动 Rerun Viewer 进程
    rerun::RecordingStream rec("rerun_visualizer");
    rec.spawn().exit_on_failure();

    std::cout << "[rerun_visualizer] Rerun viewer spawned." << std::endl;

    // ---- 全局坐标轴约定：Z 轴朝上，X 轴朝前（符合 ROS / 车辆坐标系）----
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

    // ---- 一次性加载并显示 PCD 点云地图 ----
    std::string pcd_path = "../maps/pcd/map.pcd";
    loadAndLogPointCloudMap(rec, pcd_path);

    // ---- 一次性定义车辆 Mesh3D 几何（静态部分，姿态由 Transform3D 动态更新）----
    defineVehicleMesh(rec);

    std::cout << "[rerun_visualizer] Static scene initialized, entering event loop..." << std::endl;

    SharedState state;

    // 启动渲染线程
    std::thread render_thread(renderLoop, std::ref(state), std::ref(rec));

    // 主线程：只负责消费 Dora 事件并更新共享状态，不做任何渲染
    while (true) {
        void* event = dora_next_event(dora_context);
        if (!event) {
            std::cerr << "[rerun_visualizer] Unexpected end of event" << std::endl;
            break;
        }

        DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input) {
            char*  data;
            size_t data_len;
            char*  id;
            size_t id_len;

            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &id, &id_len);
            std::string topic(id, id_len);

            if (topic == "pose") {
                PoseData pose;
                if (parsePose(std::string(data, data_len), pose)) {
                    std::lock_guard<std::mutex> lock(state.mutex);
                    state.latest_pose  = pose;
                    state.pose_updated = true;
                }
            } else if (topic == "path") {
                auto pts = parsePath(std::string(data, data_len));
                if (!pts.empty()) {
                    std::lock_guard<std::mutex> lock(state.mutex);
                    state.latest_path  = std::move(pts);
                    state.path_updated = true;
                }
            } else if (topic == "pointcloud") {
                // binary 格式，直接传 raw 指针，无需拷贝成 string
                auto cloud = parseLiveCloudLocal(data, data_len);
                if (!cloud.empty()) {
                    std::lock_guard<std::mutex> lock(state.mutex);
                    state.latest_live_cloud  = std::move(cloud);
                    state.live_cloud_updated = true;
                }
            } else if (topic == "dwa_trajectories") {
                // binary 格式，解析 DWA 预测轨迹
                auto trajectories = parseDWATrajectories(data, data_len);
                if (!trajectories.empty()) {
                    std::lock_guard<std::mutex> lock(state.mutex);
                    state.latest_dwa_trajectories  = std::move(trajectories);
                    state.dwa_trajectories_updated = true;
                }
            }
        } else if (ty == DoraEventType_Stop) {
            std::cout << "[rerun_visualizer] Received stop event" << std::endl;
            free_dora_event(event);
            break;
        }

        free_dora_event(event);
    }

    // 通知渲染线程退出并等待其结束
    state.running.store(false);
    render_thread.join();

    free_dora_context(dora_context);
    std::cout << "[rerun_visualizer] Exited." << std::endl;
    return 0;
}
