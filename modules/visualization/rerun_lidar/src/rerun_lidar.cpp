extern "C" {
#include "node_api.h"
}

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <algorithm>

#include <rerun.hpp>

// ==================== 点云二进制格式 ====================
//
// 与 livox_driver / rslidar_driver 保持统一的二进制格式（小端序）：
//   [0..7]   double   timestamp（秒）
//   [8..11]  uint32_t num_points
//   [12..]   N × 16 字节：float32 x, y, z, intensity
//
// =========================================================

/**
 * @brief 按反射强度着色：蓝(低) → 青 → 绿 → 黄 → 红(高)
 *
 * 将 intensity [0, 255] 映射到 jet 风格的热力色标。
 * 低反射率的物体（如黑色表面）呈蓝色，
 * 高反射率的物体（如标牌/车牌）呈红色。
 */
inline rerun::Color intensityToColor(float intensity) {
    // 归一化到 [0, 1]，并 clamp
    float t = std::clamp(intensity / 255.0f, 0.0f, 1.0f);

    // jet colormap 分段线性插值
    //  0.00 -> 蓝   (0, 0, 128)
    //  0.25 -> 青   (0, 255, 255)
    //  0.50 -> 绿   (0, 255, 0)
    //  0.75 -> 黄   (255, 255, 0)
    //  1.00 -> 红   (255, 0, 0)

    uint8_t r, g, b;

    if (t < 0.25f) {
        float s = t / 0.25f;
        r = 0;
        g = static_cast<uint8_t>(s * 255.0f);
        b = static_cast<uint8_t>(128.0f + s * 127.0f);
    } else if (t < 0.5f) {
        float s = (t - 0.25f) / 0.25f;
        r = 0;
        g = 255;
        b = static_cast<uint8_t>(255.0f - s * 255.0f);
    } else if (t < 0.75f) {
        float s = (t - 0.5f) / 0.25f;
        r = static_cast<uint8_t>(s * 255.0f);
        g = 255;
        b = 0;
    } else {
        float s = (t - 0.75f) / 0.25f;
        r = 255;
        g = static_cast<uint8_t>(255.0f - s * 255.0f);
        b = 0;
    }

    return rerun::Color(r, g, b, 0xFF);
}

/**
 * @brief 解析二进制点云数据
 *
 * @param data      原始字节缓冲区
 * @param data_len  缓冲区长度
 * @param out_positions  输出：3D 位置列表
 * @param out_colors     输出：对应的颜色列表（按强度映射）
 * @return 解析是否成功
 */
bool parsePointCloud(const char* data, size_t data_len,
                     std::vector<rerun::Position3D>& out_positions,
                     std::vector<rerun::Color>& out_colors) {
    constexpr size_t kHeaderSize = sizeof(double) + sizeof(uint32_t);  // 12 bytes
    constexpr size_t kPointSize  = 4 * sizeof(float);                   // 16 bytes

    if (!data || data_len < kHeaderSize) {
        return false;
    }

    // 跳过 time-stamp (8 字节)，读取点数
    uint32_t num_points = 0;
    std::memcpy(&num_points, data + sizeof(double), sizeof(uint32_t));

    if (data_len - kHeaderSize < static_cast<size_t>(num_points) * kPointSize) {
        std::cerr << "[rerun_lidar] Truncated point cloud: " << num_points
                  << " points expected, but only "
                  << (data_len - kHeaderSize) / kPointSize << " fit in buffer"
                  << std::endl;
        return false;
    }

    out_positions.clear();
    out_colors.clear();
    out_positions.reserve(num_points);
    out_colors.reserve(num_points);

    const char* ptr = data + kHeaderSize;
    for (uint32_t i = 0; i < num_points; ++i, ptr += kPointSize) {
        float vals[4];
        std::memcpy(vals, ptr, kPointSize);

        const float x = vals[0], y = vals[1], z = vals[2], intensity = vals[3];

        // 滤除 NaN / Inf 无效点
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        out_positions.emplace_back(x, y, z);
        out_colors.push_back(intensityToColor(intensity));
    }

    return true;
}

// ==================== 主函数 ====================

int main() {
    std::cout << "[rerun_lidar] Starting..." << std::endl;

    // ── Dora 上下文初始化 ──────────────────────────────────
    void* dora_context = init_dora_context_from_env();
    if (!dora_context) {
        std::cerr << "[rerun_lidar] Failed to initialize dora context" << std::endl;
        return -1;
    }
    std::cout << "[rerun_lidar] Dora context initialized." << std::endl;

    // ── Rerun 初始化 ──────────────────────────────────────
    // spawn() 自动启动 Rerun Viewer 进程
    rerun::RecordingStream rec("rerun_lidar");
    rec.spawn().exit_on_failure();
    std::cout << "[rerun_lidar] Rerun viewer spawned." << std::endl;

    // 全局坐标系：右手系 Z 轴朝上（ROS / 车辆坐标系约定）
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

    std::cout << "[rerun_lidar] Entering event loop..." << std::endl;

    // ── 主事件循环 ─────────────────────────────────────
    while (true) {
        void* event = dora_next_event(dora_context);
        if (!event) {
            std::cerr << "[rerun_lidar] Unexpected end of event" << std::endl;
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

            if (topic == "pointcloud") {
                std::vector<rerun::Position3D> positions;
                std::vector<rerun::Color>      colors;

                if (parsePointCloud(data, data_len, positions, colors)) {
                    if (!positions.empty()) {
                        rec.log(
                            "lidar/points",
                            rerun::Points3D(positions)
                                .with_colors(colors)
                                .with_radii({0.02f})
                        );
                    }
                }
            }
        } else if (ty == DoraEventType_Stop) {
            std::cout << "[rerun_lidar] Received stop event" << std::endl;
            free_dora_event(event);
            break;
        }

        free_dora_event(event);
    }

    // ── 清理 ──────────────────────────────────────────
    free_dora_context(dora_context);
    std::cout << "[rerun_lidar] Exited." << std::endl;
    return 0;
}
