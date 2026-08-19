#pragma once

#include <cstdint>
#include <string>
#include <chrono>

#include "json.hpp"
 
// 时间戳结构
struct Time {
    int32_t sec;
    uint32_t nanosec;
    
    Time() : sec(0), nanosec(0) {}
    Time(int32_t s, uint32_t ns) : sec(s), nanosec(ns) {}
    
    // 转换为chrono时间点（方便使用）
    std::chrono::system_clock::time_point toChrono() const {
        const auto duration = std::chrono::seconds(sec) + std::chrono::nanoseconds(nanosec);
        return std::chrono::system_clock::time_point{
            std::chrono::duration_cast<std::chrono::system_clock::duration>(duration)
        };
    }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Time, sec, nanosec)

// 消息头
struct HeaderMessage {
    Time stamp;
    std::string frame_id;
    uint32_t seq;
    
    HeaderMessage() : frame_id(""), seq(0) {}
    HeaderMessage(const Time& time, const std::string& frame, uint32_t sequence = 0)
        : stamp(time), frame_id(frame), seq(sequence) {}
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(HeaderMessage, stamp, frame_id, seq)



// 基础向量类型
struct Vector3 {
    double x;
    double y;
    double z;
    
    Vector3() : x(0.0), y(0.0), z(0.0) {}
    Vector3(double x_val, double y_val, double z_val) : x(x_val), y(y_val), z(z_val) {}
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Vector3, x, y, z)

// 四元数类型（用于方向表示）
struct QuaternionMessage {
    double x;
    double y;
    double z;
    double w;
    
    QuaternionMessage() : x(0.0), y(0.0), z(0.0), w(1.0) {}
    QuaternionMessage(double x_val, double y_val, double z_val, double w_val)
        : x(x_val), y(y_val), z(z_val), w(w_val) {}
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(QuaternionMessage, x, y, z, w)


// 新增：Point点坐标（用于位置）
struct PointMessage {
    double x;
    double y;
    double z;
    
    PointMessage() : x(0.0), y(0.0), z(0.0) {}
    PointMessage(double x_val, double y_val, double z_val) : x(x_val), y(y_val), z(z_val) {}
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PointMessage, x, y, z)
