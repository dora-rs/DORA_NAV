#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <chrono>

 
// 时间戳结构
struct Time {
    int32_t sec;
    uint32_t nanosec;
    
    Time() : sec(0), nanosec(0) {}
    Time(int32_t s, uint32_t ns) : sec(s), nanosec(ns) {}
    
    // 转换为chrono时间点（方便使用）
    std::chrono::system_clock::time_point toChrono() const;
};

// 消息头
struct HeaderMessage {
    Time stamp;
    std::string frame_id;
    
    HeaderMessage() : frame_id("") {}
    HeaderMessage(const Time& time, const std::string& frame) 
        : stamp(time), frame_id(frame) {}
};



// 基础向量类型
struct Vector3 {
    float x;
    float y;
    float z;
    
    Vector3() : x(0.0), y(0.0), z(0.0) {}
    Vector3(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
};

// 四元数类型（用于方向表示）
struct QuaternionMessage {
    float x;
    float y;
    float z;
    float w;
    
    QuaternionMessage() : x(0.0), y(0.0), z(0.0), w(1.0) {}
    QuaternionMessage(float x_val, float y_val, float z_val, float w_val) 
        : x(x_val), y(y_val), z(z_val), w(w_val) {}
};


// 新增：Point点坐标（用于位置）
struct PointMessage {
    float x;
    float y;
    float z;
    
    PointMessage() : x(0.0), y(0.0), z(0.0) {}
    PointMessage(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
};