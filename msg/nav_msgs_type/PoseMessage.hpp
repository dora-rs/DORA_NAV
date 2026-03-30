#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <chrono>


#include "BaseMessage.hpp"
#include "json.hpp" // nlohmann/json (需提前下载)



// 新增：Pose位姿（位置+方向）
struct PoseMessage {
    PointMessage position;
    QuaternionMessage orientation;
    
    PoseMessage() = default;
    PoseMessage(const PointMessage& pos, const QuaternionMessage& orient) : position(pos), orientation(orient) {}
    
    // 便捷构造函数
    PoseMessage(float x, float y, float z, float qx, float qy, float qz, float qw) {
        position.x = x;
        position.y = y;
        position.z = z;
        orientation.x = qx;
        orientation.y = qy;
        orientation.z = qz;
        orientation.w = qw;
    }
};
NLOHMANN_DEFINE_TYPE_INTRUSIVE( // 自动序列化/反序列化
    PoseMessage,
    position.x, position.y, position.z,
    orientation.x, orientation.y, orientation.z, orientation.w
);


// 新增：PoseStamped带时间戳的位姿[2](@ref)
struct PoseStampedMessage {
    HeaderMessage header;
    PoseMessage pose;
    
    PoseStampedMessage() = default;
    PoseStampedMessage(const HeaderMessage& hdr, const PoseMessage& p) : header(hdr), pose(p) {}
    
    // 便捷构造函数
    PoseStampedMessage(const std::string& frame_id, float x, float y, float z) {
        header.frame_id = frame_id;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.w = 1.0; // 默认无旋转
    }
};
// 2.   一行启用 JSON 互操作
NLOHMANN_DEFINE_TYPE_INTRUSIVE( // 自动序列化/反序列化
    PoseStampedMessage,
    header.stamp.sec, header.stamp.nanosec, header.frame_id, header.seq,
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
);
// NLOHMANN_DEFINE_TYPE_INTRUSIVE 是 nlohmann/json 的宏，**自动为结构体注入 JSON 序列化能力**，无需手写任何转换代码。
