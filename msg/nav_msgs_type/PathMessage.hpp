#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <chrono>


#include "BaseMessage.hpp"
#include "json.hpp" // nlohmann/json (需提前下载)

 
// Path路径消息（核心定义）[1,2](@ref)
struct PathMessage {
    HeaderMessage header;
    std::vector<PoseStampedMessage> poses;
    
    PathMessage() = default;
    
    // 带参数的构造函数
    PathMessage(const HeaderMessage& hdr, const std::vector<PoseStampedMessage>& pose_array) 
        : header(hdr), poses(pose_array) {}
    
    // 便捷方法：添加位姿点
    void addPose(const PoseStampedMessage& pose) {
        poses.push_back(pose);
    }
    
    // 便捷方法：清空路径
    void clear() {
        poses.clear();
    }
    
    // 获取路径点数
    size_t size() const {
        return poses.size();
    }
    
    // 检查路径是否为空
    bool empty() const {
        return poses.empty();
    }
};


// 2. 关键 一行启用 JSON 互操作
// NLOHMANN_DEFINE_TYPE_INTRUSIVE( // 自动序列化/反序列化
//     PathMessage,
//     header.stamp.sec, header.stamp.nanosec, header.frame_id, header.seq,
//     pose.position.x, pose.position.y, pose.position.z,
//     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
//     twist.linear.x, twist.linear.y, twist.linear.z,
//     twist.angular.x, twist.angular.y, twist.angular.z
// );
// NLOHMANN_DEFINE_TYPE_INTRUSIVE 是 nlohmann/json 的宏，**自动为结构体注入 JSON 序列化能力**，无需手写任何转换代码。
