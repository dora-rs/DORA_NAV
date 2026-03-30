#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <chrono>


#include "BaseMessage.hpp"
#include "TwistMessage.hpp"


#include "json.hpp" // nlohmann/json (需提前下载)

  
struct OdomMessage {
    HeaderMessage header;
    std::string frame_id;

    struct Pose { //相比于标准的ROS2消息，这里取消了协方差数据
        struct Position { float x=0,y=0,z=0; } position;
        struct Orientation { float x=0,y=0,z=0,w=1; } orientation;
    } pose;
    
    TwistMessage twist;
};
// 2. 关键 一行启用 JSON 互操作
NLOHMANN_DEFINE_TYPE_INTRUSIVE( // 自动序列化/反序列化
    OdomMessage,
    header.stamp.sec, header.stamp.nanosec, header.frame_id, header.seq,
    frame_id,
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    twist.linear.x, twist.linear.y, twist.linear.z,
    twist.angular.x, twist.angular.y, twist.angular.z
);
// NLOHMANN_DEFINE_TYPE_INTRUSIVE 是 nlohmann/json 的宏，**自动为结构体注入 JSON 序列化能力**，无需手写任何转换代码。
