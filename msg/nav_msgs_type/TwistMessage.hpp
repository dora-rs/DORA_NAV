#pragma once
 

#include "BaseMessage.hpp"
#include "json.hpp" // nlohmann/json (需提前下载)

// Twist消息（cmd_vel）[1,7](@ref)
struct TwistMessage {

    struct Header {
        struct Stamp { int sec; int nanosec; } stamp;
        std::string frame_id;
        int seq = 0;
    } header;
    
    Vector3 linear;
    Vector3 angular;
    
    TwistMessage() = default;
    TwistMessage(const Vector3& lin, const Vector3& ang) : linear(lin), angular(ang) {}
     
};

// 2. 关键 一行启用 JSON 互操作
NLOHMANN_DEFINE_TYPE_INTRUSIVE( // 自动序列化/反序列化
    TwistMessage,
    header.stamp.sec, header.stamp.nanosec, header.frame_id, header.seq,
    linear.x, linear.y, linear.z,
    angular.x, angular.y, angular.z
);