#pragma once
 

#include "base_message.hpp"

struct TwistDataMessage {
    Vector3 linear;
    Vector3 angular;

    TwistDataMessage() = default;
    TwistDataMessage(const Vector3& lin, const Vector3& ang) : linear(lin), angular(ang) {}
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TwistDataMessage, linear, angular)

// Twist消息（cmd_vel）[1,7](@ref)
struct TwistMessage {
    HeaderMessage header;
    Vector3 linear;
    Vector3 angular;

    TwistMessage() = default;
    TwistMessage(const Vector3& lin, const Vector3& ang) : linear(lin), angular(ang) {}
    TwistMessage(const HeaderMessage& hdr, const Vector3& lin, const Vector3& ang)
        : header(hdr), linear(lin), angular(ang) {}
};

// 2. 关键 一行启用 JSON 互操作
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE( // 自动序列化/反序列化
    TwistMessage,
    header, linear, angular
);
