#pragma once

#include <string>

#include "base_message.hpp"
#include "pose_message.hpp"
#include "twist_message.hpp"

  
struct OdomMessage {
    HeaderMessage header;
    std::string child_frame_id;
    PoseMessage pose;
    TwistDataMessage twist;

    OdomMessage() = default;
    OdomMessage(const HeaderMessage& hdr, const std::string& child_frame, const PoseMessage& p, const TwistDataMessage& t)
        : header(hdr), child_frame_id(child_frame), pose(p), twist(t) {}
};
// 2. 关键 一行启用 JSON 互操作
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE( // 自动序列化/反序列化
    OdomMessage,
    header, child_frame_id, pose, twist
);
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE 是 nlohmann/json 的宏，**自动为结构体注入 JSON 序列化能力**，无需手写任何转换代码。
