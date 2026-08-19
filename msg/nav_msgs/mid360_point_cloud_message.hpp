#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "base_message.hpp"

// Livox MID-360 点记录。offset_time 是相对 header.stamp 的纳秒偏移。
struct Mid360PointMessage {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    uint32_t offset_time = 0;
    uint8_t tag = 0;
    uint8_t line = 0;
    uint16_t reserved = 0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    Mid360PointMessage,
    x,
    y,
    z,
    intensity,
    offset_time,
    tag,
    line,
    reserved)

static_assert(std::is_standard_layout_v<Mid360PointMessage>);
static_assert(sizeof(Mid360PointMessage) == 24);
static_assert(offsetof(Mid360PointMessage, x) == 0);
static_assert(offsetof(Mid360PointMessage, y) == 4);
static_assert(offsetof(Mid360PointMessage, z) == 8);
static_assert(offsetof(Mid360PointMessage, intensity) == 12);
static_assert(offsetof(Mid360PointMessage, offset_time) == 16);
static_assert(offsetof(Mid360PointMessage, tag) == 20);
static_assert(offsetof(Mid360PointMessage, line) == 21);
static_assert(offsetof(Mid360PointMessage, reserved) == 22);

struct Mid360PointCloudMessage {
    HeaderMessage header;
    uint8_t lidar_id = 0;
    std::vector<Mid360PointMessage> points;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    Mid360PointCloudMessage,
    header,
    lidar_id,
    points)
