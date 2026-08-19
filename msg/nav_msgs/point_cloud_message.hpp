#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "base_message.hpp"

// XYZI 点记录：4 个连续的 float32，共 16 字节。
struct PointXYZIMessage {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PointXYZIMessage, x, y, z, intensity)

static_assert(std::is_standard_layout_v<PointXYZIMessage>);
static_assert(sizeof(PointXYZIMessage) == 16);
static_assert(offsetof(PointXYZIMessage, x) == 0);
static_assert(offsetof(PointXYZIMessage, y) == 4);
static_assert(offsetof(PointXYZIMessage, z) == 8);
static_assert(offsetof(PointXYZIMessage, intensity) == 12);

struct PointCloudXYZIMessage {
    HeaderMessage header;
    std::vector<PointXYZIMessage> points;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PointCloudXYZIMessage, header, points)

// FLIO v1 Standard 点记录：XYZ、强度、相对时间、线束和保留字段，共 24 字节。
struct PointXYZITRRMessage {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    float time = 0.0F;
    uint16_t ring = 0;
    uint16_t reserved = 0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PointXYZITRRMessage,
    x,
    y,
    z,
    intensity,
    time,
    ring,
    reserved)

static_assert(std::is_standard_layout_v<PointXYZITRRMessage>);
static_assert(sizeof(PointXYZITRRMessage) == 24);
static_assert(offsetof(PointXYZITRRMessage, x) == 0);
static_assert(offsetof(PointXYZITRRMessage, y) == 4);
static_assert(offsetof(PointXYZITRRMessage, z) == 8);
static_assert(offsetof(PointXYZITRRMessage, intensity) == 12);
static_assert(offsetof(PointXYZITRRMessage, time) == 16);
static_assert(offsetof(PointXYZITRRMessage, ring) == 20);
static_assert(offsetof(PointXYZITRRMessage, reserved) == 22);

struct PointCloudXYZITRRMessage {
    HeaderMessage header;
    std::string magic = "FLIO";
    uint8_t version = 1;
    uint8_t endianness = 0;
    uint8_t format_kind = 1;
    uint8_t field_flags = 0x03;
    uint16_t header_size = 40;
    uint16_t point_stride = 24;
    uint8_t dimensions = 3;
    std::vector<PointXYZITRRMessage> points;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PointCloudXYZITRRMessage,
    header,
    magic,
    version,
    endianness,
    format_kind,
    field_flags,
    header_size,
    point_stride,
    dimensions,
    points)
