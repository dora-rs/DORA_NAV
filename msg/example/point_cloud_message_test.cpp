#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "nav_msgs/point_cloud_message.hpp"

int main() {
    static_assert(std::is_same_v<decltype(HeaderMessage::seq), uint32_t>);

    static_assert(std::is_standard_layout_v<PointXYZIMessage>);
    static_assert(sizeof(PointXYZIMessage) == 16);
    static_assert(offsetof(PointXYZIMessage, x) == 0);
    static_assert(offsetof(PointXYZIMessage, intensity) == 12);

    static_assert(std::is_standard_layout_v<PointXYZITRRMessage>);
    static_assert(sizeof(PointXYZITRRMessage) == 24);
    static_assert(offsetof(PointXYZITRRMessage, x) == 0);
    static_assert(offsetof(PointXYZITRRMessage, intensity) == 12);
    static_assert(offsetof(PointXYZITRRMessage, time) == 16);
    static_assert(offsetof(PointXYZITRRMessage, ring) == 20);
    static_assert(offsetof(PointXYZITRRMessage, reserved) == 22);

    PointCloudXYZIMessage xyzi;
    xyzi.header = {{1717029202, 123456789}, "lidar", 7};
    xyzi.points.push_back({1.0F, 2.0F, 3.0F, 4.0F});

    const nlohmann::json xyzi_json = xyzi;
    const auto xyzi_roundtrip = xyzi_json.get<PointCloudXYZIMessage>();
    assert(xyzi_roundtrip.header.frame_id == "lidar");
    assert(xyzi_roundtrip.header.seq == 7);
    assert(xyzi_roundtrip.points.size() == 1);
    assert(xyzi_roundtrip.points[0].intensity == 4.0F);

    PointCloudXYZITRRMessage xyzitrr;
    assert(xyzitrr.magic == "FLIO");
    assert(xyzitrr.version == 1);
    assert(xyzitrr.endianness == 0);
    assert(xyzitrr.format_kind == 1);
    assert(xyzitrr.field_flags == 0x03);
    assert(xyzitrr.point_stride == 24);
    assert(xyzitrr.dimensions == 3);

    xyzitrr.header = {{1717029203, 987654321}, "lidar", 8};
    xyzitrr.points.push_back({1.0F, 2.0F, 3.0F, 4.0F, 0.005F, 12, 0});

    const nlohmann::json xyzitrr_json = xyzitrr;
    const auto xyzitrr_roundtrip = xyzitrr_json.get<PointCloudXYZITRRMessage>();
    assert(xyzitrr_roundtrip.header.stamp.nanosec == 987654321U);
    assert(xyzitrr_roundtrip.points.size() == 1);
    assert(xyzitrr_roundtrip.points[0].ring == 12);
    assert(xyzitrr_roundtrip.points[0].reserved == 0);
}
