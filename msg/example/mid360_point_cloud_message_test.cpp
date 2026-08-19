#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "nav_msgs/mid360_point_cloud_message.hpp"

int main() {
    static_assert(std::is_standard_layout_v<Mid360PointMessage>);
    static_assert(sizeof(Mid360PointMessage) == 24);
    static_assert(offsetof(Mid360PointMessage, x) == 0);
    static_assert(offsetof(Mid360PointMessage, intensity) == 12);
    static_assert(offsetof(Mid360PointMessage, offset_time) == 16);
    static_assert(offsetof(Mid360PointMessage, tag) == 20);
    static_assert(offsetof(Mid360PointMessage, line) == 21);
    static_assert(offsetof(Mid360PointMessage, reserved) == 22);

    Mid360PointCloudMessage cloud;
    assert(cloud.lidar_id == 0);

    cloud.header = {{1717029202, 123456789}, "mid360", 47};
    cloud.lidar_id = 1;
    cloud.points.push_back({1.0F, 2.0F, 3.0F, 42.0F, 500000U, 0x10, 3, 0});

    const nlohmann::json json = cloud;
    const auto roundtrip = json.get<Mid360PointCloudMessage>();

    assert(roundtrip.header.frame_id == "mid360");
    assert(roundtrip.header.seq == 47);
    assert(roundtrip.lidar_id == 1);
    assert(roundtrip.points.size() == 1);
    assert(roundtrip.points[0].offset_time == 500000U);
    assert(roundtrip.points[0].tag == 0x10);
    assert(roundtrip.points[0].line == 3);
    assert(roundtrip.points[0].reserved == 0);
}
