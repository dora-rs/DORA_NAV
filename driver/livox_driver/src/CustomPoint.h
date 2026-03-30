#pragma once

#include <vector>
#include <string>

struct CustomPoint{

    uint32_t offset_time; //offset time relative to the base time
    _Float32 x; //X axis, unit:m
    _Float32 y; //Y axis, unit:m
    _Float32 z; //Z axis, unit:m
    uint8_t reflectivity; //reflectivity, 0~255
    uint8_t tag; //livox tag
    uint8_t line; //laser number in lidar
};