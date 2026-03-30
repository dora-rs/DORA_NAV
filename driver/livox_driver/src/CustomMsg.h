#pragma once

#include <vector>
#include <string>
#include "CustomPoint.h"
struct Header{
    uint32_t seq;
    double stamp;
    std::string frame_id;
};

struct CustomMsg{

    Header header; //standard message header
    uint64_t timebase; //The time of first point
    uint32_t point_num; //Total number of pointclouds
    uint8_t lidar_id; //Lidar device id number
    uint8_t rsvd[3]; //Reserved use
    std::vector<CustomPoint>  points;
};

