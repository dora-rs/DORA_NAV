#pragma once

#include <string>

#include "pose_with_covariance_message.hpp"
#include "twist_with_covariance_message.hpp"

struct OdometryWithCovarianceMessage {
    HeaderMessage header;
    std::string child_frame_id;
    PoseWithCovarianceMessage pose;
    TwistWithCovarianceMessage twist;

    OdometryWithCovarianceMessage() = default;
    OdometryWithCovarianceMessage(
        const HeaderMessage& hdr,
        const std::string& child_frame,
        const PoseWithCovarianceMessage& pose_value,
        const TwistWithCovarianceMessage& twist_value)
        : header(hdr), child_frame_id(child_frame), pose(pose_value), twist(twist_value) {}
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OdometryWithCovarianceMessage, header, child_frame_id, pose, twist)
