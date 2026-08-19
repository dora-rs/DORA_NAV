#pragma once

#include <array>

#include "pose_message.hpp"

struct PoseWithCovarianceMessage {
    PoseMessage pose;
    std::array<double, 36> covariance;

    PoseWithCovarianceMessage() : covariance{} {}
    PoseWithCovarianceMessage(const PoseMessage& pose_value, const std::array<double, 36>& covariance_value)
        : pose(pose_value), covariance(covariance_value) {}
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PoseWithCovarianceMessage, pose, covariance)

struct PoseWithCovarianceStampedMessage {
    HeaderMessage header;
    PoseWithCovarianceMessage pose;

    PoseWithCovarianceStampedMessage() = default;
    PoseWithCovarianceStampedMessage(const HeaderMessage& hdr, const PoseWithCovarianceMessage& pose_value)
        : header(hdr), pose(pose_value) {}
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PoseWithCovarianceStampedMessage, header, pose)
