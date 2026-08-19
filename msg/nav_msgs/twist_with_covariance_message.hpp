#pragma once

#include <array>

#include "twist_message.hpp"

struct TwistWithCovarianceMessage {
    TwistDataMessage twist;
    std::array<double, 36> covariance;

    TwistWithCovarianceMessage() : covariance{} {}
    TwistWithCovarianceMessage(const TwistDataMessage& twist_value, const std::array<double, 36>& covariance_value)
        : twist(twist_value), covariance(covariance_value) {}
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TwistWithCovarianceMessage, twist, covariance)
