#pragma once

#include <array>

#include "base_message.hpp"

struct ImuMessage {
    HeaderMessage header;
    QuaternionMessage orientation;
    std::array<double, 9> orientation_covariance;
    Vector3 angular_velocity;
    std::array<double, 9> angular_velocity_covariance;
    Vector3 linear_acceleration;
    std::array<double, 9> linear_acceleration_covariance;

    ImuMessage()
        : orientation_covariance{},
          angular_velocity_covariance{},
          linear_acceleration_covariance{} {}
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ImuMessage,
    header,
    orientation,
    orientation_covariance,
    angular_velocity,
    angular_velocity_covariance,
    linear_acceleration,
    linear_acceleration_covariance)
