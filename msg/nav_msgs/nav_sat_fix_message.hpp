#pragma once

#include <array>
#include <cstdint>

#include "base_message.hpp"

struct NavSatStatusMessage {
    static constexpr int8_t STATUS_NO_FIX = -1;
    static constexpr int8_t STATUS_FIX = 0;
    static constexpr int8_t STATUS_SBAS_FIX = 1;
    static constexpr int8_t STATUS_GBAS_FIX = 2;

    static constexpr uint16_t SERVICE_GPS = 1;
    static constexpr uint16_t SERVICE_GLONASS = 2;
    static constexpr uint16_t SERVICE_COMPASS = 4;
    static constexpr uint16_t SERVICE_GALILEO = 8;

    int8_t status;
    uint16_t service;

    NavSatStatusMessage() : status(STATUS_NO_FIX), service(0) {}
    NavSatStatusMessage(int8_t status_value, uint16_t service_value)
        : status(status_value), service(service_value) {}
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(NavSatStatusMessage, status, service)

struct NavSatFixMessage {
    static constexpr uint8_t COVARIANCE_TYPE_UNKNOWN = 0;
    static constexpr uint8_t COVARIANCE_TYPE_APPROXIMATED = 1;
    static constexpr uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
    static constexpr uint8_t COVARIANCE_TYPE_KNOWN = 3;

    HeaderMessage header;
    NavSatStatusMessage status;
    double latitude;
    double longitude;
    double altitude;
    std::array<double, 9> position_covariance;
    uint8_t position_covariance_type;

    NavSatFixMessage()
        : latitude(0.0),
          longitude(0.0),
          altitude(0.0),
          position_covariance{},
          position_covariance_type(COVARIANCE_TYPE_UNKNOWN) {}
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    NavSatFixMessage,
    header,
    status,
    latitude,
    longitude,
    altitude,
    position_covariance,
    position_covariance_type)
