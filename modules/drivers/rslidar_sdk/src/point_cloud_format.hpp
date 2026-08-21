#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace rslidar_dora {

enum class PointCloudFormat { XYZI, XYZITRR };

inline PointCloudFormat parsePointCloudFormat(const char* value) {
  if (value == nullptr || value[0] == '\0' || std::string(value) == "XYZI") {
    return PointCloudFormat::XYZI;
  }
  if (std::string(value) == "XYZITRR") {
    return PointCloudFormat::XYZITRR;
  }
  throw std::invalid_argument(
      "POINT_CLOUD_FORMAT must be XYZI or XYZITRR (received '" +
      std::string(value) + "')");
}

inline const char* pointCloudFormatName(PointCloudFormat format) {
  return format == PointCloudFormat::XYZI ? "XYZI" : "XYZITRR";
}

template <typename T>
inline void append(std::vector<char>& bytes, std::size_t offset, const T& value) {
  std::memcpy(bytes.data() + offset, &value, sizeof(value));
}

template <typename Points>
std::vector<char> serializeXyzi(double timestamp, const Points& points) {
  const auto count = static_cast<std::uint32_t>(points.size());
  std::vector<char> bytes(12U + static_cast<std::size_t>(count) * 16U, 0);
  append(bytes, 0, timestamp);
  append(bytes, 8, count);
  std::size_t offset = 12;
  for (const auto& point : points) {
    const float intensity = static_cast<float>(point.intensity);
    append(bytes, offset, point.x);
    append(bytes, offset + 4, point.y);
    append(bytes, offset + 8, point.z);
    append(bytes, offset + 12, intensity);
    offset += 16;
  }
  return bytes;
}

template <typename Points>
std::vector<char> serializeXyzitrr(
    std::uint32_t sequence, double timestamp, const Points& points) {
  const auto count = static_cast<std::uint32_t>(points.size());
  const std::uint32_t payload_bytes = count * 24U;
  std::vector<char> bytes(40U + payload_bytes, 0);
  std::memcpy(bytes.data(), "FLIO", 4);
  bytes[4] = 1;
  bytes[5] = 0;
  bytes[6] = 1;
  bytes[7] = 0x03;
  append(bytes, 8, static_cast<std::uint16_t>(40));
  append(bytes, 10, static_cast<std::uint16_t>(24));
  append(bytes, 12, sequence);
  append(bytes, 16, timestamp);
  append(bytes, 24, count);
  append(bytes, 28, payload_bytes);
  bytes[32] = 3;

  std::size_t offset = 40;
  for (const auto& point : points) {
    const float intensity = static_cast<float>(point.intensity);
    const float relative_time = static_cast<float>(point.timestamp - timestamp);
    append(bytes, offset, point.x);
    append(bytes, offset + 4, point.y);
    append(bytes, offset + 8, point.z);
    append(bytes, offset + 12, intensity);
    append(bytes, offset + 16, relative_time);
    append(bytes, offset + 20, point.ring);
    offset += 24;
  }
  return bytes;
}

}  // namespace rslidar_dora
