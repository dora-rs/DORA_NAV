#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace livox_dora {

enum class PointCloudFormat { XYZI, XYZITRR, MID360 };

inline PointCloudFormat parsePointCloudFormat(const char* value) {
  if (value == nullptr || value[0] == '\0' || std::string(value) == "XYZI") {
    return PointCloudFormat::XYZI;
  }
  if (std::string(value) == "XYZITRR") {
    return PointCloudFormat::XYZITRR;
  }
  if (std::string(value) == "MID360") {
    return PointCloudFormat::MID360;
  }
  throw std::invalid_argument(
      "POINT_CLOUD_FORMAT must be XYZI, XYZITRR, or MID360 (received '" +
      std::string(value) + "')");
}

inline const char* pointCloudFormatName(PointCloudFormat format) {
  switch (format) {
    case PointCloudFormat::XYZI: return "XYZI";
    case PointCloudFormat::XYZITRR: return "XYZITRR";
    case PointCloudFormat::MID360: return "MID360";
  }
  return "unknown";
}

template <typename T>
inline void append(std::vector<char>& bytes, std::size_t offset, const T& value) {
  std::memcpy(bytes.data() + offset, &value, sizeof(value));
}

template <typename Points>
std::vector<char> serializeXyzi(std::uint64_t base_time, const Points& points) {
  const auto count = static_cast<std::uint32_t>(points.size());
  std::vector<char> bytes(12U + static_cast<std::size_t>(count) * 16U, 0);
  const double timestamp = static_cast<double>(base_time) / 1e9;
  append(bytes, 0, timestamp);
  append(bytes, 8, count);
  std::size_t offset = 12;
  for (const auto& point : points) {
    append(bytes, offset, point.x);
    append(bytes, offset + 4, point.y);
    append(bytes, offset + 8, point.z);
    append(bytes, offset + 12, point.intensity);
    offset += 16;
  }
  return bytes;
}

template <typename Points>
std::vector<char> serializeXyzitrr(
    std::uint32_t sequence, std::uint64_t base_time, const Points& points) {
  const auto count = static_cast<std::uint32_t>(points.size());
  const std::uint32_t payload_bytes = count * 24U;
  std::vector<char> bytes(40U + payload_bytes, 0);
  const double timestamp = static_cast<double>(base_time) / 1e9;
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
    const std::uint64_t relative_ns = point.offset_time >= base_time
        ? point.offset_time - base_time : 0;
    const float relative_time = static_cast<float>(relative_ns) / 1e9F;
    const std::uint16_t ring = point.line;
    append(bytes, offset, point.x);
    append(bytes, offset + 4, point.y);
    append(bytes, offset + 8, point.z);
    append(bytes, offset + 12, point.intensity);
    append(bytes, offset + 16, relative_time);
    append(bytes, offset + 20, ring);
    offset += 24;
  }
  return bytes;
}

template <typename Points>
std::vector<char> serializeMid360(
    std::uint32_t sequence, std::uint64_t base_time, std::uint8_t lidar_id,
    const Points& points) {
  const auto count = static_cast<std::uint32_t>(points.size());
  std::vector<char> bytes(20U + static_cast<std::size_t>(count) * 24U, 0);
  const double timestamp = static_cast<double>(base_time) / 1e9;
  append(bytes, 0, sequence);
  append(bytes, 8, timestamp);
  bytes[16] = static_cast<char>(lidar_id);

  std::size_t offset = 20;
  for (const auto& point : points) {
    const std::uint64_t relative_ns = point.offset_time >= base_time
        ? point.offset_time - base_time : 0;
    const auto bounded_ns = static_cast<std::uint32_t>(std::min<std::uint64_t>(
        relative_ns, std::numeric_limits<std::uint32_t>::max()));
    append(bytes, offset, point.x);
    append(bytes, offset + 4, point.y);
    append(bytes, offset + 8, point.z);
    append(bytes, offset + 12, point.intensity);
    append(bytes, offset + 16, bounded_ns);
    bytes[offset + 20] = static_cast<char>(point.tag);
    bytes[offset + 21] = static_cast<char>(point.line);
    offset += 24;
  }
  return bytes;
}

}  // namespace livox_dora
