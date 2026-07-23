/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING FROM, ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include "source/source.hpp"
#include <mutex>
#include <vector>
#include <cstring>
#include <nlohmann/json.hpp>

extern "C" {
#include "node_api.h"
}

namespace robosense {
namespace lidar {

/**
 * @brief Dora适配层 - 点云输出
 *
 * 参考Livox驱动的实现：
 * - 点云使用二进制格式发送（避免JSON的性能开销）
 * - IMU使用JSON格式发送（数据量小）
 * - 使用互斥锁保证线程安全（Dora C API不是线程安全的）
 */
class DestinationPointCloudDora : public DestinationPointCloud {
public:
  DestinationPointCloudDora(void* dora_context);

  virtual void init(const YAML::Node& config) override;
  virtual void sendPointCloud(const LidarPointCloudMsg& msg) override;
#ifdef ENABLE_IMU_DATA_PARSE
  virtual void sendImuData(const std::shared_ptr<ImuData>& data) override;
#endif
  virtual ~DestinationPointCloudDora() = default;

private:
  // 将点云序列化为二进制格式
  std::vector<char> serializePointCloudToBinary(const LidarPointCloudMsg& msg);

#ifdef ENABLE_IMU_DATA_PARSE
  // 将IMU数据序列化为JSON格式
  nlohmann::json serializeImuToJson(const ImuData& imu_data);
#endif

  void* dora_context_;              // Dora上下文
  std::mutex send_mutex_;           // 保护dora_send_output调用的互斥锁
  std::string pointcloud_output_;   // 点云输出名称
  std::string imu_output_;          // IMU输出名称
  std::string frame_id_;            // 坐标系ID
};

inline DestinationPointCloudDora::DestinationPointCloudDora(void* dora_context)
    : dora_context_(dora_context) {
}

inline void DestinationPointCloudDora::init(const YAML::Node& config) {
  // 固定输出topic名称，与Livox驱动保持一致
  pointcloud_output_ = "pointcloud";
  imu_output_ = "imu";
  frame_id_ = "rslidar_frame";  // 固定frame_id

  RS_INFO << "Dora PointCloud Output: " << pointcloud_output_ << RS_REND;
  RS_INFO << "Dora IMU Output: " << imu_output_ << RS_REND;
  RS_INFO << "Frame ID: " << frame_id_ << RS_REND;
}

inline void DestinationPointCloudDora::sendPointCloud(const LidarPointCloudMsg& msg) {
  if (dora_context_ == nullptr) {
    RS_ERROR << "Dora context is null" << RS_REND;
    return;
  }

  if (msg.points.empty()) {
    RS_WARNING << "Point cloud is empty, skip sending" << RS_REND;
    return;
  }

  // 序列化为二进制格式
  std::vector<char> buf = serializePointCloudToBinary(msg);

  // 线程安全发送
  std::lock_guard<std::mutex> lock(send_mutex_);
  int result = dora_send_output(
      dora_context_,
      const_cast<char*>(pointcloud_output_.c_str()),
      pointcloud_output_.length(),
      buf.data(),
      buf.size()
  );

  if (result != 0) {
    RS_ERROR << "Failed to send point cloud to Dora: " << pointcloud_output_ << RS_REND;
  }
}

#ifdef ENABLE_IMU_DATA_PARSE
inline void DestinationPointCloudDora::sendImuData(const std::shared_ptr<ImuData>& data) {
  if (dora_context_ == nullptr) {
    RS_ERROR << "Dora context is null" << RS_REND;
    return;
  }

  if (!data || !data->state) {
    return;
  }

  // 序列化为JSON
  nlohmann::json json_data = serializeImuToJson(*data);
  std::string json_string = json_data.dump();

  // 线程安全发送
  std::lock_guard<std::mutex> lock(send_mutex_);
  int result = dora_send_output(
      dora_context_,
      const_cast<char*>(imu_output_.c_str()),
      imu_output_.length(),
      const_cast<char*>(json_string.c_str()),
      json_string.length()
  );

  if (result != 0) {
    RS_ERROR << "Failed to send IMU data to Dora: " << imu_output_ << RS_REND;
  }
}
#endif

/**
 * @brief 点云二进制序列化（统一XYZI格式，与Livox驱动一致）
 *
 * 格式（小端序，16字节/点）:
 *   [0..7]    double   timestamp       (秒)
 *   [8..11]   uint32_t num_points      (点数量)
 *   [12..]    N × 16 bytes: float32 x, y, z, intensity
 */
inline std::vector<char> DestinationPointCloudDora::serializePointCloudToBinary(
    const LidarPointCloudMsg& msg) {

  const uint32_t num_points = static_cast<uint32_t>(msg.points.size());
  constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t);  // 12 bytes
  constexpr size_t POINT_SIZE  = 4 * sizeof(float);                  // 16 bytes
  const size_t total_size = HEADER_SIZE + num_points * POINT_SIZE;

  std::vector<char> buf(total_size);
  char* ptr = buf.data();

  // 写入时间戳（秒）
  double timestamp = msg.timestamp;
  std::memcpy(ptr, &timestamp, sizeof(double));
  ptr += sizeof(double);

  // 写入点数量
  std::memcpy(ptr, &num_points, sizeof(uint32_t));
  ptr += sizeof(uint32_t);

  // 写入点数据（XYZI格式）
  for (uint32_t i = 0; i < num_points; ++i) {
    const auto& point = msg.points[i];
    float vals[4] = {
      point.x,
      point.y,
      point.z,
      static_cast<float>(point.intensity)
    };
    std::memcpy(ptr, vals, POINT_SIZE);
    ptr += POINT_SIZE;
  }

  return buf;
}

#ifdef ENABLE_IMU_DATA_PARSE
/**
 * @brief IMU数据JSON序列化（与Livox驱动格式保持一致）
 *
 * JSON格式：
 * {
 *   "header": {
 *     "frame_id": "rslidar",
 *     "timestamp": 1234567890.123
 *   },
 *   "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
 *   "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0}
 * }
 */
inline nlohmann::json DestinationPointCloudDora::serializeImuToJson(const ImuData& imu_data) {
  nlohmann::json json_data;

  json_data["header"]["frame_id"] = frame_id_;
  json_data["header"]["timestamp"] = imu_data.timestamp;

  json_data["angular_velocity"]["x"] = imu_data.angular_velocity_x;
  json_data["angular_velocity"]["y"] = imu_data.angular_velocity_y;
  json_data["angular_velocity"]["z"] = -imu_data.angular_velocity_z;  // Invert Z: AIRY reports opposite sign for yaw rate

  // AIRY IMU coordinate system: Z-axis points down, invert only Z to match standard (Z-up).
  // X and Y are horizontal accelerations and should not be inverted.
  json_data["linear_acceleration"]["x"] = imu_data.linear_acceleration_x;
  json_data["linear_acceleration"]["y"] = imu_data.linear_acceleration_y;
  json_data["linear_acceleration"]["z"] = -imu_data.linear_acceleration_z;

  return json_data;
}
#endif

}  // namespace lidar
}  // namespace robosense
