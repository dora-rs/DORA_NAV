//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc_dora.h"
#include "comm/ldq.h"
#include "comm/comm.h"
#include "dora_node.h"
#include "lds_lidar.h"

#include <inttypes.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdint.h>
#include <chrono>
#include <cstring>
#include <vector>
#include <nlohmann/json.hpp>

namespace livox_ros {

/** Lidar Data Distribute Control for Dora--------------------------------------------*/
LddcDora::LddcDora(int format, int multi_topic, int data_src, int output_type,
    double frq, std::string &frame_id, DoraNode* dora_node)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id),
      dora_node_(dora_node),
      lds_(nullptr),
      future_(exit_signal_.get_future()) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  std::cout << "LddcDora initialized with publish frequency: " << publish_frq_ << " Hz" << std::endl;
}

LddcDora::~LddcDora() {
  PrepareExit();

  // Signal threads to stop, then wait for them
  try { exit_signal_.set_value(); } catch (...) {}

  if (pcd_thread_ && pcd_thread_->joinable()) {
    pcd_thread_->join();
  }
  if (imu_thread_ && imu_thread_->joinable()) {
    imu_thread_->join();
  }

  std::cout << "LddcDora destroyed" << std::endl;
}

int LddcDora::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

/**
 * @brief Start the point-cloud and IMU polling threads.
 *
 * Mirrors the official driver's PointCloudDataPollThread / ImuDataPollThread:
 *   - Both threads sleep 3 s on startup to let the SDK connect to the lidar.
 *   - Each then loops: block on its semaphore → drain the queue → repeat.
 *   - Exit when future_ becomes ready (set_value() called in destructor).
 */
void LddcDora::StartPollThreads() {
  pcd_thread_ = std::make_shared<std::thread>(&LddcDora::PcdPollThread, this);
  imu_thread_ = std::make_shared<std::thread>(&LddcDora::ImuPollThread, this);
  std::cout << "[LddcDora] Point-cloud and IMU poll threads started." << std::endl;
}

// ── Point-cloud poll thread ────────────────────────────────────────────────
/**
 * Exact mirror of DriverNode::PointCloudDataPollThread() in the official driver:
 *   sleep 3 s → loop { Wait(pcd_semaphore) → drain queue } until exit signal.
 */
void LddcDora::PcdPollThread() {
  std::this_thread::sleep_for(std::chrono::seconds(3));
  std::cout << "[PcdPollThread] Starting point-cloud polling loop." << std::endl;

  std::future_status status;
  do {
    if (!lds_ || lds_->IsRequestExit()) break;

    lds_->pcd_semaphore_.Wait();

    for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
      LidarDevice *lidar = &lds_->lidars_[i];
      LidarDataQueue *p_queue = &lidar->data;
      if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
        continue;
      }
      PollingLidarPointCloudData(static_cast<uint8_t>(i), lidar);
    }

    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);

  std::cout << "[PcdPollThread] Exiting." << std::endl;
}

// ── IMU poll thread ────────────────────────────────────────────────────────
/**
 * Exact mirror of DriverNode::ImuDataPollThread() in the official driver:
 *   sleep 3 s → loop { Wait(imu_semaphore) → drain queue } until exit signal.
 *
 * MID360 produces IMU data at 200 Hz — this thread wakes up 200 times/second,
 * completely independent of any Dora timer.
 */
void LddcDora::ImuPollThread() {
  std::this_thread::sleep_for(std::chrono::seconds(3));
  std::cout << "[ImuPollThread] Starting IMU polling loop (200 Hz hardware rate)." << std::endl;

  std::future_status status;
  do {
    if (!lds_ || lds_->IsRequestExit()) break;

    lds_->imu_semaphore_.Wait();

    for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
      LidarDevice *lidar = &lds_->lidars_[i];
      LidarImuDataQueue *p_queue = &lidar->imu_data;
      if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
        continue;
      }
      PollingLidarImuData(static_cast<uint8_t>(i), lidar);
    }

    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);

  std::cout << "[ImuPollThread] Exiting." << std::endl;
}

// ── Per-lidar helpers ──────────────────────────────────────────────────────

void LddcDora::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }

  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    PublishPointcloud2(p_queue, index);
  }
}

void LddcDora::PollingLidarImuData(uint8_t index, LidarDevice *lidar) {
  LidarImuDataQueue& p_queue = lidar->imu_data;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishImuData(p_queue, index);
  }
}

void LddcDora::PrepareExit(void) {
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

// ── Publish helpers ────────────────────────────────────────────────────────

void LddcDora::PublishPointcloud2(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud2 failed, the pkg points is empty.\n");
      continue;
    }

    std::vector<char> buf = SerializePointCloudToBinary(pkg);
    PublishPointCloudBinary(buf, index);
  }
}

void LddcDora::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {
  ImuData imu_data;
  if (!imu_data_queue.Pop(imu_data)) {
    return;
  }

  nlohmann::json json_data = SerializeImuToJson(imu_data);
  PublishImuJson(json_data, index);
}

// ── Serialisation ──────────────────────────────────────────────────────────

/**
 * Binary wire format (little-endian, packed):
 *   [0..7]   double   timestamp      (seconds, base_time / 1e9)
 *   [8..11]  uint32_t num_points
 *   [12..]   N × 16 bytes: float32 x, y, z, intensity
 */
std::vector<char> LddcDora::SerializePointCloudToBinary(const StoragePacket& pkg) {
  const uint32_t n = static_cast<uint32_t>(pkg.points_num);
  constexpr size_t HEADER_SIZE = sizeof(double) + sizeof(uint32_t); // 12 bytes
  constexpr size_t POINT_SIZE  = 4 * sizeof(float);                 // 16 bytes

  std::vector<char> buf(HEADER_SIZE + n * POINT_SIZE);
  char* ptr = buf.data();

  double ts = pkg.base_time / 1e9;
  std::memcpy(ptr, &ts, sizeof(double));  ptr += sizeof(double);

  std::memcpy(ptr, &n, sizeof(uint32_t)); ptr += sizeof(uint32_t);

  for (uint32_t i = 0; i < n; ++i) {
    float vals[4] = {
      pkg.points[i].x,
      pkg.points[i].y,
      pkg.points[i].z,
      pkg.points[i].intensity
    };
    std::memcpy(ptr, vals, POINT_SIZE);
    ptr += POINT_SIZE;
  }

  return buf;
}

nlohmann::json LddcDora::SerializeImuToJson(const ImuData& imu_data) {
  nlohmann::json json_data;

  json_data["header"]["frame_id"]  = frame_id_;
  json_data["header"]["timestamp"] = imu_data.time_stamp / 1e9;  // ns → s

  json_data["angular_velocity"]["x"] = imu_data.gyro_x;
  json_data["angular_velocity"]["y"] = imu_data.gyro_y;
  json_data["angular_velocity"]["z"] = imu_data.gyro_z;

  // acc unit from hardware: g  (conversion to m/s² is done in the localization node)
  json_data["linear_acceleration"]["x"] = imu_data.acc_x;
  json_data["linear_acceleration"]["y"] = imu_data.acc_y;
  json_data["linear_acceleration"]["z"] = imu_data.acc_z;

  return json_data;
}

// ── Dora output helpers ────────────────────────────────────────────────────

void LddcDora::PublishPointCloudBinary(const std::vector<char>& buf, const uint8_t index) {
  if (!dora_node_) {
    std::cerr << "Dora node is null" << std::endl;
    return;
  }

  std::string topic_name = GetPointCloudTopicName(index);
  int result = dora_node_->SendOutput(topic_name, std::string(buf.data(), buf.size()));
  if (result != 0) {
    std::cerr << "Failed to publish binary point cloud to topic: " << topic_name << std::endl;
  }
}

void LddcDora::PublishImuJson(const nlohmann::json& json_data, const uint8_t index) {
  if (!dora_node_) {
    std::cerr << "Dora node is null" << std::endl;
    return;
  }

  std::string topic_name  = GetImuTopicName(index);
  std::string json_string = json_data.dump();

  int result = dora_node_->SendOutput(topic_name, json_string);
  if (result != 0) {
    std::cerr << "Failed to publish IMU data to topic: " << topic_name << std::endl;
  }
}

// ── Topic name helpers ─────────────────────────────────────────────────────

std::string LddcDora::GetPointCloudTopicName(uint8_t index) {
  if (use_multi_topic_) {
    return "pointcloud_" + std::to_string(index);
  }
  return "pointcloud";
}

std::string LddcDora::GetImuTopicName(uint8_t index) {
  if (use_multi_topic_) {
    return "imu_" + std::to_string(index);
  }
  return "imu";
}

}  // namespace livox_ros
