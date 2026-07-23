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

#ifndef LIVOX_DORA_DRIVER2_LDDC_H_
#define LIVOX_DORA_DRIVER2_LDDC_H_

#include "include/livox_ros_driver2.h"
#include "dora_node.h"
#include "lds.h"
#include <atomic>
#include <future>
#include <memory>
#include <thread>
#include <nlohmann/json.hpp>

namespace livox_ros {

/** Send pointcloud message Data to dora subscriber */
typedef enum {
  kOutputToRos = 0,  // Reuse this enum for Dora output
} DestinationOfMessageOutput;

/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
} TransferType;

class DoraNode;

/**
 * @brief Lidar Data Distribute Control for Dora
 *
 * Hardware-driven design (mirrors the official livox_ros_driver2):
 *   - PcdPollThread:  blocks on pcd_semaphore_, publishes point clouds at
 *                     whatever rate the hardware produces them (typically 10 Hz)
 *   - ImuPollThread:  blocks on imu_semaphore_, publishes IMU frames at
 *                     whatever rate the hardware produces them (200 Hz for MID360)
 *
 * Neither thread depends on a Dora timer.  The Dora event loop in the main
 * thread only needs to wait for DoraEventType_Stop to perform a clean shutdown.
 */
class LddcDora final {
 public:
  LddcDora(int format, int multi_topic, int data_src, int output_type, double frq,
      std::string &frame_id, DoraNode* dora_node);
  ~LddcDora();

  int  RegisterLds(Lds *lds);

  /**
   * @brief Start the point-cloud and IMU polling threads.
   * Must be called after RegisterLds() and after the lidar SDK has been
   * initialised (i.e. after LdsLidar::InitLdsLidar succeeds).
   */
  void StartPollThreads();

  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void)      { return use_multi_topic_; }
  void    SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

 public:
  Lds *lds_;

 private:
  // ── thread bodies (mirror PointCloudDataPollThread / ImuDataPollThread) ──
  void PcdPollThread();
  void ImuPollThread();

  // ── per-lidar helpers ────────────────────────────────────────────────────
  void PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar);
  void PollingLidarImuData(uint8_t index, LidarDevice *lidar);

  void PublishPointcloud2(LidarDataQueue *queue, uint8_t index);
  void PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index);

  // Point cloud: binary serialisation (avoids JSON text overhead for large payloads)
  std::vector<char> SerializePointCloudToBinary(const StoragePacket& pkg);
  void PublishPointCloudBinary(const std::vector<char>& buf, const uint8_t index);

  // IMU: JSON is fine — payload is tiny (~200 bytes/frame)
  nlohmann::json SerializeImuToJson(const ImuData& imu_data);
  void PublishImuJson(const nlohmann::json& json_data, const uint8_t index);

  std::string GetPointCloudTopicName(uint8_t index);
  std::string GetImuTopicName(uint8_t index);

 private:
  uint8_t  transfer_format_;
  uint8_t  use_multi_topic_;
  uint8_t  data_src_;
  uint8_t  output_type_;
  double   publish_frq_;
  uint32_t publish_period_ns_;
  std::string frame_id_;

  DoraNode* dora_node_;

  // ── background threads (same lifecycle pattern as official driver) ────────
  std::shared_ptr<std::thread> pcd_thread_;
  std::shared_ptr<std::thread> imu_thread_;
  std::promise<void>           exit_signal_;
  std::shared_future<void>     future_;
};

}  // namespace livox_ros

#endif // LIVOX_DORA_DRIVER2_LDDC_H_
