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

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>
#include <string>
#include "livox_ros_driver2.h"
#include "dora_node.h"
#include "lddc_dora.h"
#include "lds_lidar.h"

extern "C"{
#include "node_api.h"
}

using namespace livox_ros;

int main(int argc, char **argv) {
  std::cout << "Livox Dora Driver Version: " << LIVOX_ROS_DRIVER2_VERSION_STRING << std::endl;

  // ── Dora context ──────────────────────────────────────────────────────────
  void *dora_context = init_dora_context_from_env();
  if (dora_context == nullptr) {
    std::cerr << "Failed to initialize Dora context" << std::endl;
    return -1;
  }
  std::cout << "Dora context initialized successfully" << std::endl;

  livox_ros::DoraNode dora_node(dora_context);

  // ── Driver parameters ─────────────────────────────────────────────────────
  int         xfer_format  = kPointCloud2Msg;
  int         multi_topic  = 0;
  int         data_src     = kSourceRawLidar;
  double      publish_freq = 10.0;   // point-cloud rate declared to the SDK (Hz)
  int         output_type  = kOutputToRos;
  std::string frame_id     = "livox_frame";

  std::cout << "Point-cloud publish frequency: " << publish_freq << " Hz" << std::endl;
  std::cout << "IMU publish rate: hardware-driven (200 Hz for MID360)" << std::endl;

  // ── Create LDDC and register LDS ─────────────────────────────────────────
  dora_node.lddc_ptr_ = std::make_unique<LddcDora>(
      xfer_format, multi_topic, data_src, output_type,
      publish_freq, frame_id, &dora_node);

  if (data_src != kSourceRawLidar) {
    std::cerr << "Invalid data src (" << data_src << "), only raw lidar is supported" << std::endl;
    free_dora_context(dora_context);
    return -1;
  }

  std::cout << "Data Source is raw lidar." << std::endl;

  std::string user_config_path =
      "../modules/drivers/livox_driver/config/MID360_config.json";
  std::cout << "Config file: " << user_config_path << std::endl;

  LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
  dora_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

  if (!read_lidar->InitLdsLidar(user_config_path)) {
    std::cerr << "Init lds lidar failed!" << std::endl;
    free_dora_context(dora_context);
    return -1;
  }
  std::cout << "Init lds lidar successfully!" << std::endl;

  // ── Start hardware-driven poll threads (mirrors official ROS driver) ──────
  //
  //   PcdPollThread:  blocks on pcd_semaphore_, wakes when the SDK has a
  //                   complete point-cloud frame ready (~10 Hz for MID360).
  //
  //   ImuPollThread:  blocks on imu_semaphore_, wakes when the SDK has new
  //                   IMU samples ready (200 Hz for MID360).
  //
  //   No Dora timer is involved — both threads are fully hardware-triggered.
  //
  dora_node.lddc_ptr_->StartPollThreads();

  // ── Dora event loop — only needs to handle Stop ───────────────────────────
  //
  // Because data publishing is handled entirely by the two background threads,
  // the main loop has nothing to do except keep the process alive and react to
  // the Dora stop signal for a clean shutdown.
  //
  std::cout << "Entering Dora event loop (waiting for Stop event)..." << std::endl;

  while (true) {
    void *event = dora_next_event(dora_context);
    if (event == nullptr) {
      std::cerr << "ERROR: unexpected end of event" << std::endl;
      break;
    }

    enum DoraEventType ty = read_dora_event_type(event);
    free_dora_event(event);
    
    if (ty == DoraEventType_Stop) {
      std::cout << "Received Dora stop event — shutting down." << std::endl;
      break;
    }
    // Any other event type (none expected since we have no inputs) is ignored.
  }

  // ── Cleanup ───────────────────────────────────────────────────────────────
  // ~LddcDora() will signal the poll threads to exit and join them.
  // ~DoraNode() will free the Dora context.
  std::cout << "Livox Dora Driver finished successfully." << std::endl;
  return 0;
}
