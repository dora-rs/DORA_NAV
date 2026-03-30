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
extern "C"
{
#include "node_api.h"   
// #include "operator_api.h"
// #include "operator_types.h"
}


#include "lddc_dora.h"
#include "comm/ldq.h"
#include "comm/comm.h"

#include <inttypes.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdint.h>
#include <future>
#include <thread>
#include <chrono>
#include <time.h>
#include <sys/time.h>


#include "lds_lidar.h"

namespace livox_ros {

/** Lidar Data Distribute Control--------------------------------------------*/

Lddc::Lddc(int format, int multi_topic, int data_src, 
    double frq, std::string &frame_id)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      publish_frq_(frq),
      frame_id_(frame_id){
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
}


Lddc::~Lddc() {
  PrepareExit();
  std::cout << "lddc destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
}

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::DistributePointCloudData(void *dora_context) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributePointCloudData is RequestExit" << std::endl;
    return;
  }
  lds_->pcd_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar, dora_context);    
  }
}

void Lddc::DistributeImuData(void *dora_context) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributeImuData is RequestExit" << std::endl;
    return;
  }
  
  lds_->imu_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarImuDataQueue *p_queue = &lidar->imu_data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarImuData(lidar_id, lidar, dora_context);
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar, void *dora_context) {
  LidarDataQueue *p_queue = &lidar->data;
  
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    std::cout << "this function already return" << std::endl;
    return;
  }
  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    while(!QueueIsEmpty(p_queue)) {
      auto start_time = std::chrono::high_resolution_clock::now();
      StoragePacket pkg;
      QueuePop(p_queue, &pkg);
      if (pkg.points.empty()) {
        printf("Publish point cloud2 failed, the pkg points is empty.\n");
        continue;
      }
      uint64_t timestamp = 0;
      if (!pkg.points.empty()) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        timestamp = static_cast<uint64_t>(tv.tv_sec) * 1e6 + static_cast<uint64_t>(tv.tv_usec);
        // std::cout << std::fixed << std::setprecision(6)<< "start : " << timestamp*1e-6 << std::endl;
      }
      static uint32_t msg_seq = 0;
      uint32_t points_num = pkg.points_num;
      size_t all_size = 16 + points_num * 16;
      uint8_t *point_data_ptr = new uint8_t[all_size];
      uint32_t* seq_ptr = (uint32_t*)point_data_ptr;
      *seq_ptr = msg_seq;
      ++msg_seq;
      uint64_t* timestamp_ptr = (uint64_t*)(point_data_ptr + 8);
      *timestamp_ptr = timestamp;
      for (uint32_t i = 0; i < points_num; ++i) {
        float* data_float = (float*)(point_data_ptr + 16 + 16*i);
        *data_float = pkg.points[i].x;
        data_float = (float*)(point_data_ptr + 16+4+16*i);
        *data_float = pkg.points[i].y;
        data_float = (float*)(point_data_ptr + 16+8+16*i);
        *data_float = pkg.points[i].z;
        data_float = (float*)(point_data_ptr + 16+12+16*i);
        *data_float = pkg.points[i].intensity;
      }
      char *output_data = (char *)point_data_ptr;
      size_t output_data_len = ((points_num + 1) * 16);
      std::string out_id = "pointcloud_livox";
      
      auto end_time = std::chrono::high_resolution_clock::now();
      auto elapsed_time =std::chrono::duration_cast<std::chrono::nanoseconds>(end_time-start_time);
      // std::cout << "the time is spent :" << elapsed_time.count() << "ns" << std::endl;
      int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);
      delete[] point_data_ptr;
      if (result != 0)
      {
        std::cerr << "LidarRawObject: failed to send output" << std::endl;
      }
      // std::cout << "success send out !!!! "<< std::endl;
                
                // struct timeval tv;
                // gettimeofday(&tv, NULL);//获取时间
                // auto start = tv.tv_sec + tv.tv_usec * 1e-6;
                // std::cout << std::fixed << std::setprecision(6)<< "start : " << start << std::endl;
    }
    // }
  }
  //std::cout << "exit !!!!!!!!!!" << std::endl;
}

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar, void *dora_context) {
  LidarImuDataQueue& p_queue = lidar->imu_data;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
  }
}

void Lddc::PrepareExit(void) {
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

// void Lddc::InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index) {
//   livox_msg.header.frame_id.assign(frame_id_);

//   uint64_t timestamp = 0;
//   if (!pkg.points.empty()) {
//     timestamp = pkg.base_time;
//   }
//   livox_msg.timebase = timestamp;

//   livox_msg.point_num = pkg.points_num;
//   if (lds_->lidars_[index].lidar_type == kLivoxLidarType) {
//     livox_msg.lidar_id = lds_->lidars_[index].handle;
//   } else {
//     printf("Init custom msg lidar id failed, the index:%u.\n", index);
//     livox_msg.lidar_id = 0;
//   }
// }

// void Lddc::FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg) {
//   uint32_t points_num = pkg.points_num;
//   const std::vector<PointXyzlt>& points = pkg.points;
//   for (uint32_t i = 0; i < points_num; ++i) {
//     CustomPoint point;
//     point.x = points[i].x;
//     point.y = points[i].y;
//     point.z = points[i].z;
//     point.reflectivity = points[i].intensity;
//     point.tag = points[i].tag;
//     point.line = points[i].line;
//     point.offset_time = static_cast<uint32_t>(points[i].offset_time - pkg.base_time);

//     livox_msg.points.push_back(std::move(point));
//   }
// }



// void Lddc::InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp) {
//   imu_msg.header.frame_id = "livox_frame";

//   timestamp = imu_data.time_stamp;

//   imu_msg.angular_velocity.x = imu_data.gyro_x;
//   imu_msg.angular_velocity.y = imu_data.gyro_y;
//   imu_msg.angular_velocity.z = imu_data.gyro_z;
//   imu_msg.linear_acceleration.x = imu_data.acc_x;
//   imu_msg.linear_acceleration.y = imu_data.acc_y;
//   imu_msg.linear_acceleration.z = imu_data.acc_z;
// }

// void Lddc::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {
//   ImuData imu_data;
//   if (!imu_data_queue.Pop(imu_data)) {
//     //printf("Publish imu data failed, imu data queue pop failed.\n");
//     return;
//   }

//   ImuMsg imu_msg;
//   uint64_t timestamp;
//   InitImuMsg(imu_data, imu_msg, timestamp);

// }



}  // namespace livox_ros
