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

#ifndef LIVOX_ROS_DRIVER2_LDDC_H_
#define LIVOX_ROS_DRIVER2_LDDC_H_



#include "lds.h"

namespace livox_ros {



/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
} TransferType;




class Lddc final {
 public:
  Lddc(int format, int multi_topic, int data_src, 
    double frq, std::string &frame_id);
  ~Lddc();

  int RegisterLds(Lds *lds);
  void DistributePointCloudData(void *dora_context);
  void DistributeImuData(void *dora_context);
  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void) { return use_multi_topic_; }


  // void SetRosPub(ros::Publisher *pub) { global_pub_ = pub; };  // NOT USED
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

 public:
  Lds *lds_;

 private:
  void PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar, void *dora_context);
  void PollingLidarImuData(uint8_t index, LidarDevice *lidar, void *dora_context);

  // void PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index);


  // void PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index);


  // void InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index);
  // void FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg);
  // void PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index);


  // void InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp);


  // void FillPointsToCustomMsg(CustomMsg& livox_msg, LivoxPointXyzrtlt* src_point, uint32_t num,
  //     uint32_t offset_time, uint32_t point_interval, uint32_t echo_num);


 private:
  uint8_t transfer_format_;
  uint8_t use_multi_topic_;
  uint8_t data_src_;
  double publish_frq_;
  uint32_t publish_period_ns_;
  std::string frame_id_;
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER2_LDDC_H_
