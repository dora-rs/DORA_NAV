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

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>
#include <future>

#include "lddc_dora.h"
#include "lds_lidar.h"
#include "comm/ldq.h"

using namespace livox_ros;
std::unique_ptr<Lddc> lddc_ptr_;
std::shared_ptr<std::thread> pointclouddata_poll_thread_;
std::shared_ptr<std::thread> imudata_poll_thread_;
std::shared_future<void> future_;
std::promise<void> exit_signal_;

// std::shared_ptr<std::thread> PointCloudDataPollThread(void *dora_context) {
//     return std::make_shared<std::thread>([dora_context] {
//         std::future_status status;
//         std::this_thread::sleep_for(std::chrono::seconds(3));
//         do {
//             lddc_ptr_->DistributePointCloudData(dora_context);
//             status = future_.wait_for(std::chrono::microseconds(0));
//         } while (status == std::future_status::timeout);
//     });
// }

// std::shared_ptr<std::thread> ImuDataPollThread(void *dora_context) {
//     return std::make_shared<std::thread>([dora_context] {
//         std::future_status status;
//         std::this_thread::sleep_for(std::chrono::seconds(3));
//         do {
//             lddc_ptr_->DistributeImuData(dora_context);
//             status = future_.wait_for(std::chrono::microseconds(0));
//         } while (status == std::future_status::timeout);
//     });
// }

int run(void *dora_context){

  while(!lddc_ptr_->lds_->IsRequestExit()){
    void *event = dora_next_event(dora_context);
    if(event == NULL){
      printf("[c node] ERROR: unexpected end of event\n");
      return -1;
    }

    enum DoraEventType ty = read_dora_event_type(event);

    if(ty == DoraEventType_Input){
     lddc_ptr_->DistributePointCloudData(dora_context);
    }
    else if(ty == DoraEventType_Stop){
      printf("[c node] recevied stop event\n");
      lddc_ptr_->lds_->RequestExit();
    }
    else{
      printf("[c node] recevied unexpected event: %d\n", ty);
    }
    free_dora_event(event);
  }
  
  return 0;
    
  
}

int main(int argc, char **argv) {
  
  /** Init default system parameter */
  int xfer_format = kLivoxCustomMsg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  std::string frame_id = "livox_frame";

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, 
                                    publish_freq, frame_id);//driver->start()
  

  if (data_src == kSourceRawLidar) {

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar("livox/livox_dora_driver2/config/MID360_config.json"))) {
      std::cout << "Init lds lidar successfully!" << std::endl;
    } else {
      std::cout << "Init lds lidar failed!" << std::endl;
    }
  } else {
    std::cout <<  "Invalid data src" << data_src << "please check the launch file" << std::endl;
  }

  auto dora_context = init_dora_context_from_env();
  
  // pointclouddata_poll_thread_ = PointCloudDataPollThread(dora_context);
  // imudata_poll_thread_ = ImuDataPollThread(dora_context);
  auto ret = run(dora_context);
  free_dora_context(dora_context);

  lddc_ptr_->lds_->RequestExit();//driver->close()
  // exit_signal_.set_value();
  // if (pointclouddata_poll_thread_) {
  //   pointclouddata_poll_thread_->join();
  // }
  // if (imudata_poll_thread_) {
  //   imudata_poll_thread_->join();
  // }
  return 0;
}

























