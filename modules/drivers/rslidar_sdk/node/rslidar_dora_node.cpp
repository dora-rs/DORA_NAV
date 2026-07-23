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
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING FROM, OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include "manager/node_manager.hpp"

#include <rs_driver/macro/version.hpp>
#include <signal.h>
#include <iostream>

extern "C" {
#include "node_api.h"
}

using namespace robosense::lidar;

static volatile bool g_running = true;

static void sigHandler(int sig) {
  RS_MSG << "RoboSense-LiDAR-Dora-Driver is stopping....." << RS_REND;
  g_running = false;
}

int main(int argc, char** argv) {
  signal(SIGINT, sigHandler);

  RS_TITLE << "********************************************************" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR
    << "." << RSLIDAR_VERSION_MINOR
    << "." << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
  RS_TITLE << "**********           (Dora Mode)              **********" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "********************************************************" << RS_REND;

  // ──────────────────────────────────────────────────────────────────────────
  // 初始化Dora上下文
  // ──────────────────────────────────────────────────────────────────────────
  void* dora_context = init_dora_context_from_env();
  if (dora_context == nullptr) {
    RS_ERROR << "Failed to initialize Dora context from environment" << RS_REND;
    return -1;
  }
  RS_INFO << "Dora context initialized successfully" << RS_REND;

  // ──────────────────────────────────────────────────────────────────────────
  // 加载配置文件
  // ──────────────────────────────────────────────────────────────────────────
  std::string config_path = "../modules/drivers/rslidar_sdk/config/config.yaml";

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
    RS_INFO << "--------------------------------------------------------" << RS_REND;
    RS_INFO << "Config loaded from PATH:" << RS_REND;
    RS_INFO << config_path << RS_REND;
    RS_INFO << "--------------------------------------------------------" << RS_REND;
  } catch (...) {
    RS_ERROR << "The format of config file " << config_path
             << " is wrong. Please check (e.g. indentation)." << RS_REND;
    free_dora_context(dora_context);
    return -1;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 初始化并启动节点管理器
  // ──────────────────────────────────────────────────────────────────────────
  std::shared_ptr<NodeManager> manager_ptr = std::make_shared<NodeManager>(dora_context);
  manager_ptr->init(config);
  manager_ptr->start();

  RS_MSG << "RoboSense-LiDAR-Dora-Driver is running....." << RS_REND;

  // ──────────────────────────────────────────────────────────────────────────
  // Dora事件循环 - 等待Stop事件以实现干净关闭
  // ──────────────────────────────────────────────────────────────────────────
  // 由于数据发布由rs_driver的内部线程处理，主循环只需保持进程运行
  // 并响应Dora的停止信号
  RS_INFO << "Entering Dora event loop (waiting for Stop event)..." << RS_REND;

  while (g_running) {
    void* event = dora_next_event(dora_context);
    if (event == nullptr) {
      RS_ERROR << "Unexpected end of Dora event" << RS_REND;
      break;
    }

    enum DoraEventType event_type = read_dora_event_type(event);

    if (event_type == DoraEventType_Stop) {
      RS_MSG << "Received Dora stop event - shutting down gracefully" << RS_REND;
      free_dora_event(event);
      break;
    }

    free_dora_event(event);
    // 其他事件类型被忽略（此节点没有输入）
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 清理资源
  // ──────────────────────────────────────────────────────────────────────────
  manager_ptr->stop();
  manager_ptr.reset();

  free_dora_context(dora_context);

  RS_MSG << "RoboSense-LiDAR-Dora-Driver finished successfully" << RS_REND;
  return 0;
}
