extern "C"
{
#include "node_api.h"
  // #include "operator_api.h"
  // #include "operator_types.h"
}

#include <iostream>
#include <vector>
#include <cstring>
#include <atomic> // 仅用于安全退出，不破坏原有结构
#include <thread> // 仅用于安全退出，不破坏原有结构

// rs lidar driver
#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

typedef struct Vec_uint8
{
  /** <No documentation available> */
  uint8_t *ptr;

  /** <No documentation available> */
  size_t len;

  /** <No documentation available> */
  size_t cap;
} Vec_uint8_t;

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace robosense::lidar;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

RSDriverParam param;
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);
}

std::string exceptionCallback(const Error &code)
{
  RS_WARNING << code.toString() << RS_REND;
  return "";
}

std::atomic<bool> to_exit_process{false};

void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    free_cloud_queue.push(msg);
  }
}

int run(void *dora_context)
{
  unsigned char counter = 0;

  // ========================= 修复核心 1 =========================
  // 启动独立消费线程，实时清空雷达队列，根治缓冲区溢出
  std::thread cloud_process_thread(processCloud);

  while (!to_exit_process)
  {
    void *event = dora_next_event(dora_context);
    if (event == NULL)
    {
      printf("[c node] ERROR: unexpected end of event\n");
      return -1;
    }

    enum DoraEventType ty = read_dora_event_type(event);

    if (ty == DoraEventType_Input)
    {
      std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
      if (msg.get() == NULL)
      {
        free_dora_event(event);
        continue;
      }

      const size_t points_count = msg->points.size();
      const size_t cloudSize = 16 + points_count * 16;
      std::vector<uint8_t> payload(cloudSize);

      uint32_t seq = msg->seq;
      uint64_t timestamp_us = static_cast<uint64_t>(msg->timestamp * 1e6);

      // std::cout << " [DEBUG] Preparing to send frame... Seq: " << seq
      //           << " | Points: " << points_count
      //           << " | Expected Size: " << cloudSize << " bytes" << std::endl;

      {
        uint32_t seq = msg->seq;
        std::memcpy(payload.data() + 0, &seq, sizeof(uint32_t));

        uint64_t timestamp_us = static_cast<uint64_t>(msg->timestamp * 1e6);
        std::memcpy(payload.data() + 8, &timestamp_us, sizeof(uint64_t));
      }

      for (size_t i = 0; i < points_count; ++i)
      {
        const auto &p = msg->points[i];
        float x = p.x;
        float y = p.y;
        float z = p.z;
        float intensity = static_cast<float>(p.intensity);

        const size_t base = 16 + i * 16;
        std::memcpy(payload.data() + base + 0, &x, sizeof(float));
        std::memcpy(payload.data() + base + 4, &y, sizeof(float));
        std::memcpy(payload.data() + base + 8, &z, sizeof(float));
        std::memcpy(payload.data() + base + 12, &intensity, sizeof(float));
      }

      free_cloud_queue.push(msg);

      char *output_data = reinterpret_cast<char *>(payload.data());
      size_t output_data_len = payload.size();
      counter += 1;

      std::string out_id = "pointcloud_rs";
      int resultend = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);

      if (resultend != 0)
      {
        std::cerr << "failed to send output" << std::endl;
        return 1;
      }
    }
    else if (ty == DoraEventType_Stop)
    {
      printf("[c node] received stop event\n");
      to_exit_process = true;
    }
    else
    {
      printf("[c node] received unexpected event: %d\n", ty);
    }

    free_dora_event(event);
  }

  if (cloud_process_thread.joinable())
  {
    cloud_process_thread.join();
  }

  return 0;
}

int main()
{
  std::cout << "rslidar driver for dora " << std::endl;
  auto dora_context = init_dora_context_from_env();

  const char *online_lidar_env = std::getenv("ONLINE_LIDAR");
  bool is_online_lidar = (online_lidar_env && std::string(online_lidar_env) == "1");
  if (is_online_lidar)
  {
    param.input_type = InputType::ONLINE_LIDAR;
    param.decoder_param.wait_for_difop = 1;
    std::cout << "Using ONLINE LIDAR! " << std::endl;
  }
  else
  {
    param.input_type = InputType::PCAP_FILE;
    std::cout << "Using PCAP FILE! " << std::endl;
    const char *pcap_path_env = std::getenv("PCAP_PATH");
    std::cout << "pcap_path_env: " << std::string(pcap_path_env) << std::endl;
    if (pcap_path_env)
    {
      param.input_param.pcap_path = std::string(pcap_path_env);
    }
    else
    {
      std::cerr << "Warning: PCAP_PATH is not set, but ONLINE_LIDAR=0. Please check." << std::endl;
      return -1;
    }
    param.decoder_param.wait_for_difop = 0;
  }

  param.input_param.msop_port = 6699;
  param.input_param.difop_port = 7788;
  param.input_param.host_address = "0.0.0.0";
  param.input_param.group_address = "0.0.0.0";
  param.input_param.use_vlan = false;
  std::cout << "param.input_param.msop_port: " << param.input_param.msop_port << std::endl
            << " param.input_param.difop_port: " << param.input_param.difop_port << std::endl;

  std::string lidar_type_env = std::getenv("LIDAR_TYPE") ? std::getenv("LIDAR_TYPE") : "RSHELIOS_16P";
  std::cout << "lidar_type_env: " << lidar_type_env << std::endl;

  param.lidar_type = strToLidarType(lidar_type_env);

  param.print();

  LidarDriver<PointCloudMsg> driver;
  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback);
  driver.regExceptionCallback(exceptionCallback);
  if (!driver.init(param))
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }

  driver.start();
  RS_DEBUG << "RoboSense Lidar-Driver Linux online demo start......" << RS_REND;

  auto ret = run(dora_context);
  free_dora_context(dora_context);

  to_exit_process = true;
  driver.stop();
  std::cout << "exit rslidar driver ..." << std::endl;
  return ret;
}