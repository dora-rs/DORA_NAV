# RSLidar SDK Dora移植指南

本文档说明如何将速腾雷达驱动从ROS移植到Dora框架。

## 移植概览

### 保持不变
- **rs_driver核心驱动**：完全独立，不依赖ROS
- **Source-Destination架构**：保持原有的设计模式
- **配置文件格式**：继续使用YAML配置

### 主要变更
1. **移除ROS依赖**：删除所有ROS/ROS2相关代码
2. **新增Dora适配层**：`source_pointcloud_dora.hpp`
3. **修改节点入口**：`rslidar_dora_node.cpp`
4. **更新构建系统**：`CMakeLists_dora.txt`

## 文件结构

### 新增文件
```
rslidar_sdk/
├── src/source/source_pointcloud_dora.hpp    # Dora点云/IMU适配层
├── node/rslidar_dora_node.cpp               # Dora节点入口
├── CMakeLists_dora.txt                      # Dora版本CMake配置
└── config/config_dora.yaml                  # Dora版本配置文件
```

### 修改文件
```
rslidar_sdk/
├── src/manager/node_manager.hpp             # 添加dora_context_成员
├── src/manager/node_manager.cpp             # 替换ROS为Dora适配
```

### 可删除文件（原ROS版本）
```
rslidar_sdk/
├── node/rslidar_sdk_node.cpp               # ROS节点入口
├── src/source/source_pointcloud_ros.hpp    # ROS点云适配
├── src/source/source_packet_ros.hpp        # ROS包适配
├── src/msg/ros_msg/                        # ROS消息定义
├── launch/                                 # ROS launch文件
├── rviz/                                   # RViz配置
└── package.xml                             # ROS包配置
```

## 数据格式

### 统一的雷达驱动层格式

**重要：RS雷达驱动的输出格式与Livox驱动完全一致，确保所有雷达驱动层解耦。**

### 点云数据（二进制格式）

**固定格式（16字节/点）：**
```
[0..7]    double   timestamp      (秒)
[8..11]   uint32_t num_points     (点数量)
[12..]    N × 16 bytes: float32 x, y, z, intensity
```

**说明：**
- 虽然RS雷达支持XYZIRT（含ring和timestamp）等扩展格式，但为了与Livox驱动统一
- **只输出XYZI（x, y, z, intensity）四个字段**
- 丢弃ring、per-point timestamp、feature等额外字段
- 这样所有雷达（Livox MID360、RS16、RSM1等）输出格式完全相同
- 下游节点（定位、建图等）无需关心雷达型号

### IMU数据（JSON格式）

```json
{
  "header": {
    "frame_id": "rslidar",
    "timestamp": 1234567890.123
  },
  "angular_velocity": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },
  "linear_acceleration": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  }
}
```

### 输出Topic命名（固定）

- **点云**: `"pointcloud"` （与Livox一致）
- **IMU**: `"imu"` （与Livox一致）

**注意：topic名称固定，无法通过配置修改，这是为了统一所有雷达驱动的接口。**

## 编译步骤

### 1. 准备依赖

```bash
# 安装yaml-cpp
sudo apt-get install libyaml-cpp-dev

# 安装nlohmann_json
sudo apt-get install nlohmann-json3-dev

# 确保Dora node API可访问
# 默认路径：../../../dora/apis/c/node/node_api.h
```

### 2. 配置CMake

```bash
cd rslidar_sdk

# 使用Dora版本的CMakeLists.txt
cp CMakeLists_dora.txt CMakeLists.txt

# 或者直接指定
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
```

### 3. 编译

```bash
make -j$(nproc)
```

### 4. 安装

```bash
sudo make install
```

## 配置说明

### config_dora.yaml

```yaml
common:
  msg_source: 1                      # 1: 在线雷达, 3: PCAP文件
  send_point_cloud_dora: true        # 启用Dora点云输出

lidar:
  - driver:
      lidar_type: RSM1               # 雷达型号
      msop_port: 6699                # 数据端口
      difop_port: 7788               # 设备信息端口
      imu_port: 6688                 # IMU端口（0=禁用，6688=启用）
      # ... 其他驱动参数 ...
      
    dora:
      frame_id: rslidar              # 坐标系ID
      # 输出topic固定为 "pointcloud" 和 "imu"，与Livox保持一致
```

## 运行

### 基本运行

```bash
# 使用默认配置文件
./rslidar_dora_node

# 指定配置文件
./rslidar_dora_node /path/to/config_dora.yaml
```

### 在Dora dataflow中集成

创建dataflow YAML文件：

```yaml
nodes:
  - id: rslidar
    operator:
      native: /path/to/rslidar_dora_node
      args: ["/path/to/config_dora.yaml"]
    outputs:
      - pointcloud
      - imu

  - id: processing
    inputs:
      lidar_points: rslidar/pointcloud
      lidar_imu: rslidar/imu
    # ... 其他配置 ...
```

## 关键实现细节

### 1. 线程安全

Dora的C API（`dora_send_output`）不是线程安全的。由于`rs_driver`内部使用多线程处理数据，我们使用互斥锁保护所有发送操作：

```cpp
std::lock_guard<std::mutex> lock(send_mutex_);
dora_send_output(dora_context_, ...);
```

### 2. 事件循环

主线程运行Dora事件循环，等待Stop事件以实现优雅关闭：

```cpp
while (running) {
  void* event = dora_next_event(dora_context);
  if (read_dora_event_type(event) == DoraEventType_Stop) {
    break;
  }
  free_dora_event(event);
}
```

### 3. 数据发布

- 点云数据由`rs_driver`的内部回调线程触发
- 通过`DestinationPointCloudDora`的`sendPointCloud`方法发送
- IMU数据（如果启用）通过`sendImuData`方法发送

## 编译选项

在CMakeLists.txt中可配置：

```cmake
# 点类型（虽然支持XYZIRT等，但输出统一为XYZI格式）
set(POINT_TYPE XYZIRT)

# 启用IMU数据解析
option(ENABLE_IMU_DATA_PARSE "Enable IMU data parsing" ON)

# 启用DIFOP包解析
option(ENABLE_DIFOP_PARSE "Enable DIFOP packet parsing" ON)

# 其他选项...
```

**注意：** 
- 虽然内部可以设置POINT_TYPE为XYZIRT等，但**最终输出到Dora的格式统一为XYZI**
- 这样做是为了与Livox驱动保持一致，实现雷达驱动层的完全解耦
- 额外字段（如ring、per-point timestamp）在序列化时被丢弃

## 性能对比

与ROS版本相比：

- **延迟**：相当或更低（取决于Dora性能）
- **CPU占用**：相似（主要消耗在rs_driver核心）
- **内存**：更低（无ROS中间层开销）
- **吞吐量**：相当或更高（二进制格式，零拷贝潜力）

## 故障排除

### 编译错误

1. **找不到node_api.h**
   ```bash
   # 检查路径
   ls ../../../dora/apis/c/node/node_api.h
   
   # 或在CMake中指定
   cmake .. -DDORA_NODE_API_INCLUDE_DIR=/path/to/dora/apis/c/node
   ```

2. **找不到nlohmann/json.hpp**
   ```bash
   sudo apt-get install nlohmann-json3-dev
   ```

### 运行时错误

1. **"Failed to initialize Dora context"**
   - 确保在Dora dataflow环境中运行
   - 检查环境变量是否正确设置

2. **"Failed to send output"**
   - 检查输出名称与dataflow配置匹配
   - 查看Dora日志获取详细错误信息

3. **没有点云输出**
   - 检查雷达连接和配置
   - 确认`send_point_cloud_dora: true`
   - 检查端口配置（msop_port, difop_port）

## 与Livox驱动的一致性

本移植参考了Livox MID360的Dora实现：

- ✅ 相同的二进制点云格式
- ✅ 相同的JSON IMU格式
- ✅ 相同的线程安全机制
- ✅ 相同的事件循环模式

这确保了与现有Dora生态系统的兼容性。

## 后续优化建议

1. **零拷贝优化**：如果Dora支持，可以直接传递内存指针
2. **批量发送**：累积多帧后批量发送以减少系统调用
3. **压缩**：对点云数据进行压缩以减少传输带宽
4. **异步发送**：使用队列缓冲以避免阻塞驱动线程

## 参考

- [rs_driver文档](https://github.com/RoboSense-LiDAR/rs_driver)
- [Dora框架文档](https://github.com/dora-rs/dora)
- Livox驱动Dora实现：`modules/drivers/livox_driver/src/lddc_dora.cpp`
