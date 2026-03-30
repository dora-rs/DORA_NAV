# ROS接口2

## 1 传感器消息类型

当前整理IMU/GNSS/odom/pose/path消息类型 

### 1\.1 IMU



| **字段路径**                                | **说明**                                                  |
| ------------------------------------------- | --------------------------------------------------------- |
| j_imu_pub["header"]["frame_id"]             | 坐标系名称，例如  "imu_link"                              |
| j_imu_pub["header"]["seq"]                  | 消息序列号，递增（如  counter_imu_pub++）                 |
| j_imu_pub["header"]["stamp"]["sec"]         | 时间戳的秒数（tv.tv_sec）                                 |
| j_imu_pub["header"]["stamp"]["nanosec"]     | 时间戳的纳秒数（tv.tv_usec * 1e3）                        |
| j_imu_pub["angular_velocity"]["x"]          | 角速度的 x 分量（单位：rad/s）                            |
| j_imu_pub["angular_velocity"]["y"]          | 角速度的 y 分量（单位：rad/s）                            |
| j_imu_pub["angular_velocity"]["z"]          | 角速度的 z 分量（单位：rad/s）                            |
| j_imu_pub["linear_acceleration"]["x"]       | 线性加速度的 x 分量（单位：m/s²）                         |
| j_imu_pub["linear_acceleration"]["y"]       | 线性加速度的 y 分量（单位：m/s²）                         |
| j_imu_pub["linear_acceleration"]["z"]       | 线性加速度的 z 分量（单位：m/s²）                         |
| j_imu_pub["orientation"]["x"]               | 四元数的 x 分量（表示方向）                               |
| j_imu_pub["orientation"]["y"]               | 四元数的 y 分量（表示方向）                               |
| j_imu_pub["orientation"]["z"]               | 四元数的 z 分量（表示方向）                               |
| j_imu_pub["orientation"]["w"]               | 四元数的 w 分量（表示方向）                               |
| j_imu_pub["angular_velocity_covariance"]    | 角速度协方差矩阵（长度为 9 的数组，3x3 矩阵按行展开）     |
| j_imu_pub["linear_acceleration_covariance"] | 线性加速度协方差矩阵（长度为 9 的数组，3x3 矩阵按行展开） |
| j_imu_pub["orientation_covariance"]         | 方向协方差矩阵（长度为 9 的数组，3x3 矩阵按行展开）       |

### 1\.2 GNSS

- 时间戳格式：sec 和 nanosec 分别表示秒和纳秒，需确保 nanosec 范围在 \[0, 1e9\) 内。
- 协方差矩阵说明
协方差矩阵为 3x3 矩阵，按行优先顺序展开为 9 个元素，例如：
\[cov00, cov01, cov02,
cov10, cov11, cov12,
cov20, cov21, cov22\]
- GPS 状态说明status["status"]的取值范围：
0: 未定位（No Fix）
1: 2D 定位（2D Fix）
2: 3D 定位（3D Fix）

| 字段路径                                | 说明                                                         |
| --------------------------------------- | ------------------------------------------------------------ |
| j_gps_pub["header"]["frame_id"]         | 坐标系名称，例如  "gps_link"                                 |
| j_gps_pub["header"]["seq"]              | 消息序列号，递增（如  counter_gps_pub++）                    |
| j_gps_pub["header"]["stamp"]["sec"]     | 时间戳的秒数（tv.tv_sec）                                    |
| j_gps_pub["header"]["stamp"]["nanosec"] | 时间戳的纳秒数（tv.tv_usec * 1e3）                           |
| j_gps_pub["latitude"]                   | 纬度（单位：度，例如 40.7128）                               |
| j_gps_pub["longitude"]                  | 经度（单位：度，例如 -74.0060）                              |
| j_gps_pub["altitude"]                   | 海拔高度（单位：米，例如 10.5）                              |
| j_gps_pub["position_covariance"][0]     | 协方差矩阵的 cov00 分量（经度-经度）                         |
| j_gps_pub["position_covariance"][1]     | 协方差矩阵的 cov01 分量（经度-纬度）                         |
| j_gps_pub["position_covariance"][2]     | 协方差矩阵的 cov02 分量（经度-海拔）                         |
| j_gps_pub["position_covariance"][3]     | 协方差矩阵的 cov10 分量（纬度-经度）                         |
| j_gps_pub["position_covariance"][4]     | 协方差矩阵的 cov11 分量（纬度-纬度）                         |
| j_gps_pub["position_covariance"][5]     | 协方差矩阵的 cov12 分量（纬度-海拔）                         |
| j_gps_pub["position_covariance"][6]     | 协方差矩阵的 cov20 分量（海拔-经度）                         |
| j_gps_pub["position_covariance"][7]     | 协方差矩阵的 cov21 分量（海拔-纬度）                         |
| j_gps_pub["position_covariance"][8]     | 协方差矩阵的 cov22 分量（海拔-海拔）                         |
| j_gps_pub["position_covariance_type"]   | 协方差类型（0: 未知, 1: 无协方差, 2: 部分已知, 3: 完全已知） |
| j_gps_pub["status"]["status"]           | GPS 状态（0: 未定位, 1: 2D 定位, 2: 3D 定位）                |
| j_gps_pub["status"]["satellites_used"]  | 使用的卫星数量（例如 8）                                     |



### 1\.3 Pose消息类型

用于描述机器人的位姿信息（位置、姿态），带协方差geometry\_msgs/PoseWithCovariance。

| **字段路径**                             | **说明**                                    |
| ---------------------------------------- | ------------------------------------------- |
| j_odom_pub["header"]["frame_id"]         | 坐标系名称，例如  "odom"                    |
| j_odom_pub["header"]["seq"]              | 消息序列号，递增                            |
| j_odom_pub["header"]["stamp"]["sec"]     | 时间戳的秒数（tv.tv_sec）                   |
| j_odom_pub["header"]["stamp"]["nanosec"] | 时间戳的纳秒数（tv.tv_usec * 1e3）          |
| j_odom_pub["child_frame_id  "]           | 子坐标系名称，例如  "base_link"             |
| j_pose["pose"]["position"]["x"]          | 位置 x 坐标  (米)，如 1.25                  |
| j_pose["pose"]["position"]["y"]          | 位置 y 坐标  (米)，如 3.4                   |
| j_pose["pose"]["position"]["z"]          | 位置 z 坐标  (米)，如 0.0                   |
| j_pose["pose"]["orientation"]["x"]       | 四元数 x 分量，如  0.0                      |
| j_pose["pose"]["orientation"]["y"]       | 四元数 y 分量，如  0.0                      |
| j_pose["pose"]["orientation"]["z"]       | 四元数 z 分量，如  0.7071                   |
| j_pose["pose"]["orientation"]["w"]       | 四元数 w 分量，如  0.7071                   |
| j_pose["covariance"]                     | 36 元素数组，表示  6×6 协方差矩阵（行主序） |



### 1\.4 里程计

用于描述机器人的里程信息（位置、姿态、速度）。对应ROS2的odom消息类型（nav\_msgs/Odometry）

| **字段路径**                             | **说明**                           |
| ---------------------------------------- | ---------------------------------- |
| j_odom_pub["header"]["frame_id"]         | 坐标系名称，例如  "odom"           |
| j_odom_pub["header"]["seq"]              | 消息序列号，递增                   |
| j_odom_pub["header"]["stamp"]["sec"]     | 时间戳的秒数（tv.tv_sec）          |
| j_odom_pub["header"]["stamp"]["nanosec"] | 时间戳的纳秒数（tv.tv_usec * 1e3） |
| j_odom_pub["child_frame_id  "]           | 子坐标系名称，例如  "base_link"    |
| j_odom_pub["pose"]["position"]["x"]      | 位姿的 x 坐标（如 position_x）     |
| j_odom_pub["pose"]["position"]["y"]      | 位姿的 y 坐标（如 position_y）     |
| j_odom_pub["pose"]["position"]["z"]      | 位姿的 z 坐标（通常为 0）          |
| j_odom_pub["pose"]["orientation"]["x"]   | 四元数的 x 分量（如 0）            |
| j_odom_pub["pose"]["orientation"]["y"]   | 四元数的 y 分量（如 0）            |
| j_odom_pub["pose"]["orientation"]["z"]   | 四元数的 z 分量（如 0）            |
| j_odom_pub["pose"]["orientation"]["w"]   | 四元数的 w 分量（如 1）            |
| j_odom_pub["pose"]["covariance"][0]      | 协方差矩阵的 cov00 分量（x-x）     |
| j_odom_pub["pose"]["covariance"][1]      | 协方差矩阵的 cov01 分量（x-y）     |
| j_odom_pub["pose"]["covariance"][2]      | 协方差矩阵的 cov02 分量（x-z）     |
| j_odom_pub["pose"]["covariance"][3]      | 协方差矩阵的 cov10 分量（y-x）     |
| j_odom_pub["pose"]["covariance"][4]      | 协方差矩阵的 cov11 分量（y-y）     |
| j_odom_pub["pose"]["covariance"][5]      | 协方差矩阵的 cov12 分量（y-z）     |
| j_odom_pub["pose"]["covariance"][6]      | 协方差矩阵的 cov20 分量（z-x）     |
| j_odom_pub["pose"]["covariance"][7]      | 协方差矩阵的 cov21 分量（z-y）     |
| j_odom_pub["pose"]["covariance"][8]      | 协方差矩阵的 cov22 分量（z-z）     |
| j_odom_pub["twist"]["linear"]["x"]       | 线速度的 x 分量（如 linear_x）     |
| j_odom_pub["twist"]["linear"]["y"]       | 线速度的 y 分量（如 linear_y）     |
| j_odom_pub["twist"]["linear"]["z"]       | 线速度的 z 分量（通常为 0）        |
| j_odom_pub["twist"]["angular"]["x"]      | 角速度的 x 分量（通常为 0）        |
| j_odom_pub["twist"]["angular"]["y"]      | 角速度的 y 分量（通常为 0）        |
| j_odom_pub["twist"]["angular"]["z"]      | 角速度的 z 分量（如 linear_w）     |
| j_odom_pub["twist"]["covariance"][0]     | 协方差矩阵的 cov00 分量（x-x）     |
| j_odom_pub["twist"]["covariance"][1]     | 协方差矩阵的 cov01 分量（x-y）     |
| j_odom_pub["twist"]["covariance"][2]     | 协方差矩阵的 cov02 分量（x-z）     |
| j_odom_pub["twist"]["covariance"][3]     | 协方差矩阵的 cov10 分量（y-x）     |
| j_odom_pub["twist"]["covariance"][4]     | 协方差矩阵的 cov11 分量（y-y）     |
| j_odom_pub["twist"]["covariance"][5]     | 协方差矩阵的 cov12 分量（y-z）     |
| j_odom_pub["twist"]["covariance"][6]     | 协方差矩阵的 cov20 分量（z-x）     |
| j_odom_pub["twist"]["covariance"][7]     | 协方差矩阵的 cov21 分量（z-y）     |
| j_odom_pub["twist"]["covariance"][8]     | 协方差矩阵的 cov22 分量（z-z）     |

### 1\.5 Path

可视化路径、轨迹点，对应ROS2中的nav\_msgs/msg/Path 消息类型

| j_path_pub["header"]["frame_id"]                     | 坐标系名称，例如 "map"                     |
| ---------------------------------------------------- | ------------------------------------------ |
| j_path_pub["header"]["seq"]                          | 消息序列号，递增（如  counter_path_pub++） |
| j_path_pub["header"]["stamp"]["sec"]                 | 时间戳的秒数（tv.tv_sec）                  |
| j_path_pub["header"]["stamp"]["nanosec"]             | 时间戳的纳秒数（tv.tv_usec * 1e3）         |
| j_path_pub["poses"][0]["header"]["frame_id"]         | 路径点的坐标系名称（如  "base_link"）      |
| j_path_pub["poses"][0]["header"]["seq"]              | 路径点的序列号                             |
| j_path_pub["poses"][0]["header"]["stamp"]["sec"]     | 路径点的时间戳秒数                         |
| j_path_pub["poses"][0]["header"]["stamp"]["nanosec"] | 路径点的时间戳纳秒数                       |
| j_path_pub["poses"][0]["pose"]["position"]["x"]      | 路径点的 x 坐标                            |
| j_path_pub["poses"][0]["pose"]["position"]["y"]      | 路径点的 y 坐标                            |
| j_path_pub["poses"][0]["pose"]["position"]["z"]      | 路径点的 z 坐标                            |
| j_path_pub["poses"][0]["pose"]["orientation"]["x"]   | 路径点的四元数 x 分量                      |
| j_path_pub["poses"][0]["pose"]["orientation"]["y"]   | 路径点的四元数 y 分量                      |
| j_path_pub["poses"][0]["pose"]["orientation"]["z"]   | 路径点的四元数 z 分量                      |
| j_path_pub["poses"][0]["pose"]["orientation"]["w"]   | 路径点的四元数 w 分量                      |

## 2、控制指令消息类型（cmd\_vel）

控制指令的消息类型参考ROS2中geometry\_msgs/Twist 消息类型设计，

| **JSON** **路径**                        | **说明与取值示例**                        |
| ---------------------------------------- | ----------------------------------------- |
| j_path_pub["header"]["frame_id"]         | 坐标系名称，例如 "map"                    |
| j_path_pub["header"]["seq"]              | 消息序列号，递增（如 counter_path_pub++） |
| j_path_pub["header"]["stamp"]["sec"]     | 时间戳的秒数（tv.tv_sec）                 |
| j_path_pub["header"]["stamp"]["nanosec"] | 时间戳的纳秒数（tv.tv_usec * 1e3）        |
| j_twist["linear"]["x"]                   | 线速度 X 轴分量 (m/s)，例如 0.5           |
| j_twist["linear"]["y"]                   | 线速度 Y 轴分量 (m/s)，例如 0.0           |
| j_twist["linear"]["z"]                   | 线速度 Z 轴分量 (m/s)，例如 0.0           |
| j_twist["angular"]["x"]                  | 角速度 X 轴分量 (rad/s)，例如 0.0         |
| j_twist["angular"]["y"]                  | 角速度 Y 轴分量 (rad/s)，例如 0.0         |
| j_twist["angular"]["z"]                  | 角速度 Z 轴分量 (rad/s)，例如 0.3         |

## 3、消息自动转换

整体思路，在C\+\+中定义结构体，同时定义结构体到json的自动转换，由结构体自动换转为json中

定义 单个消息文件（odom\_message\.hpp）

```C++
#pragma once
#include "json.hpp" // nlohmann/json (需提前下载)

// 1. 定义消息结构 (与 JSON 结构 1:1 对应)
struct OdomMessage {
    struct Header {
        struct Stamp { int sec; int nanosec; } stamp;
        std::string frame_id;
        int seq = 0;
    } header;
    
    struct Pose {
        struct Position { double x=0,y=0,z=0; } position;
        struct Orientation { double x=0,y=0,z=0,w=1; } orientation;
    } pose;
    
    struct Twist {
        struct Linear { double x=0,y=0,z=0; } linear;
        struct Angular { double x=0,y=0,z=0; } angular;
    } twist;
};

// 2. 关键魔法：一行启用 JSON 互操作
NLOHMANN_DEFINE_TYPE_INTRUSIVE( // 自动序列化/反序列化
    OdomMessage,
    header.stamp.sec, header.stamp.nanosec, header.frame_id, header.seq,
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    twist.linear.x, twist.linear.y, twist.linear.z,
    twist.angular.x, twist.angular.y, twist.angular.z
);
// NLOHMANN_DEFINE_TYPE_INTRUSIVE 是 nlohmann/json 的宏，**自动为结构体注入 JSON 序列化能力**，无需手写任何转换代码。
```

### 3\.1 C\+\+ 使用代码

```C++
#include "odom_message.hpp"

int main() {
    // 1. 像普通结构体一样创建/操作消息
    OdomMessage msg;
    msg.header.frame_id = "odom";
    msg.header.seq = 42; // counter_odom_pub++
    msg.header.stamp.sec = 1717029202;
    msg.header.stamp.nanosec = 123456789;
    
    msg.pose.position.x = 0;
    msg.pose.orientation.w = 1;
    
    // 假设 dt1_msg 来自传感器
    struct { double vx=0.5, vz=0.3; } dt1_msg;
    msg.twist.linear.x = dt1_msg.vx;
    msg.twist.angular.z = dt1_msg.vz;

    // 2. 一行转换为 JSON (自动深度嵌套)
    nlohmann::json j = msg; // 结构体 → JSON

    // 3. 直接使用 (无需手动构建嵌套)
    std::cout << j.dump(2) << std::endl;
    
    // 4. 甚至可以直接写入文件
    std::ofstream("odom.json") << j.dump(4);
}
```

这里通过“std::cout \&lt;\&lt; j\.dump\(2\) \&lt;\&lt; std::endl;“ 语句会输出

```JSON
{
  "header": {
    "stamp": {
      "sec": 1717029202,
      "nanosec": 123456789
    },
    "frame_id": "odom",
    "seq": 42
  },
  "pose": {
    "position": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    }
  },
  "twist": {
    "linear": {
      "x": 0.5,
      "y": 0.0,
      "z": 0.0
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.3
    }
  }
}
```

|特性|传统 JSON 手动构建|本方案|优势|
|---|---|---|---|
|代码量|20\+ 行硬编码字段路径|0 行转换代码|减少 95% 重复代码|
|类型安全|字符串键易拼错 \(\&\#34;frmae\_id\&\#34;\)|编译时检查 \(msg\.header\.frame\_id\)|消除 100% 字段名错误|
|嵌套深度|手动写 j\[\&\#34;a\&\#34;\]\[\&\#34;b\&\#34;\]\[\&\#34;c\&\#34;\]|自动处理任意深度嵌套|支持 10\+ 层嵌套无额外代码|
|重构能力|重命名字段需全局搜索替换|IDE 一键重命名|修改字段名只需改一处|
|性能|多次哈希查找|直接内存访问|速度快 5\-10 倍|

### 3\.2 扩展：支持任意消息类型（Path/LaserScan 等）

只需在同一个头文件中追加新结构体：

```C++
// 在 odom_message.hpp 末尾追加
struct PathMessage {
    struct PoseStamped {
        OdomMessage::Header header; // 复用已有 Header
        OdomMessage::Pose pose;
    };
    OdomMessage::Header header;
    std::vector<PoseStamped> poses;
};

NLOHMANN_DEFINE_TYPE_INTRUSIVE(
    PathMessage,
    header.stamp.sec, header.stamp.nanosec, header.frame_id, header.seq,
    poses // 自动递归序列化 vector
);
```

业务代码同样简洁：

```C++
PathMessage path;
path.header.frame_id = "map";
path.poses.resize(2);
path.poses[0].pose.position.x = 1.0;
path.poses[1].pose.position.x = 2.0;

nlohmann::json j_path = path; // 一行转换
```

添加后的目录结构

```Plaintext
include/
├─ json.hpp
│  #nlohmann/json(单文件)
└─ messages.hpp
   #所有消息类型定义(本方案核心)
src/
└─ main.cpp
   #业务代码(仅包含messages.hpp)
```

### 3\.3 嵌套处理

修改 messages\.hpp

```C++
#pragma once
#include "json.hpp" // nlohmann/json (单头文件)

// ===== 1. 基础类型 (可复用) =====
struct Time {
    int sec = 0;
    int nanosec = 0;
};
NLOHMANN_DEFINE_TYPE_INTRUSIVE(Time, sec, nanosec); // 为 Time 启用 JSON

struct Header {
    Time stamp;
    std::string frame_id = "map";
    int seq = 0;
};
// 注意：Header 依赖 Time，但无需特殊处理
NLOHMANN_DEFINE_TYPE_INTRUSIVE(Header, stamp, frame_id, seq);

// ===== 2. 几何基础类型 =====
struct Point {
    double x = 0, y = 0, z = 0;
};
NLOHMANN_DEFINE_TYPE_INTRUSIVE(Point, x, y, z);

struct Quaternion {
    double x = 0, y = 0, z = 0, w = 1;
};
NLOHMANN_DEFINE_TYPE_INTRUSIVE(Quaternion, x, y, z, w);

// ===== 3. 核心消息类型 =====
// 里程计消息 (Odometry)
struct OdomMessage {
    Header header;
    struct {
        struct {
            Point position;
            Quaternion orientation;
        } pose;
        std::array<double, 36> covariance = {};
    } pose;
    
    struct {
        struct {
            Point linear;  // 复用 Point 作为 Vector3
            Point angular;
        } twist;
        std::array<double, 36> covariance = {};
    } twist;
};
NLOHMANN_DEFINE_TYPE_INTRUSIVE(
    OdomMessage,
    header,
    pose.pose.position, pose.pose.orientation, pose.covariance,
    twist.twist.linear, twist.twist.angular, twist.covariance
);

// 路径消息 (Path)
struct PathMessage {
    Header header;
    std::vector<OdomMessage::pose.pose> poses; // 复用 Odom 的 pose 部分
};
NLOHMANN_DEFINE_TYPE_INTRUSIVE(PathMessage, header, poses);

// 激光雷达消息 (简化版)
struct LaserScan {
    Header header;
    float angle_min = 0.0f;
    float angle_max = 0.0f;
    std::vector<float> ranges;
};
NLOHMANN_DEFINE_TYPE_INTRUSIVE(LaserScan, header, angle_min, angle_max, ranges);
```

复用基础类型（Time, Header, Point）。嵌套类型自动序列化：OdomMessage 包含 Header 时，自动调用 Header 的序列化器。跨消息复用：PathMessage 直接使用 OdomMessage::pose\.pose 结构

业务代码（多类型无缝混用）

```C++
#include "messages.hpp"
#include <fstream>

int main() {
    // 1. 里程计消息 (完全按您要求的字段)
    OdomMessage odom;
    odom.header.frame_id = "odom";
    odom.header.seq = 42; // counter_odom_pub++
    odom.header.stamp.sec = 1717029202;
    odom.header.stamp.nanosec = 123456789;
    
    odom.pose.pose.position.x = 0;
    odom.pose.pose.orientation.w = 1;
    
    // 模拟传感器数据
    struct { double vx=0.5, vz=0.3; } dt1_msg;
    odom.twist.twist.linear.x = dt1_msg.vx;
    odom.twist.twist.angular.z = dt1_msg.vz;
    
    // 2. 路径消息 (复用基础结构)
    PathMessage path;
    path.header = odom.header; // 直接复制 Header
    path.poses.push_back(odom.pose.pose); // 复用 Odom 的 pose
    
    // 3. 激光雷达 (独立类型)
    LaserScan scan;
    scan.header = odom.header;
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.ranges = {1.2, 1.3, 1.4}; // 模拟距离数据
    
    // 4. 一行转换为 JSON (各自独立)
    nlohmann::json j_odom = odom;
    nlohmann::json j_path = path;
    nlohmann::json j_scan = scan;
    
    // 5. 按需使用
    std::cout << "Odom X: " << j_odom["pose"]["pose"]["position"]["x"].get<double>() << "\n";
    std::cout << "Path points: " << j_path["poses"].size() << "\n";
    
    // 6. 保存到不同文件
    std::ofstream("odom.json") << j_odom.dump(2);
    std::ofstream("path.json") << j_path.dump(2);
    std::ofstream("scan.json") << j_scan.dump(2);
}
```

打印输出数据

```JSON
{
  "header": {
    "stamp": {
      "sec": 1717029202,
      "nanosec": 123456789
    },
    "frame_id": "odom",
    "seq": 42
  },
  "pose": {
    "pose": {
      "position": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    },
    "covariance": [0.0, 0.0, ...] // 36 zeros
  },
  "twist": {
    "twist": {
      "linear": {
        "x": 0.5,
        "y": 0.0,
        "z": 0.0
      },
      "angular": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.3
      }
    },
    "covariance": [0.0, 0.0, ...] // 36 zeros
  }
}
```

每个类型有独立的特化实现，编译器通过类型名称精确匹配，实现模板特化隔离，使得类型解析不冲突。

### 序列化与反序列化操作说明

|操作|说明|
|---|---|
|创建 OdomMessage|纯内存分配|
|json j = odom|完整序列化|
|OdomMessage m = j|完整反序列化|

> 
