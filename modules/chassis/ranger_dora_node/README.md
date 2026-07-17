# Ranger Mini V3 DORA Node

Ranger Mini V3 DORA底盘节点，用于控制AgileX Ranger系列机器人（Mini V1/V2/V3及标准版）。

## 功能特性

- 通过CAN总线与ugv_sdk通信
- 支持4种运动模式：
  - 双阿克曼转向 (Dual Ackerman)
  - 平行模式 (Parallel/Omnidirectional)
  - 原地旋转 (Spinning)
  - 侧滑模式 (Side Slip)
- 实时里程计计算（阿克曼运动学模型）
- JSON格式数据传输

## 代码风格

节点设计风格参考 `dora_mickrobot` 和 `pure_pursuit_dora` / `dwa_planner_dora`：
- 单一 `.cpp` 文件 + 独立头文件
- 匿名 namespace 封装内部类型
- 运行时配置硬编码在 `RuntimeConfig` 结构体中
- DORA 事件循环：`dora_next_event` → 按 input ID 分发

## 配置

所有配置参数硬编码在 `src/ranger_bringup.cpp` 的 `RuntimeConfig` 结构体中：

```cpp
struct RuntimeConfig {
  std::string can_port = "can0";
  std::string robot_model = "ranger_mini_v3";
  int update_rate = 100;
};
```

如需修改，直接编辑源码中的默认值。

## 输入输出接口

### 输入

- **cmd_vel**: 速度命令
  ```json
  {"linear": {"x": 0.5}, "angular": {"z": 0.2}}
  ```
  - `linear.x`: 线速度 (m/s)
  - `angular.z`: 角速度 (rad/s)

- **timer**: 定时器触发（里程计发布周期）
  - 来源: `dora/timer/millis/50`

### 输出

- **Odometry**: 里程计数据
  ```json
  {
    "pose": {
      "position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    "twist": {
      "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
      "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
    }
  }
  ```

## 依赖项

- CMake >= 3.10
- C++17编译器
- ugv_sdk (AgileX SDK)
- DORA runtime
- nlohmann/json (header-only)
- CAN总线接口（can0或can1）

## 编译

```bash
cd NavigationFramework
mkdir -p build && cd build
cmake ..
make ranger_miniv3_node
```

## 运行

1. 配置CAN总线：
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

2. 启动节点：
```bash
dora up
dora start ranger_miniv3_dataflow.yml
```

## 运动模式说明

节点根据输入的速度命令自动切换运动模式：

1. **双阿克曼模式** (Dual Ackerman)
   - 条件: 转弯半径 >= min_turn_radius
   - 行为: 类似汽车的阿克曼转向方式

2. **原地旋转模式** (Spinning)
   - 条件: 转弯半径 < min_turn_radius
   - 行为: 原地旋转

3. **平行模式** (Parallel) — 保留支持
   - 条件: cmd_vel 含 linear.y 时触发
   - 行为: 全向移动，可斜向行驶

4. **侧滑模式** (Side Slip) — 保留支持
   - 行为: 横向移动

## 坐标系

节点内部维护 `odom` → `base_link` 的里程计位姿（不发布TF，Dora框架无TF概念）。调用方自行理解坐标系关系。

## 机器人参数 (Ranger Mini V3)

```
轮距(track): 0.364 m
轴距(wheelbase): 0.494 m
最大线速度: 1.5 m/s
最大角速度: 4.8 rad/s
最小转弯半径: 0.476 m
```
