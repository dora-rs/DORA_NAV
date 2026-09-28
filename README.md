## DORA-NAV 

DORA-NAV 的目标是提供面向软硬件融合场景提供一体化导航整体方案，操作系统侧支持 Linux、开源鸿蒙、OpenEuler，硬件平台兼容 X86、NVIDIA Jetson、昇腾、此芯全系列算力设备。

开发者：(qiuji chen)2276606574@qq.com  、 cenruping@vip.qq.com  、 bob

室内导航案例视频：https://www.bilibili.com/video/BV199KJ6bEwj

## 硬件、操作系统支持列表

| 序号 |    硬件平台     |   系统    | 使用文档                                    |
| :--: | :-------------: | :-------: | ------------------------------------------- |
|  1   |     X86平台     |   linux   | 参考 [INSTALL_x86.md](doc/INSTALL_x86.md)   |
|  2   | Jetson AGX Orin |   linux   | 参考 [INSTALL_Orin.md](doc/INSTALL_Orin.md) |
|  3   |   瑞莎星睿O6    |   linux   | 已支持                                      |
|  4   |   瑞莎星睿O6    | 开源鸿蒙  | 已支持                                      |
|  5   |      昇腾       | OpenEuler | 参考 [INSTALL_310P.md](doc/INSTALL_310P.md) |
|  6   |     RK3588      | 开源鸿蒙  | 已支持                                      |

## 特性

- **模块化节点**：传感器驱动 / 定位 / 建图 / 规划 / 底盘 / 可视化各节点独立编译，按需启停。
- **数据流编排**：用 `dora run apps/xxx.yml` 一键运行，切换数据流即可组合不同任务。
- **跨平台**：x86 Ubuntu 22.04、Jetson Orin Ubuntu 20.04、昇腾 310P OpenEuler。
- **可视化**：基于 Rerun Viewer 实时显示点云、位姿、路径、预测轨迹与建图过程。

## 模块总览

| 类别       | 模块                                        | 说明                                             |
| ---------- | ------------------------------------------- | ------------------------------------------------ |
| 传感器驱动 | `drivers/livox_driver`                      | Livox MID360 驱动（点云 + IMU）                  |
|            | `drivers/rslidar_sdk`                       | RoboSense 速腾雷达驱动（点云 + IMU）             |
| 定位       | `localization/hdl_localization_dora`        | 点云 + IMU 融合定位（NDT 配准，带雷达-底盘外参） |
| 建图       | `mapping/hdl_graph_slam_dora`               | 移植自 hdl_graph_slam 的图优化 SLAM              |
|            | `mapping/lightning_lm_mapping`              | 基于 lightning-lm 的建图（带回环检测）           |
|            | `mapping/ndt_mapping_dora`                  | NDT 建图（默认不编译）                           |
| 全局规划   | `planner/global_planner/astar_planner_dora` | A* 全局路径规划                                  |
| 局部规划   | `planner/local_planner/pure_pursuit_dora`   | 纯跟踪路径跟踪                                   |
|            | `planner/local_planner/dwa_planner_dora`    | DWA 局部规划（避障，默认不编译）                 |
| 底盘       | `chassis/ranger_dora_node`                  | Ranger MiniV3 底盘驱动（UGV SDK）                |
|            | `chassis/dora_mickrobot`                    | MickrRobotX4 底盘驱动（默认不编译）              |
| 任务接口   | `interface/goal_publisher`                  | UDP 目标点发布                                   |
| 可视化     | `visualization/rerun_visualizer`            | 实时显示点云 / 位姿 / 路径                       |
|            | `visualization/rerun_lidar`                 | 雷达点云可视化                                   |

## 快速开始

### 1. 环境安装

按目标平台选择安装文档：

- **x86 / Ubuntu 22.04** → [INSTALL_x86.md](docs/INSTALL_x86.md)
- **Jetson AGX Orin / Ubuntu 20.04** → [INSTALL_Orin.md](docs/INSTALL_Orin.md)
- **昇腾 310P / OpenEuler** → [INSTALL_310P.md](docs/INSTALL_310P.md)

安装内容包括：dora 1.0（命令行 + `third_party/dora/lib` 下 `libdora_node_api_c.a`）、第三方库（Livox-SDK2、ndt_omp、serial、g2o、nlohmann-json 等）、Rerun SDK 0.31.2，并手动创建空目录（`maps/pcd`、`maps/pgm`、`third_party/dora/...` 等）。

### 2. 编译

```shell
mkdir build && cd build
cmake .. && make -j$(nproc)
cd ..
```

子模块可通过 CMake 开关独立控制，如 `-DBUILD_MAPPING=OFF` 关闭建图节点（见 `CMakeLists.txt` 顶部）。

### 3. 运行

```shell
dora run apps/run.yml        # 完整导航（默认使用 rslidar 雷达）
dora run apps/localization.yml   # 仅定位
dora run apps/mapping.yml        # 建图
dora run apps/lidar.yml          # 点云可视化
```

- `dora run` 在本机隔离运行数据流，无需 `dora up`；如需 list/stop/logs 管理，改用 `dora up` + `dora start apps/xxx.yml --detach` + `dora down`。
- 节点的工作目录以 yml 所在目录（`apps/`）为根，配置中的相对路径均以此为基准。
- 更换雷达/底盘：在对应 yml 中注释/启用 livox 或 mickrobot 节点即可。

## 地图工具

`tools/` 下为独立编译的离线工具（在主 CMake 之外单独编译）：

| 工具         | 用途                                                         |
| ------------ | ------------------------------------------------------------ |
| `map_trans`  | 将 PCD 点云地图转换为 PGM/YAML 栅格地图（`./pcd2pgm config/config.json`） |
| `pgm_editor` | 离线编辑 PGM 栅格地图，清除人影残留等干扰点，或添加虚拟障碍物 |

编辑完成后的栅格地图放至 `maps/pgm/`，供 A* 全局规划使用。

## 目录结构

```
NavigationFramework/
├── apps/               # dora dataflow 编排文件（run/localization/mapping/lidar）
├── modules/            # 功能节点（drivers/localization/mapping/planner/chassis/interface/visualization）
│   ├── drivers/        #   雷达驱动（livox / rslidar）
│   ├── localization/   #   hdl_localization（定位）
│   ├── mapping/        #   hdl_graph_slam / lightning_lm / ndt_mapping（建图）
│   ├── planner/        #   global(astar) / local(pure_pursuit, dwa)
│   ├── chassis/        #   ranger / mickrobot 底盘
│   ├── interface/      #   goal_publisher（UDP 目标点）
│   └── visualization/  #   rerun 相关
├── maps/               # 点云(pcd)与栅格(pgm)地图
├── tools/              # map_trans / pgm_editor 离线工具
├── third_party/        # dora SDK、Livox-SDK2、ndt_omp、serial 等
├── CHANGELOG.md        # 开发/更新日志
└── INSTALL_*.md        # 各平台安装文档
```

## 说明

- 传感器 / 定位 / 建图 / 全局规划等节点通过 `modules/**/config/` 下的 json/yaml 配置参数（文件内路径以 `apps/` 为工作根目录）。
- 开发中的问题记录与设计取舍见 [CHANGELOG.md](docs/CHANGELOG.md)。
