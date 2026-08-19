# MickRobot MuJoCo 仿真

本模块在 MuJoCo中通过已有的urdf文件导入移动机器人模型，模型包含四轮差速底盘、16 线 3D 激光雷达、RGB 相机、IMU、轮速里程计和 GNSS。仿真节点接收 Dora 框架下的 `cmd_vel`控制命令，发布3D激光点云、图像、IMU、轮速里程计、GNSS五类传感器数据；并通过rerun可视化仿真数据。

## 依赖项安装

```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-dev libglfw3 libgl1 libegl1
```

安装 MuJoCo、Dora Python API、NumPy、PyArrow、PyYAML 和 Rerun SDK：

```bash
cd DORA_NAV/modules/simulation/mujoco_mickrobot
python3 -m pip install --user -e '.[rerun]'
```

如果其他 Python 依赖已经安装、只缺少 Rerun，可单独执行：

```bash
python3 -m pip install --user "rerun-sdk>=0.36,<1"
```

安装后检查系统 Python 是否能找到关键模块：

```bash
python3 -c "import dora, mujoco, numpy, pyarrow, rerun, yaml; print('Python dependencies OK')"
dora version
```

`dora-rs` 是 Python 节点 API，不能替代 `dora` CLI；`dora version` 必须能正常输出版本。需要运行测试时，再安装测试依赖：

```bash
python3 -m pip install --user -e '.[test,rerun]'
```

## URDF模型导入Mujoco中

先离线转换并验证模型：

```bash
python3 scripts/build_model.py
```

静态查看机器人与传感器安装关系，不启动 Dora：

```bash
python3 scripts/build_model.py
python3 -m mujoco.viewer --mjcf models/mickrobot.xml
```

每次执行脚本 build_model.py 都会重新读取 URDF，转换生成mujoco中的`models/mujoco_extensions.xml` 和测试地图。

DORA 环境下启动仿真节点(DORA 仿真节点只加载models/xxxx.xml文件，在不同的电脑上使用时需要注意徐工models/mickrobot.xml文件中3D模型的路径)：

```bash
dora run dataflow_mujoco_mickrobot.yml
```

`dataflow_mujoco_mickrobot.yml` 包含MuJoCo GUI 节点和 Rerun 节点。当服务器无界面运行时，可以尝试修改修改配置文件：

```yaml
runtime:
  headless: true
  fastest: true
rerun:
  spawn: false
```

然后设置：

```bash
MUJOCO_CONFIG=/absolute/path/headless.yaml MUJOCO_GL=egl dora run dataflow.yml
```

## Dora 接口

输入端口 `cmd_vel` 使用 UTF-8 JSON，并只读取 `linear.x` 和 `angular.z`：

```json
{"linear":{"x":0.8},"angular":{"z":0.3}}
```

非法数值和错误类型会被拒绝；超过 500 ms 没有有效命令时立即停车。

输出端口及默认频率：

| 端口 | 默认频率 | 数据 |
|---|---:|---|
| `pointcloud` | 10 Hz | 雷达局部坐标 XYZI |
| `image` | 30 Hz | 640×480 RGB8 |
| `imu` | 100 Hz | JSON，姿态、rad/s、加速度 g |
| `Odometry` | 50 Hz | JSON，轮反馈积分 pose/twist |
| `gnss` | 10 Hz | JSON，WGS84、ENU 速度、协方差 |

点云与 rslidar 节点兼容，全部小端：

```text
double timestamp_s | uint32 point_count | point_count × float32[x,y,z,intensity]
```

图像格式为：

```text
double timestamp_s | uint32 width | uint32 height | uint8 encoding | RGB bytes
```

`encoding=1` 表示 RGB8。二进制头分别为 12 字节和 17 字节。

## 测试脚本—测试轨迹发布

`tests/fixtures/cmd_vel_sender.py` 不是固定速度发送器。它订阅 `Odometry` 和 `imu`，在 100 ms timer 到期时发布闭环 `cmd_vel`。位置以原点为初值，使用 `Odometry.twist.linear.x` 和消息仿真时间差沿 IMU yaw 积分；轮速里程计的 pose/yaw 不参与控制，因为四轮滑移转向时它们会显著偏离真实运动。

启动路线先避开原点东侧障碍物：

```text
(0,0) → (0,-8.5)
```

随后持续循环：

```text
(0,-8.5) → (-8.5,-8.5) → (-8.5,8.5)
→ (8.5,8.5) → (8.5,-8.5) → (0,-8.5) → 重复
```

航点阈值为 0.35 m，最大线速度 0.6 m/s、最大角速度 0.8 rad/s。航向误差超过 20° 时原地转向，避免大曲率启动弧线碰到 `low_obstacle`。Odometry 或 IMU 任一路超过 0.5 s 未更新时发送零速度。

确定性 MuJoCo 验收以 500 Hz 物理、50 Hz 轮速和 100 Hz IMU 运行 40 仿真秒，验证启动段到达、首次航点切换以及机器人与全部围墙/障碍物零接触。由于不用 GNSS 和点云，轮速尺度误差会长期累计；持续运行很多圈后不能提供绝对定位意义上的永久无碰撞保证。

## 仿真节点参数配置

默认配置在 `config/default.yaml`，主要包含：

- `physics`：2 ms 步长、重力；
- `vehicle`：轮距、轮径、速度/加速度/力限制和命令超时；
- `lidar`：16×900、视场角、量程、频率和距离噪声；
- `camera`：640×480、90° 水平视场和 30 Hz；
- `imu`、`odometry`、`gnss`：频率、frame 与噪声；
- `runtime`：GUI/headless、fastest、确定性随机种子和输出失败阈值；
- `rerun`：spawn/connect、频率窗口和打印周期。

仿真环境坐标系为为 ENU（X 东、Y 北、Z 上），GNSS 默认原点为 `(39.9042, 116.4074, 50.0)`。

机器人 link、joint、名称、父子关系、origin、axis、visual、collision、质量和惯量均以 URDF 为准。相机、IMU、GNSS site、MuJoCo sensor 和四轮 actuator 等 URDF 无法表达的内容位于 `models/mujoco_extensions.xml`。修改模型后需要再次手动运行 `python3 scripts/build_model.py`；运行配置不能修改模型坐标。

## Rerun 可视化界面

`mujoco-mickrobot-rerun` 订阅 `pointcloud`、`image`、`imu`、`Odometry` 和 `gnss`。它接收数据，并在窗口打印 数据频率Hz、累计帧数和最新样本时间； 
