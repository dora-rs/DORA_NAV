# Livox Dora 驱动

## 点云输出格式

节点通过环境变量 `POINT_CLOUD_FORMAT` 选择 `pointcloud` output 的二进制格式。
环境变量未设置或为空时默认使用原有 `XYZI` 格式，变量值区分大小写；不支持的值会使节点在启动时退出并输出错误信息。

```bash
# 保持原有格式（默认）
export POINT_CLOUD_FORMAT=XYZI

# 与 RoboSense 驱动相同的 FLIO v1 Standard 格式
export POINT_CLOUD_FORMAT=XYZITRR

# 保留 MID360 原生点字段
export POINT_CLOUD_FORMAT=MID360
```

### `XYZI`

兼容驱动原有格式，小端序：8 字节 `float64` 帧时间戳（秒）、4 字节 `uint32`
点数，以及每点 16 字节的 `float32 x, y, z, intensity`。总长度为
`12 + point_count * 16` 字节。

### `XYZITRR`

使用与 `rslidar_sdk` 相同的 FLIO v1 Standard 格式：40 字节头部，加上每点 24 字节：

```text
float32 x, y, z, intensity, time
uint16 ring, reserved
```

Livox 的纳秒时间转换为相对帧起始时间的秒数写入 `time`，`line` 写入 `ring`，
`reserved` 固定为 0。

### `MID360`

使用小端序的 20 字节头部：

| 偏移 | 类型 | 含义 |
| ---: | --- | --- |
| 0 | `uint32` | 帧序号 |
| 4 | 4 字节保留字段 | 固定为 0 |
| 8 | `float64` | 帧时间戳，单位秒 |
| 16 | `uint8` | lidar ID（多 topic 模式下的设备索引） |
| 17 | 3 字节保留字段 | 固定为 0 |

随后每点为 24 字节，与 `Mid360PointMessage` 的字段布局一致：

```text
float32 x, y, z, intensity
uint32 offset_time
uint8 tag, line
uint16 reserved
```

`offset_time` 是相对帧时间戳的纳秒偏移，超过 `uint32` 范围时取最大值；
`tag` 和 `line` 保留 MID360 原始值，`reserved` 固定为 0。驱动实现不引用
`msg/nav_msgs/mid360_point_cloud_message.hpp`。

> 切换格式时，下游节点必须使用匹配的反序列化器。

## 使用 dataflow 启动

驱动目录中的 `dataflow.yml` 默认以 `MID360` 格式启动节点。在仓库根目录执行：

```bash
dora up
dora start modules/drivers/livox_driver/dataflow.yml
```

如需切换格式，修改 `dataflow.yml` 中的 `POINT_CLOUD_FORMAT`，可选值为
`"XYZI"`、`"XYZITRR"` 或 `"MID360"`。
