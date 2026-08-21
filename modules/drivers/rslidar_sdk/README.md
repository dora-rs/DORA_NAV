# RoboSense Dora 驱动

## 点云输出格式

节点通过环境变量 `POINT_CLOUD_FORMAT` 选择 `pointcloud` output 的二进制格式。
环境变量未设置或为空时默认使用 `XYZI`，变量值区分大小写；不支持的值会使节点在启动时退出并输出错误信息。

```bash
# 保持原有格式（默认）
export POINT_CLOUD_FORMAT=XYZI

# FLIO v1 Standard 格式
export POINT_CLOUD_FORMAT=XYZITRR
```

### `XYZI`

兼容驱动原有格式，小端序：

- 8 字节 `float64` 帧时间戳（秒）
- 4 字节 `uint32` 点数
- 每点 16 字节：`float32 x, y, z, intensity`

总长度为 `12 + point_count * 16` 字节。

### `XYZITRR`

使用 FLIO v1 Standard 小端格式：40 字节头部，随后是 24 字节的点记录。头部包含
`FLIO` magic、版本、帧序号、帧时间戳、点数和 payload 长度。每个点为：

```text
float32 x, y, z, intensity, time
uint16 ring, reserved
```

`time` 是该点相对帧时间戳的秒数，`ring` 是 RoboSense 激光线束编号，`reserved` 固定为 0。
完整字段偏移与 `msg/nav_msgs/point_cloud_message.hpp` 中的 `PointXYZITRRMessage` 一致，
驱动实现本身不引用该消息头文件。

> 切换格式时，下游节点必须使用匹配的反序列化器；现有只支持 12 字节头部 `XYZI`
> 的消费者不能直接读取 `XYZITRR`。

## 使用 dataflow 启动

驱动目录中的 `dataflow.yml` 默认以 `XYZITRR` 格式启动节点。在仓库根目录执行：

```bash
dora up
dora start modules/drivers/rslidar_sdk/dataflow.yml
```

如需恢复 `XYZI`，将 `dataflow.yml` 中的 `POINT_CLOUD_FORMAT` 改为 `"XYZI"`。
