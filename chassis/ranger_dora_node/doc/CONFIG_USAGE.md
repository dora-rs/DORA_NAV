# YAML配置文件使用说明

## 概述

Ranger DORA节点现在支持从YAML配置文件加载参数，替代了之前的环境变量方式。

## 配置文件位置

默认配置文件路径：`config/ranger_config.yaml`

可以通过环境变量 `RANGER_CONFIG_FILE` 指定自定义配置文件路径：

```bash
export RANGER_CONFIG_FILE=/path/to/your/config.yaml
```

## 配置文件格式

```yaml
# CAN interface configuration
can_port: "can0"

# Robot model selection
# Options: "ranger", "ranger_mini_v1", "ranger_mini_v2", "ranger_mini_v3"
robot_model: "ranger_mini_v3"

# Frame IDs
odom_frame: "odom"
base_frame: "base_link"

# Update rate in Hz
update_rate: 100

# Odometry topic name
odom_topic_name: "odom"

# TF publishing (not supported in DORA, kept for compatibility)
publish_odom_tf: false
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|-------|------|--------|------|
| `can_port` | string | "can0" | CAN总线接口名称 |
| `robot_model` | string | "ranger_mini_v3" | 机器人型号 |
| `odom_frame` | string | "odom" | 里程计坐标系名称 |
| `base_frame` | string | "base_link" | 基座坐标系名称 |
| `update_rate` | int | 100 | 状态更新频率（Hz） |
| `odom_topic_name` | string | "odom" | 里程计话题名称 |
| `publish_odom_tf` | bool | false | 是否发布TF变换（DORA不支持） |

## 支持的机器人型号

- `ranger` - 标准Ranger
- `ranger_mini_v1` - Ranger Mini V1（支持侧滑模式）
- `ranger_mini_v2` - Ranger Mini V2
- `ranger_mini_v3` - Ranger Mini V3（推荐）

## 使用方法

### 方法1：使用默认配置文件

```bash
# 确保配置文件存在
ls config/ranger_config.yaml

# 直接运行
dora start ranger_miniv3_dataflow.yml
```

### 方法2：使用自定义配置文件

```bash
# 设置环境变量
export RANGER_CONFIG_FILE=/path/to/custom_config.yaml

# 运行
dora start ranger_miniv3_dataflow.yml
```

### 方法3：在dataflow.yml中指定

修改 `ranger_miniv3_dataflow.yml`：

```yaml
nodes:
  - id: ranger_miniv3_node
    custom:
      source: build/ranger_miniv3_node
      inputs:
        # ...
      outputs:
        # ...
      envs:
        RANGER_CONFIG_FILE: /path/to/custom_config.yaml
```

## 错误处理

如果YAML文件加载失败，节点会：
1. 打印错误信息
2. 使用默认参数继续运行
3. 输出使用的默认参数值

示例输出：
```
Error loading YAML config file: bad file
Using default parameters...
Successfully loaded the following parameters:
  port_name: can0
  robot_model: ranger_mini_v3
  odom_frame: odom
  base_frame: base_link
  update_rate: 100
  odom_topic_name: odom
```

## 配置文件示例

### Ranger Mini V3 配置

```yaml
can_port: "can0"
robot_model: "ranger_mini_v3"
odom_frame: "odom"
base_frame: "base_link"
update_rate: 100
odom_topic_name: "odom"
publish_odom_tf: false
```

### Ranger Mini V1 配置（支持侧滑）

```yaml
can_port: "can0"
robot_model: "ranger_mini_v1"
odom_frame: "odom"
base_frame: "base_link"
update_rate: 50
odom_topic_name: "odom"
publish_odom_tf: false
```

### 多机器人配置

如果需要运行多个机器人，可以为每个机器人创建单独的配置文件：

```bash
# 机器人1
config/robot1_config.yaml

# 机器人2
config/robot2_config.yaml
```

然后在启动时指定：

```bash
# 终端1
export RANGER_CONFIG_FILE=config/robot1_config.yaml
dora start robot1_dataflow.yml

# 终端2
export RANGER_CONFIG_FILE=config/robot2_config.yaml
dora start robot2_dataflow.yml
```

## 依赖安装

需要安装 yaml-cpp 库：

### Ubuntu/Debian

```bash
sudo apt-get install libyaml-cpp-dev
```

### 从源码安装

```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## 编译

```bash
cd ranger_dora_node
./build.sh
```

如果遇到 yaml-cpp 找不到的错误，请确认：
1. yaml-cpp 已正确安装
2. CMake 能找到 yaml-cpp（检查 `/usr/local/lib` 或 `/usr/lib`）

## 与环境变量方式的对比

| 特性 | 环境变量 | YAML配置文件 |
|------|---------|-------------|
| 易读性 | ❌ 分散 | ✅ 集中 |
| 易修改 | ❌ 需要重新设置 | ✅ 直接编辑文件 |
| 版本控制 | ❌ 不便 | ✅ 方便 |
| 多配置管理 | ❌ 困难 | ✅ 简单 |
| 注释支持 | ❌ 无 | ✅ 有 |

## 故障排除

### 问题1：找不到配置文件

```
Error loading YAML config file: bad file
```

**解决方案**：
- 检查配置文件路径是否正确
- 确认配置文件存在
- 检查文件权限

### 问题2：YAML格式错误

```
Error loading YAML config file: yaml-cpp: error at line X, column Y
```

**解决方案**：
- 检查YAML语法（缩进、冒号、引号）
- 使用在线YAML验证器检查格式
- 参考示例配置文件

### 问题3：编译错误 - 找不到yaml-cpp

```
fatal error: yaml-cpp/yaml.h: No such file or directory
```

**解决方案**：
```bash
sudo apt-get install libyaml-cpp-dev
# 或
sudo find / -name "yaml.h" 2>/dev/null
```

## 总结

使用YAML配置文件的优势：
1. ✅ 配置集中管理
2. ✅ 易于版本控制
3. ✅ 支持注释
4. ✅ 易于多配置切换
5. ✅ 更好的可读性

建议所有新项目使用YAML配置文件方式。
