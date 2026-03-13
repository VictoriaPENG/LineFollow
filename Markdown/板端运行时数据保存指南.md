# 板端运行时数据保存指南

本文说明如何在开发板运行整机程序时，保存图像、运行参数、日志和 ROS 话题记录，便于后续回放、定位问题和复现实验结果。

## 1. 目标

建议至少保存以下三类数据：

- 当前运行参数
- 程序运行日志
- ROS 话题记录，包含图像和控制数据

如果现场出现异常，建议额外保存节点列表、话题列表和关键参数查询结果。

## 2. 建立本次测试目录

每次测试都单独建一个目录，避免不同轮次的数据混在一起。

```bash
RUN_DIR=/userdata/test_logs/$(date +%F_%H%M%S)
mkdir -p "$RUN_DIR"
```

后续日志、参数、bag 数据都建议放在这个目录下。

## 3. 保存当前运行参数

本工程已经提供了参数快照脚本，可以直接导出当前运行中的参数。

```bash
bash /home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow/tools/save_runtime_params.sh "$RUN_DIR/params"
```

执行后会生成类似如下文件：

- `line_follow_angle_node.yaml`
- `line_follow_motor_model_node.yaml`
- `motor_driver_control_node.yaml`

建议在以下时机执行一次：

- 调参稳定后
- 测试结束前
- 故障发生后

## 4. 保存程序运行日志

启动整机时，把终端日志同时保存到文件。

```bash
ros2 launch line_follow line_follow_system.launch.py 2>&1 | tee "$RUN_DIR/launch.log"
```

如果是分节点启动，也可以分别保存：

```bash
ros2 run line_follow line_follow_angle_node 2>&1 | tee "$RUN_DIR/angle.log"
ros2 run line_follow line_follow_motor_model_node 2>&1 | tee "$RUN_DIR/model.log"
ros2 run line_follow motor_driver_control_node 2>&1 | tee "$RUN_DIR/driver.log"
```

日志适合用于排查：

- 节点启动失败
- 参数修改失败
- 串口重连失败
- 驱动写失败和故障锁定

## 5. 保存 ROS 话题数据

如果要把图像和控制链路一起保存，建议使用 `ros2 bag record`。

### 5.1 录制普通图像话题 `/image`

```bash
cd "$RUN_DIR"

ros2 bag record \
  /image \
  /line_follow/line_angle_deg \
  /motor_speed_cmd \
  /line_follow/debug_binary \
  /line_follow/debug_detect \
  /line_follow/debug_angle_curve
```

### 5.2 录制板端 `hbmem` 图像话题 `/hbmem_img`

```bash
cd "$RUN_DIR"

ros2 bag record \
  /hbmem_img \
  /line_follow/line_angle_deg \
  /motor_speed_cmd \
  /line_follow/debug_binary \
  /line_follow/debug_detect \
  /line_follow/debug_angle_curve
```

建议优先录制这些话题：

- 原始图像：`/image` 或 `/hbmem_img`
- 识别结果图：`/line_follow/debug_detect`
- 二值图：`/line_follow/debug_binary`
- 角度曲线图：`/line_follow/debug_angle_curve`
- 角度输出：`/line_follow/line_angle_deg`
- 电机速度命令：`/motor_speed_cmd`

如果存储空间有限，先保留以下最小集合：

```bash
ros2 bag record \
  /line_follow/line_angle_deg \
  /motor_speed_cmd \
  /line_follow/debug_detect
```

## 6. 打开调试图像发布

如果调试图像默认没有发布，先在运行时打开。

```bash
ros2 param set line_follow_angle_node publish_debug_image true
```

然后检查调试图像话题是否存在：

```bash
ros2 topic list | grep /line_follow/debug
```

如果需要修改调试图像话题名，也可以运行时设置：

```bash
ros2 param set line_follow_angle_node debug_detect_topic /line_follow/debug_detect_2
ros2 param set line_follow_angle_node debug_binary_topic /line_follow/debug_binary_2
ros2 param set line_follow_angle_node debug_angle_curve_topic /line_follow/debug_angle_curve_2
```

## 7. 故障发生时建议额外保存的信息

如果现场出现丢线、转向异常、驱动故障锁定、串口异常，建议立刻额外保存以下信息。

### 7.1 保存节点列表

```bash
ros2 node list > "$RUN_DIR/node_list.txt"
```

### 7.2 保存话题列表

```bash
ros2 topic list > "$RUN_DIR/topic_list.txt"
```

### 7.3 单独保存关键参数查询结果

```bash
ros2 param get motor_driver_control_node driver_command_timeout_sec >> "$RUN_DIR/key_params.txt"
ros2 param get motor_driver_control_node max_consecutive_write_errors >> "$RUN_DIR/key_params.txt"
ros2 param get motor_driver_control_node write_retry_count >> "$RUN_DIR/key_params.txt"
ros2 param get line_follow_motor_model_node heading_gain >> "$RUN_DIR/key_params.txt"
ros2 param get line_follow_motor_model_node base_speed_ratio >> "$RUN_DIR/key_params.txt"
```

### 7.4 再导出一份完整参数快照

```bash
bash /home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow/tools/save_runtime_params.sh "$RUN_DIR/params_after_fault"
```

## 8. 推荐的实际操作流程

建议至少开三个终端。

### 终端 1：启动整机并保存日志

```bash
RUN_DIR=/userdata/test_logs/$(date +%F_%H%M%S)
mkdir -p "$RUN_DIR"

ros2 launch line_follow line_follow_system.launch.py 2>&1 | tee "$RUN_DIR/launch.log"
```

### 终端 2：录制 bag，保存图像和控制话题

如果使用 `hbmem`：

```bash
cd "$RUN_DIR"

ros2 bag record \
  /hbmem_img \
  /line_follow/line_angle_deg \
  /motor_speed_cmd \
  /line_follow/debug_binary \
  /line_follow/debug_detect \
  /line_follow/debug_angle_curve
```

如果使用普通图像：

```bash
cd "$RUN_DIR"

ros2 bag record \
  /image \
  /line_follow/line_angle_deg \
  /motor_speed_cmd \
  /line_follow/debug_binary \
  /line_follow/debug_detect \
  /line_follow/debug_angle_curve
```

### 终端 3：保存参数快照和临时查询

```bash
bash /home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow/tools/save_runtime_params.sh "$RUN_DIR/params"
```

调参后可以重复执行一次，保存最终稳定参数。

## 9. 如果想直接保存成图片文件

如果你的目标不是后续回放，而是想直接得到 `png` 或 `jpg` 文件，建议额外写一个图像订阅节点，把以下话题按帧保存：

- `/image` 或 `/hbmem_img`
- `/line_follow/debug_detect`
- `/line_follow/debug_binary`
- `/line_follow/debug_angle_curve`

当前最稳妥的现场保存方式仍然是先录 `ros2 bag`，因为：

- 不会漏掉时间戳
- 能保留多话题同步关系
- 后续可以重复回放

## 10. 建议最少保存哪些内容

如果你不想保存太多数据，最少建议保留以下内容：

- `launch.log`
- 一份参数快照 YAML
- 一个 `ros2 bag` 目录

这三类数据通常已经足够排查大多数现场问题。

## 11. 推荐目录结构

一次测试结束后，目录结构建议类似如下：

```text
/userdata/test_logs/2026-03-24_153000/
├── launch.log
├── node_list.txt
├── topic_list.txt
├── key_params.txt
├── params/
│   ├── line_follow_angle_node.yaml
│   ├── line_follow_motor_model_node.yaml
│   └── motor_driver_control_node.yaml
├── params_after_fault/
│   ├── line_follow_angle_node.yaml
│   ├── line_follow_motor_model_node.yaml
│   └── motor_driver_control_node.yaml
└── rosbag2_xxx/
```

## 12. 一句话建议

现场记录优先级建议按下面顺序执行：

1. 先保日志
2. 再保参数
3. 同时录 bag
4. 出现异常后补保存节点列表、话题列表和关键参数
