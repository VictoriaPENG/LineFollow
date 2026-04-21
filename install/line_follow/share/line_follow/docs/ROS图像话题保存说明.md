# ROS 图像话题保存说明

本文档说明如何在板端把 ROS 图像话题保存下来，适用于：

- 保存原始相机图像
- 保存视觉节点发布的调试图像
- 后续离线回放和问题复现

推荐优先使用 `ros2 bag`，不要一开始就直接存 `png`。

## 1. 先确认要保存哪些图像话题

先列出当前图像类话题：

```bash
ros2 topic list | grep -E "image|debug|hbmem"
```

本工程常见图像话题：

- `/image`
- `/hbmem_img`
- `/line_follow/debug_binary`
- `/line_follow/debug_detect`
- `/line_follow/debug_angle_curve`

进一步确认消息类型：

```bash
ros2 topic type /image
ros2 topic type /hbmem_img
ros2 topic type /line_follow/debug_detect
```

常见情况：

- `/image` 和调试图像通常是标准 `sensor_msgs/msg/Image`
- `/hbmem_img` 是板端零拷贝图像消息，不一定适合直接导出成普通图片

## 2. 保存前先打开调试图像发布

如果你想保存视觉调试图像，先确认视觉节点已经发布这些话题：

```bash
ros2 param set /line_follow_angle_node publish_debug_image true
ros2 topic list | grep /line_follow/debug
```

如果图像话题名被改过，也要先确认当前真实话题名。

## 3. 推荐方式一：录制 ros2 bag

这是现场最稳妥的方式。

先创建目录：

```bash
RUN_DIR=/userdata/test_logs/$(date +%F_%H%M%S)
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"
```

### 3.1 录制普通图像 `/image`

```bash
ros2 bag record \
  /image \
  /line_follow/debug_binary \
  /line_follow/debug_detect \
  /line_follow/debug_angle_curve \
  /line_follow/line_angle_deg \
  /motor_speed_cmd
```

### 3.2 录制板端图像 `/hbmem_img`

```bash
ros2 bag record \
  /hbmem_img \
  /line_follow/debug_binary \
  /line_follow/debug_detect \
  /line_follow/debug_angle_curve \
  /line_follow/line_angle_deg \
  /motor_speed_cmd
```

### 3.3 只录最小必要集合

如果空间紧张，至少保留：

```bash
ros2 bag record \
  /line_follow/debug_detect \
  /line_follow/line_angle_deg \
  /motor_speed_cmd
```

## 4. 推荐方式二：直接导出单帧图片

如果你只是想抓几张图做人工检查，可以用下面的方法。

### 4.1 查看单帧是否正常

```bash
ros2 topic echo --once /line_follow/debug_detect
```

这只能确认消息存在，不适合真正看图。

### 4.2 使用订阅脚本按帧保存

如果目标是落盘成 `png/jpg`，建议单独写一个小订阅节点，订阅这些标准图像话题：

- `/image`
- `/line_follow/debug_binary`
- `/line_follow/debug_detect`
- `/line_follow/debug_angle_curve`

保存成带时间戳文件名，例如：

- `debug_detect_1713088800_123.png`
- `debug_binary_1713088800_456.png`

注意：

- `hbmem` 话题通常不能直接按普通 `sensor_msgs/msg/Image` 方式处理
- 如果原始输入是 `/hbmem_img`，建议同时保存调试图像话题，因为这些通常已经是标准 ROS 图像消息

## 5. 如何检查 bag 里有没有录到图像

录制完成后检查：

```bash
ros2 bag info rosbag2_*
```

重点确认：

- 目标图像话题是否在列表里
- 每个图像话题是否有非零消息数

## 6. 推荐保存策略

现场调试建议按下面顺序：

1. 先打开 `publish_debug_image`
2. 再录 `ros2 bag`
3. 同时保存运行日志
4. 必要时再额外抓单帧图片

原因：

- `bag` 能保留时间戳和多话题同步关系
- 后续可重复回放
- 单帧图片只能看现象，不能做时序分析

## 7. 常见问题

### 7.1 `Node not found`

说明目标节点没起来。先检查：

```bash
ros2 node list | grep line_follow
```

### 7.2 看不到 `/line_follow/debug_*`

先确认：

```bash
ros2 param get /line_follow_angle_node publish_debug_image
```

如果是 `False`，先打开：

```bash
ros2 param set /line_follow_angle_node publish_debug_image true
```

### 7.3 录到了 `/hbmem_img` 但不好直接看

这是正常现象。`hbmem` 更适合现场高效传输和保存，不一定适合直接转普通图片。

如果你的目标是人工看图，优先保存：

- `/line_follow/debug_detect`
- `/line_follow/debug_binary`
- `/line_follow/debug_angle_curve`

## 8. 相关文档

- [板端运行时数据保存指南.md](/home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow/Markdown/板端运行时数据保存指南.md)
- [整机调试指南.md](/home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow/Markdown/整机调试指南.md)
