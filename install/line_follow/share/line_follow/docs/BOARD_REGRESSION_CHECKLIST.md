# 板端回归测试清单

建议按下面顺序在板端执行，前 1 到 6 项架空测试，确认通过后再落地。

## 1. 环境与编译

执行：

```bash
source /opt/ros/humble/setup.bash
cd /userdata/dev_ws
colcon build --packages-select line_follow
source /userdata/dev_ws/install/setup.bash
```

检查点：

- 编译成功
- `ros2 pkg executables line_follow` 能看到 3 个节点
- `bash /userdata/dev_ws/src/line_follow/tools/save_runtime_params.sh /tmp/lf_params_smoke` 可执行

## 2. 正常启动整机

执行：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 launch line_follow line_follow_system.launch.py \
  device:=/dev/video0 \
  image_topic:=/hbmem_img \
  image_msg_type:=hbmem \
  serial_port:=/dev/ttyUSB0
```

检查点：

- 3 个节点都起来
- `ros2 node list | grep -E 'line_follow_angle_node|line_follow_motor_model_node|motor_driver_control_node'`
- 无 Python 异常退出

## 3. 视觉链路正常

执行：

```bash
ros2 topic list | grep -E '^/hbmem_img$|^/line_follow/line_angle_deg$'
ros2 topic echo /line_follow/line_angle_deg
```

检查点：

- 有图像时能持续输出角度
- 视觉丢线时日志有 `line detection failed`
- 恢复后有 recovered 日志

## 4. 动态打开或关闭调试图像

执行：

```bash
ros2 param set line_follow_angle_node publish_debug_image false
ros2 topic list | grep /line_follow/debug
ros2 param set line_follow_angle_node publish_debug_image true
ros2 topic list | grep /line_follow/debug
```

检查点：

- 关闭后调试图像不再发布
- 打开后重新发布

再测改话题名：

```bash
ros2 param set line_follow_angle_node debug_detect_topic /line_follow/debug_detect_2
ros2 topic list | grep /line_follow/debug_detect_2
```

## 5. 运动模型运行时调参

执行：

```bash
ros2 param get line_follow_motor_model_node heading_gain
ros2 param set line_follow_motor_model_node heading_gain 2.2
ros2 topic echo /motor_speed_cmd
```

检查点：

- 改参立即生效
- `/motor_speed_cmd` 变化符合预期
- `center_motor_rpm` 和 `delta_motor_rpm` 与实际下发趋势一致

## 6. 驱动节点正常收命令与超时停车

执行：

```bash
ros2 topic pub --once /motor_speed_cmd std_msgs/msg/Float32MultiArray "{data: [300.0, -300.0]}"
```

检查点：

- 电机有响应
- 超过 `driver_command_timeout_sec` 后自动停机

可先看参数：

```bash
ros2 param get motor_driver_control_node driver_command_timeout_sec
```

## 7. 驱动热更新成功路径

执行：

```bash
ros2 param set motor_driver_control_node max_speed_rpm 2800
ros2 param set motor_driver_control_node write_retry_count 3
```

检查点：

- 返回成功
- 节点不中断

再测串口参数热更新：

```bash
ros2 param set motor_driver_control_node serial_timeout_sec 0.2
```

检查点：

- 返回成功
- 串口重开后节点仍可收命令

## 8. 驱动热更新失败路径

执行：

```bash
ros2 param set motor_driver_control_node serial_port /dev/not_exist
```

预期：

- 命令返回失败
- 旧串口仍保持可用

验证：

```bash
ros2 topic pub --once /motor_speed_cmd std_msgs/msg/Float32MultiArray "{data: [300.0, -300.0]}"
```

## 9. 站号热更新失败路径

执行：

```bash
ros2 param set motor_driver_control_node left_slave 99
```

预期：

- 返回失败
- 旧站号不被覆盖

验证：

```bash
ros2 topic pub --once /motor_speed_cmd std_msgs/msg/Float32MultiArray "{data: [300.0, -300.0]}"
```

## 10. 写失败锁定

做法：

- 人为断开 RS485，或关闭驱动器供电
- 再发速度命令

```bash
ros2 topic pub --once /motor_speed_cmd std_msgs/msg/Float32MultiArray "{data: [300.0, -300.0]}"
```

检查点：

- 日志出现重试
- 达到 `max_consecutive_write_errors` 后进入 fault latch
- 电机被尝试停机
- 后续命令被拒绝

## 11. 故障复位

先恢复硬件，再执行：

```bash
ros2 param set motor_driver_control_node fault_reset_counter 1
```

检查点：

- 日志出现 fault reset
- 再发速度命令可恢复工作

二次复位测试：

```bash
ros2 param set motor_driver_control_node fault_reset_counter 2
```

## 12. 参数快照保存

执行：

```bash
bash /userdata/dev_ws/src/line_follow/tools/save_runtime_params.sh /tmp/lf_params_01
ls -l /tmp/lf_params_01
```

检查点：

- 生成 3 个 yaml
- 文件非空
- 参数值与当前运行时一致

## 13. 停机与重启

执行 `Ctrl+C` 停掉整机。

检查点：

- 驱动节点退出前有停机动作
- 重启后可正常恢复

## 14. 低速落地回归

建议参数：

```bash
base_speed_ratio:=0.22
heading_gain:=1.6
max_speed_rpm:=2800
```

检查点：

- 直行稳定
- 转向不过猛
- 丢线能停
- 故障恢复后无异常残留

## 重点判定标准

- 失败时必须“返回失败或进入故障锁定”，不能假成功。
- 故障时必须“停机优先”。
- 运行时调参后，旧可用配置不能被错误覆盖。
- 参数快照必须能导出当前实际值。
