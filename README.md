# Line Follow

`line_follow` 是一个基于 ROS 2 Humble 的履带车视觉巡线包，面向 OriginBot/RDK X5 板端运行。程序通过 USB 相机获取赛道图像，检测引导线的角度和横向偏移，转换成左右履带电机转速，并通过 Modbus RTU 串口下发到电机驱动器。

项目同时包含长按遥控切换、摇杆人工接管、Web 多画面调试、相机标定、运行数据记录和 systemd 开机自启配置。

## 功能概览

- USB 相机图像采集，支持 `/image -> /hbmem_img` 共享内存图像链路。
- 视觉巡线检测，输出线角度、横向偏移、检测状态和调试图像。
- 运动模型节点，将视觉误差转换为左右履带目标 RPM。
- 电机驱动节点，通过 Modbus RTU 控制左右电机驱动器。
- 长按遥控节点，用遥控前进/后退通道切换手动前进和自动巡线。
- 摇杆节点，用 GPIO 四向输入临时接管车辆控制。
- Web 调试页面，显示原图、检测图、二值图、角度曲线和电机速度状态。
- 运行时参数、rosbag 和现场日志保存脚本。

## 目录结构

```text
Line_follow/
├── src/                    # ROS 2 Python 节点源码
├── launch/                 # 启动文件
├── config/                 # 巡线、遥控、电机和相机标定参数
├── board_tools/            # 板端辅助脚本
├── tools/                  # 开发、同步、标定和调试工具
├── Markdown/               # 细分调试和部署文档
├── systemd/                # 开机自启服务模板
├── package.xml
└── setup.py
```

## 主要节点

| 节点 | 入口 | 作用 |
| --- | --- | --- |
| `line_follow_angle_node` | `src/angle_node.py` | 订阅相机图像，检测引导线，发布角度、偏移、检测状态和调试图像 |
| `line_follow_motor_model_node` | `src/motor_model_node.py` | 根据视觉误差生成左右履带目标转速 |
| `motor_driver_control_node` | `src/motor_driver_control.py` | 订阅速度命令，通过串口 Modbus RTU 写入电机驱动器 |
| `remote_long_press_start_line_follow_node` | `src/remote_long_press_start_line_follow_node.py` | 监听遥控 GPIO，长按切换巡线/手动模式 |
| `joystick_drive_node` | `src/joystick_drive_node.py` | 监听摇杆 GPIO，发布人工接管速度 |
| `web_debug_dashboard_node` | `src/web_debug_dashboard_node.py` | 提供四宫格 MJPEG 调试页面和电机速度状态 |

## 依赖环境

- ROS 2 Humble
- Python 3.10
- RDK X5 板端图像链路相关包：
  - `hobot_usb_cam`
  - `hobot_codec`
  - `hobot_shm`
  - `hbm_img_msgs`
  - `websocket`
- Python/ROS 依赖：
  - `rclpy`
  - `sensor_msgs`
  - `std_msgs`
  - `cv_bridge`
  - `python3-serial`
  - `opencv-python` 或系统 OpenCV Python 绑定

## 编译

在工作空间根目录执行：

```bash
cd /userdata/dev_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select line_follow
source install/setup.bash
```

开发机同步到板端的流程见 [Markdown/代码同步与编译指南.md](Markdown/代码同步与编译指南.md)。

## 快速启动

### 推荐整机启动

整机运行推荐使用 `joystick_remote_hybrid.launch.py`。它会统一启动相机、图像解码、巡线核心、电机驱动、长按遥控、摇杆和 Web 调试页。

```bash
ros2 launch line_follow joystick_remote_hybrid.launch.py \
  device:=/dev/video0 \
  enable_websocket:=true \
  enable_dashboard:=true
```

默认 dashboard 地址：

```text
http://<板端IP>:8091/
```

### 只启动巡线核心

如果相机链路已经由其他服务提供，只启动视觉检测、运动模型和驱动：

```bash
ros2 launch line_follow line_follow_core.launch.py \
  image_topic:=/hbmem_img \
  image_msg_type:=hbmem \
  serial_port:=/dev/ttyUSB0
```

### 启动相机和 Web 图像链路

```bash
ros2 launch line_follow usb_cam_web.launch.py \
  device:=/dev/video0 \
  enable_websocket:=true \
  enable_dashboard:=true
```

### 启动相机 + 巡线核心

```bash
ros2 launch line_follow line_follow_system.launch.py \
  device:=/dev/video0 \
  enable_websocket:=false \
  enable_dashboard:=true
```

## 常用参数

主要参数文件：

- `config/line_follow_params.yaml`：视觉检测和运动模型参数。
- `config/runtime_params.yaml`：长按遥控、摇杆、电机驱动参数。
- `config/camera_calibration.json`、`config/camera_calibration.yaml`：USB 相机标定结果。

常调参数：

| 参数 | 所属节点 | 说明 |
| --- | --- | --- |
| `image_topic` | `line_follow_angle_node` | 输入图像话题，RDK 默认 `/hbmem_img` |
| `image_msg_type` | `line_follow_angle_node` | 输入类型，RDK 共享内存用 `hbmem`，普通 ROS 图像用 `raw` |
| `detection_method` | `line_follow_angle_node` | 检测方法，支持 `threshold`、`clahe_adaptive`、`edge_line` |
| `line_is_white` | `line_follow_angle_node` | 目标线颜色，`false` 表示黑线白底 |
| `thresh` | `line_follow_angle_node` | 二值化阈值，`-1` 表示 Otsu 自动阈值 |
| `roi_*` | `line_follow_angle_node` | 检测 ROI 范围 |
| `angle_bias_deg` | `line_follow_angle_node` | 相机安装角度补偿 |
| `target_offset_px` | `line_follow_angle_node` | 直行参考横向偏移 |
| `base_motor_rpm` | `line_follow_motor_model_node` | 巡线基础速度 |
| `heading_gain` | `line_follow_motor_model_node` | 航向误差转差速的增益 |
| `offset_gain_deg` | `line_follow_motor_model_node` | 横向偏移折算为等效角度的增益 |
| `max_motor_rpm` | `line_follow_motor_model_node` | 巡线输出转速上限 |
| `serial_port` | `motor_driver_control_node` | 电机驱动串口，默认 `/dev/ttyUSB0` |
| `left_slave` / `right_slave` | `motor_driver_control_node` | 左右电机驱动器 Modbus 站号 |

运行时临时调参示例：

```bash
ros2 param set /line_follow_angle_node thresh 115
ros2 param set /line_follow_motor_model_node base_motor_rpm 400.0
ros2 param set /line_follow_motor_model_node heading_gain 20.0
```

## 关键话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/image` | `sensor_msgs/Image` 或压缩图像链路 | USB 相机原始图像链路 |
| `/hbmem_img` | `hbm_img_msgs/HbmMsg1080P` | RDK 共享内存图像 |
| `/line_follow/line_angle_deg` | `std_msgs/Float32` | 检测到的线角度 |
| `/line_follow/visual_heading_error_deg` | `std_msgs/Float32` | 补偿后的视觉航向误差 |
| `/line_follow/visual_lateral_error_norm` | `std_msgs/Float32` | 归一化横向误差 |
| `/line_follow/line_detected` | `std_msgs/Bool` | 是否检测到有效线 |
| `/line_follow/set_enabled` | `std_msgs/Bool` | 巡线使能控制 |
| `/motor_speed_cmd` | `std_msgs/Float32MultiArray` | 自动/遥控基础速度命令 `[left_rpm, right_rpm]` |
| `/joystick_motor_speed_cmd` | `std_msgs/Float32MultiArray` | 摇杆接管速度命令 |
| `/joystick_override_active` | `std_msgs/Bool` | 摇杆接管状态 |
| `/motor_speed_status` | `std_msgs/Float32MultiArray` | 驱动节点发布的当前速度状态 |
| `/line_follow/debug_undistorted` | `sensor_msgs/Image` | 去畸变图像 |
| `/line_follow/debug_detect` | `sensor_msgs/Image` | 检测调试图 |
| `/line_follow/debug_binary` | `sensor_msgs/Image` | 二值图 |
| `/line_follow/debug_angle_curve` | `sensor_msgs/Image` | 角度历史曲线 |

## 遥控和摇杆默认 GPIO

默认使用 40Pin 物理排针编号，即 `use_board_numbering: true`。

| 功能 | 默认引脚 |
| --- | --- |
| 遥控前进 | 31 |
| 遥控后退/巡线触发 | 29 |
| 摇杆前进 | 11 |
| 摇杆后退 | 15 |
| 摇杆左转 | 13 |
| 摇杆右转 | 16 |

联合启动时默认逻辑：

- 长按后退超过 `0.5s`：发布 `/line_follow/set_enabled=true`，进入巡线。
- 长按前进：退出巡线并发布手动前进速度。
- 松开按键：停车。
- 摇杆有效时：`motor_driver_control_node` 优先执行 `/joystick_motor_speed_cmd`。

## 开机自启

当前推荐只保留一个 systemd 服务：

```text
line-follow-hybrid.service
```

安装和启停方法见 [systemd/README.md](systemd/README.md)。不要同时运行旧的 `line-follow-remote.service`、`line-follow-stack.service`、`line-follow-web.service`，否则容易出现相机、串口或节点重复启动。

## 调试命令

查看节点：

```bash
ros2 node list
```

查看关键话题：

```bash
ros2 topic list | grep -E 'line_follow|motor|joystick|hbmem|image'
```

确认只有一个巡线模型节点：

```bash
ros2 node list | grep line_follow_motor_model_node | wc -l
```

查看检测是否成功：

```bash
ros2 topic echo /line_follow/line_detected
ros2 topic echo /line_follow/visual_heading_error_deg
ros2 topic echo /line_follow/visual_lateral_error_norm
```

查看电机命令：

```bash
ros2 topic echo /motor_speed_cmd
ros2 topic echo /motor_speed_status
```

查看 systemd 日志：

```bash
journalctl -u line-follow-hybrid.service -n 100 --no-pager
```

## 常见问题

### Web 页面打不开

确认启动时打开了：

```bash
enable_websocket:=true enable_dashboard:=true
```

再检查服务日志和端口：

```bash
journalctl -u line-follow-hybrid.service -n 100 --no-pager
```

dashboard 默认端口是 `8091`。

### 没有巡线输出

依次检查：

```bash
ros2 topic echo /hbmem_img --once
ros2 topic echo /line_follow/line_detected
ros2 topic echo /line_follow/visual_heading_error_deg
ros2 topic echo /motor_speed_cmd
```

如果 `/hbmem_img` 没有数据，优先检查相机、`hobot_codec` 和 `hobot_shm` 链路。如果检测失败，优先调 `thresh`、`roi_*`、`line_is_white`、`detection_method`。

### 电机不转

检查：

- 串口是否是 `/dev/ttyUSB0`。
- `left_slave`、`right_slave` 是否与驱动器站号一致。
- `/motor_speed_cmd` 是否有非零速度。
- `/motor_speed_status` 是否更新。
- `motor_driver_control_node` 日志是否有串口打开失败或 Modbus 写失败。

### 出现重复节点或抢串口

停止旧服务和残留节点：

```bash
systemctl stop line-follow-remote.service line-follow-stack.service line-follow-web.service line-follow-hybrid.service || true
pkill -f line_follow || true
systemctl start line-follow-hybrid.service
```

## 相关文档

- [Markdown/代码同步与编译指南.md](Markdown/代码同步与编译指南.md)
- [Markdown/程序状态监控常用命令.md](Markdown/程序状态监控常用命令.md)
- [Markdown/摇杆遥控器联合控制调试指南.md](Markdown/摇杆遥控器联合控制调试指南.md)
- [Markdown/开发机保存网页调试图像.md](Markdown/开发机保存网页调试图像.md)
- [Markdown/USB相机标定指南.md](Markdown/USB相机标定指南.md)
- [Markdown/Gazebo仿真验证指南.md](Markdown/Gazebo仿真验证指南.md)
- [Markdown/真实车体姿态接入教程.md](Markdown/真实车体姿态接入教程.md)
- [systemd/README.md](systemd/README.md)
