# RDK X5 部署与逐步验证

本文档用于将本 ROS2 Python 包部署到 RDK X5，并按阶段完成验证，避免一次性全量启动后难以排障。

## 1. 目录内容

本方案已整理为标准 `ament_python` 包，包名为 `line_follow`。当前仓库源码目录是 `src/`，安装后的 Python 包名是 `line_follow`。包含 3 个核心节点和 1 个整系统 launch：

- `src/angle_node.py`
  视觉巡线角度检测，发布 `/line_follow/line_angle_deg`
- `src/motor_model_node.py`
  将视觉角度转换为左右电机目标 RPM，发布 `/motor_speed_cmd`
- `src/motor_driver_control.py`
  订阅 `/motor_speed_cmd`，通过 Modbus RTU 下发给驱动器
- `launch/line_follow_system.launch.py`
  一次启动相机、角度检测、运动模型、驱动控制

## 2. 前提条件

RDK X5 上需要具备以下运行环境：

- ROS2 环境可正常 `source`
- Python3 可用
- 已安装并可用的 Python 模块：
  - `rclpy`
  - `cv2`
  - `cv_bridge`
  - `serial` 或 `pyserial`
- 已安装并可用的 ROS2 包：
  - `hobot_usb_cam`
  - `websocket`
  - `hobot_codec`
  - `hobot_shm`

部署前先在 X5 上检查：

```bash
python3 -c "import rclpy, cv2, serial; print('python deps ok')"
ls /dev/video*
ls /dev/ttyUSB*
```

如果 `cv_bridge` 需要验证，可执行：

```bash
python3 -c "from cv_bridge import CvBridge; print('cv_bridge ok')"
```

## 3. 部署到 RDK X5

假设 X5 用户名为 `sunrise`，IP 为 `192.168.127.10`，目标工作空间为 `/userdata/dev_ws`。

在开发机执行：

```bash
ssh sunrise@192.168.127.10 "mkdir -p /userdata/dev_ws/src/originbot"
scp -r /home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow sunrise@192.168.127.10:/userdata/dev_ws/src/originbot/
```

登录 X5：

```bash
ssh sunrise@192.168.127.10
cd /userdata/dev_ws
```

编译工作空间：

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j1
colcon build --executor sequential --parallel-workers 1 --packages-select line_follow
```

说明：

- `--executor sequential` 强制顺序构建
- `--parallel-workers 1` 限制为单任务
- `MAKEFLAGS=-j1` 限制底层编译任务并发

如果 X5 内存仍然紧张，编译前建议关闭其他占内存进程，并确认已配置交换分区。

如果尚未配置 swap，可先临时创建 2GB 交换文件：

```bash
fallocate -l 2G /userdata/swapfile
chmod 600 /userdata/swapfile
mkswap /userdata/swapfile
swapon /userdata/swapfile
free -h
```

编译完成后如果想关闭临时 swap：

```bash
swapoff /userdata/swapfile
```

如果要长期保留，再额外把 `/userdata/swapfile none swap sw 0 0` 写入 `/etc/fstab`。

## 4. 环境准备

每次启动前先加载 ROS2 环境。实际路径按你的 X5 安装位置调整：

```bash
source /opt/ros/humble/setup.bash
```

然后加载本包：

```bash
source /userdata/dev_ws/install/setup.bash
ros2 pkg list | grep line_follow
```

后续每打开一个新终端，都建议重复执行：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
```

## 5. 分阶段验证

验证时务必先将履带架空，避免参数未调好时直接冲车。

### 5.1 验证摄像头链路

先只启动摄像头与 web：

```bash
ros2 launch line_follow usb_cam_web.launch.py device:=/dev/video8
```

另开一个终端检查话题：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 topic list
ros2 topic hz /image
```

预期结果：

- `/image` 存在
- 图像频率正常
- Web 页面能看到实时画面

如果 `/image` 不存在，先排查相机设备号是否正确。

### 5.2 验证视觉角度节点

启动角度检测节点：

```bash
ros2 run line_follow line_follow_angle_node \
  --ros-args -p image_topic:=/image -p line_is_white:=false -p show_debug:=false
```

另开终端观察输出：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 topic echo /line_follow/line_angle_deg
```

预期结果：

- 相机看到引导线时，`/line_follow/line_angle_deg` 持续输出角度
- 线条右倾时角度为正，左倾时角度为负

如果没有输出：

- 调整 `line_is_white`
- 调整阈值 `thresh`
- 调整窗口参数 `margin`、`minpix`

### 5.3 验证运动模型节点

启动运动模型节点：

```bash
ros2 run line_follow line_follow_motor_model_node \
  --ros-args -p track_width_m:=0.577 -p heading_gain:=1.8 -p base_speed_ratio:=0.32 -p speed_reduce_gain:=0.55 -p allow_reverse:=true
```

另开终端查看速度指令：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 topic echo /motor_speed_cmd
```

预期结果：

- 角度接近 0 时，左右 RPM 接近相等
- 线偏右时，右侧 RPM 下降、左侧 RPM 升高
- 线偏左时，左侧 RPM 下降、右侧 RPM 升高
- 大角度时，内侧履带允许出现负 RPM，对应驱动器反转模式 `0703`

如果差速方向反了，有两种修正办法：

- 交换左右驱动站号
- 将 `heading_gain` 的符号逻辑改掉

### 5.4 验证驱动通信节点

接好 RS485/串口，确认驱动器地址与脚本参数一致，再启动：

```bash
ros2 run line_follow motor_driver_control_node \
  --ros-args -p serial_port:=/dev/ttyUSB0 -p left_slave:=6 -p right_slave:=8 -p min_speed_rpm:=0 -p max_speed_rpm:=5000
```

另开终端手工发一个测试速度：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 topic pub --once /motor_speed_cmd std_msgs/msg/Float32MultiArray "{data: [300.0, 300.0]}"
```

预期结果：

- 两侧电机低速转动
- 程序日志无串口打开失败、无 Modbus 超时

在手工发速度前，建议先确认串口节点已经成功打开，并且电机履带处于架空状态。

再做差速测试：

```bash
ros2 topic pub --once /motor_speed_cmd std_msgs/msg/Float32MultiArray "{data: [450.0, 200.0]}"
```

预期结果：

- 左右两侧速度明显不同

再做反转测试：

```bash
ros2 topic pub --once /motor_speed_cmd std_msgs/msg/Float32MultiArray "{data: [300.0, -300.0]}"
```

预期结果：

- 一侧正转，另一侧反转
- 驱动器向 `8106` 写入 `0701/0703`

如果电机不转：

- 检查串口号是否正确
- 检查 485 A/B 线
- 检查驱动器站号 `left_slave/right_slave`
- 检查驱动器是否允许外部闭环速度模式

### 5.5 全链路联调

确认单节点验证都通过后，再启动整系统：

```bash
ros2 launch line_follow line_follow_system.launch.py \
  device:=/dev/video8 \
  image_topic:=/image \
  serial_port:=/dev/ttyUSB0 \
  track_width_m:=0.577 \
  allow_reverse:=true \
  left_slave:=6 \
  right_slave:=8 \
  show_debug:=false \
  line_is_white:=false
```

联调时建议同时观察：

```bash
ros2 topic echo /line_follow/line_angle_deg
ros2 topic echo /motor_speed_cmd
```

预期结果：

- 检测到线时有角度输出
- `/motor_speed_cmd` 随角度变化
- 电机响应差速变化
- 视觉输入丢失超过超时阈值后自动停机

## 6. 标定建议

### 6.1 `track_width_m`

这是最关键的参数，因为题目提供的数据里没有左右履带中心距。建议实测左右履带中心线距离后写入。

现象与修正：

- 转向偏软：适当增大 `heading_gain`
- 转向过猛：减小 `heading_gain`
- 整体太快：减小 `base_speed_ratio`
- 弯道跟踪时冲出：增大 `speed_reduce_gain`

### 6.2 黑线/白线

如果场地是黑线白底：

```bash
line_is_white:=false
```

如果场地是白线黑底：

```bash
line_is_white:=true
```

## 7. 常见故障排查

### 7.1 没有图像

- 检查 `/dev/video*`
- 检查 `device:=/dev/videoX`
- 检查相机权限

### 7.2 有图像但没有角度

- 调 `line_is_white`
- 调阈值和窗口参数
- 用 `show_debug:=true` 观察二值化和滑窗结果

### 7.3 有角度但没有速度

- 检查 `/line_follow/line_angle_deg` 是否持续发布
- 检查 `/motor_speed_cmd` 是否存在
- 检查模型节点是否因为超时发了 `0,0`

### 7.4 有速度但电机不动

- 检查串口和 RS485 接线
- 检查站号
- 检查驱动器模式配置
- 检查供电和急停

## 8. 推荐上线顺序

推荐按下面顺序上线，不要跳步：

1. 只通相机
2. 只看角度话题
3. 只看速度话题
4. 手工发速度验证驱动
5. 全链路架空联调
6. 低速落地测试
7. 逐步提高 `base_speed_ratio`
