# RDK X5 部署与逐步验证

本文档用于将本 ROS2 Python 包部署到 RDK X5，并按阶段完成验证，避免一次性全量启动后难以排障。

如果你当前还在开发机上单独调试视觉算法，建议先阅读：

- `Markdown/LOCAL_VISUAL_DEBUG.md`

## 1. 目录内容

本方案已整理为标准 `ament_python` 包，包名为 `line_follow`。当前仓库源码目录是 `src/`，安装后的 Python 包名是 `line_follow`。包含 3 个核心节点和 1 个整系统 launch：

- `src/angle_node.py`
  视觉巡线角度检测，发布 `/line_follow/line_angle_deg`
- `src/motor_model_node.py`
  将视觉角度转换为左右电机目标 RPM，发布 `/motor_speed_cmd`
- `src/motor_driver_control.py`
  订阅 `/motor_speed_cmd`，通过 Modbus RTU 下发给驱动器
- `launch/line_follow_system.launch.py`
  一次启动相机、角度检测、运动模型、驱动控制，并可透传关键运行参数

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

假设 X5 用户名为 `root`，IP 为 `192.168.127.10`，目标工作空间为 `/userdata/dev_ws`。

在开发机执行：

```bash
ssh root@192.168.127.10 "mkdir -p /userdata/dev_ws/src/originbot"
bash /home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow/tools/sync_to_board.sh
```

说明：

- 这里仍然使用 `rsync`，但统一通过 `tools/sync_to_board.sh` 调用，避免每次手写排除参数
- 同步脚本默认排除本地编译产物：`build/`、`install/`、`log/`、`__pycache__/`、`*.pyc`
- 如果目标目录不存在，`rsync` 会自动创建 `Line_follow/` 下的内容结构

登录 X5：

```bash
ssh root@192.168.127.10
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

## 4.1 程序修改后的重新同步与重新编译

当你在开发机上修改了 `Line_follow` 目录中的程序后，建议按下面步骤重新同步到 RDK X5。

在开发机执行增量同步：

```bash
bash /home/yunbo/rdk_backup/dev_ws/src/originbot/Line_follow/tools/sync_to_board.sh
```

说明：

- `--delete` 会删除 RDK 端目标目录中已经不存在的旧文件，避免残留旧版本脚本
- 同步脚本会继续排除本地编辑器目录，同时不会把本地 `build/`、`install/`、`log/` 等编译产物同步到 RDK

同步完成后，登录 RDK：

```bash
ssh root@192.168.127.10
cd /userdata/dev_ws
```

重新编译当前包：

```bash
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j1
colcon build --executor sequential --parallel-workers 1 --packages-select line_follow
```

编译完成后重新加载环境：

```bash
source /userdata/dev_ws/install/setup.bash
```

如果只是修改了 launch、Python 源码或文档，仍然建议重新执行一次 `colcon build --packages-select line_follow`，这样安装目录中的脚本会同步到最新版本。

## 4.2 当前整系统 launch 已暴露的关键参数

当前 `line_follow_system.launch.py` 已与三个节点的常用部署参数对齐，现场可直接通过 `ros2 launch` 覆盖：

- 视觉节点：
  - `device`
  - `image_topic`
  - `line_is_white`
  - `blur_ksize`
  - `thresh`
  - `morph_ksize`
  - `n_windows`
  - `margin`
  - `minpix`
  - `kp`
  - `roi_y_start_ratio`
  - `min_component_area`
  - `max_component_area`
  - `max_component_width_px`
  - `max_component_width_ratio`
  - `component_intensity_limit`
  - `show_debug`
  - `publish_debug_image`
  - `show_angle_curve`
  - `angle_curve_history_size`
  - `angle_curve_limit_deg`

当前默认配置中，`margin` 已调整为 `300`，用于扩大滑动窗口的横向搜索范围。
- 运动模型节点：
  - `track_width_m`
  - `max_visual_angle_deg`
  - `heading_gain`
  - `base_speed_ratio`
  - `speed_reduce_gain`
  - `min_output_rpm`
  - `allow_reverse`
  - `command_timeout_sec`
  - `drive_wheel_diameter_m`
  - `track_pitch_m`
  - `track_link_count`
- 驱动控制节点：
  - `serial_port`
  - `baud_rate`
  - `serial_timeout_sec`
  - `left_slave`
  - `right_slave`
  - `min_speed_rpm`
  - `max_speed_rpm`
  - `auto_start`

说明：

- 这份 launch 主要覆盖部署和联调时最常改的参数
- 节点内部仍保留少量高级参数默认值，例如调试话题名、额定电机参数等；如果后续需要，也可以继续在 launch 中补充

## 5. 分阶段验证

验证时务必先将履带架空，避免参数未调好时直接冲车。

### 5.1 验证摄像头链路

先只启动摄像头与 web：

```bash
ros2 launch line_follow usb_cam_web.launch.py device:=/dev/video8
```

启动后可直接访问 Web 调试页：

- 开发板本机：`http://127.0.0.1:8091/`
- 局域网其他设备：`http://<BOARD_IP>:8091/`
- 检测图直连：`http://<BOARD_IP>:8091/stream/detect.mjpg`
- 原始相机图直连：`http://<BOARD_IP>:8091/stream/source.mjpg`

另开一个终端检查话题：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 topic list | grep -E '^/image$|^/hbmem_img$'
ros2 topic hz /image
```

预期结果：

- `/image` 存在，这是 `hobot_usb_cam` 发布的 `sensor_msgs/msg/CompressedImage`
- `/hbmem_img` 存在，这是 `hobot_codec_decoder` 解码后发布的共享内存图像
- `/image` 图像频率正常
- Web 页面能看到实时画面

说明：

- Web 显示链路使用 `/image`
- `line_follow_angle_node` 当前默认订阅 `/hbmem_img`

如果 `/image` 或 `/hbmem_img` 不存在，先排查相机设备号、`hobot_codec_decoder` 和 `hobot_shm` 是否正常启动。

### 5.2 验证视觉角度节点

启动角度检测节点：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 run line_follow line_follow_angle_node \
  --ros-args \
  -p image_topic:=/hbmem_img \
  -p image_msg_type:=hbmem \
  -p line_is_white:=false \
  -p min_component_area:=0 \
  -p max_component_area:=20000 \
  -p max_component_width_px:=100 \
  -p component_intensity_limit:=-1.0 \
  -p roi_y_start_ratio:=0.5 \
  -p show_debug:=false
```

另开终端观察输出：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 topic echo /line_follow/line_angle_deg
```

再检查调试图像话题是否存在：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 topic list | grep line_follow
```

预期结果：

- 相机看到引导线时，`/line_follow/line_angle_deg` 持续输出角度
- 线条右倾时角度为正，左倾时角度为负
- 默认会发布调试图像话题：
  - `/line_follow/debug_binary`
  - `/line_follow/debug_detect`
  - `/line_follow/debug_angle_curve`

如果需要直接查看检测结果图像，可执行：

```bash
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在 `rqt_image_view` 中选择：

- `/line_follow/debug_binary`
- `/line_follow/debug_detect`
- `/line_follow/debug_angle_curve`

说明：

- `debug_binary` 用于检查二值化是否正确
- `debug_binary` 也可用于观察连通域过滤后的最终保留对象
- `debug_detect` 用于检查滑动窗口、拟合直线和角度检测结果是否正确
- `debug_angle_curve` 用于远程查看最近一段时间的角度变化曲线
- 如果在纯 SSH 终端下没有桌面环境，程序会自动关闭 `show_debug:=true` 对应的 OpenCV 本地窗口，避免 Qt 插件初始化失败；调试图像话题仍然可以正常发布

如果需要远程查看角度曲线，可直接运行：

```bash
ros2 run line_follow line_follow_angle_node \
  --ros-args \
  -p image_topic:=/hbmem_img \
  -p image_msg_type:=hbmem \
  -p line_is_white:=false \
  -p show_debug:=false \
  -p publish_debug_image:=true \
  -p show_angle_curve:=true
```

如果需要关闭调试图像发布，可在启动时增加：

```bash
-p publish_debug_image:=false
```

如果没有输出：

- 调整 `line_is_white`
- 调整阈值 `thresh`
- 调整目标过滤参数 `min_component_area`、`max_component_area`、`max_component_width_px`、`max_component_width_ratio`、`component_intensity_limit`
- 调整窗口参数 `margin`、`minpix`
- 适当减小 `roi_y_start_ratio` 扩大底部直方图 ROI，例如 `0.5 -> 0.35`

### 5.3 验证运动模型节点

启动运动模型节点：

```bash
ros2 run line_follow line_follow_motor_model_node \
  --ros-args -p track_width_m:=0.577 -p heading_gain:=1.8 -p base_speed_ratio:=0.32 -p speed_reduce_gain:=0.55 -p allow_reverse:=true -p command_timeout_sec:=0.5
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
- 当角度输入中断超过 `command_timeout_sec` 后，模型节点会主动发布 `[0.0, 0.0]`

如果差速方向反了，有两种修正办法：

- 交换左右驱动站号
- 将 `heading_gain` 的符号逻辑改掉

### 5.4 验证驱动通信节点

接好 RS485/串口，确认驱动器地址与脚本参数一致，再启动：

```bash
ros2 run line_follow motor_driver_control_node \
  --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=9600 -p serial_timeout_sec:=0.1 -p left_slave:=6 -p right_slave:=8 -p min_speed_rpm:=0 -p max_speed_rpm:=5000 -p auto_start:=true
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
ros2 topic pub --once /motor_speed_cmd std_msgs/msg/Float32MultiArray "{data: [-300.0, 300.0]}"
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
  image_topic:=/hbmem_img \
  image_msg_type:=hbmem \
  serial_port:=/dev/ttyUSB0 \
  track_width_m:=0.577 \
  command_timeout_sec:=0.5 \
  allow_reverse:=true \
  left_slave:=6 \
  right_slave:=8 \
  show_debug:=false \
  publish_debug_image:=true \
  show_angle_curve:=true \
  line_is_white:=false \
  min_component_area:=200 \
  max_component_area:=20000 \
  max_component_width_px:=180 \
  component_intensity_limit:=90 \
  roi_y_start_ratio:=0.5
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

### 6.3 扩大视觉 ROI

视觉节点用于初始化滑动窗口的直方图统计区域，默认从图像高度的 `0.5` 位置开始向下统计，也就是使用下半幅图像作为 ROI。

如果线条离相机较远，或者底部区域经常看不到有效线条，可减小 `roi_y_start_ratio` 来扩大 ROI：

```bash
roi_y_start_ratio:=0.35
```

经验建议：

- `0.5`：默认值，抗干扰相对更好
- `0.35`：更容易提前捕捉远处线条，适合作为第一步放大
- `0.2`：ROI 很大，但更容易把阴影和杂散纹理一起纳入统计

调试时可查看 `/line_follow/debug_detect`，其中会画一条横线标出当前 ROI 起始位置。

### 6.4 限制对象大小、去掉过宽目标、保留更黑对象

当前视觉节点已经支持在二值化后增加一层连通域过滤。建议优先在本地调试脚本中把参数调通，再带到 RDK X5 上。

可调参数：

- `min_component_area`
- `max_component_area`
- `max_component_width_px`
- `max_component_width_ratio`
- `component_intensity_limit`

在哪里调整：

- 本地 OpenCV 调试：`tools/opencv_line_verify.py`
- ROS 单节点调试：`ros2 run line_follow line_follow_angle_node --ros-args -p ...`
- 整系统联调：`ros2 launch line_follow line_follow_system.launch.py ...`

达到什么效果时需要调整什么参数：

- 想去掉小噪点、小碎块：
  增大 `min_component_area`
- 想去掉大块阴影、大块污渍：
  减小 `max_component_area`
- 想去掉过于宽的对象：
  减小 `max_component_width_px` 或 `max_component_width_ratio`
- 想只保留更黑的对象：
  在黑线模式下减小 `component_intensity_limit`
- 正常目标线也被误删：
  适当放宽 `max_component_area`、`max_component_width_px`，或增大 `component_intensity_limit`

黑线白底场景示例：

```bash
ros2 run line_follow line_follow_angle_node \
  --ros-args \
  -p image_topic:=/hbmem_img \
  -p image_msg_type:=hbmem \
  -p line_is_white:=false \
  -p min_component_area:=200 \
  -p max_component_area:=20000 \
  -p max_component_width_px:=180 \
  -p component_intensity_limit:=90 \
  -p show_debug:=false \
  -p publish_debug_image:=true
```

建议观察：

- `/line_follow/debug_binary`
  用于确认宽黑块和浅灰污渍是否已经被过滤掉
- `/line_follow/debug_detect`
  用于确认滑动窗口是否还跟在线上

## 7. 常见故障排查

### 7.1 没有图像

- 检查 `/dev/video*`
- 检查 `device:=/dev/videoX`
- 检查相机权限

### 7.2 有图像但没有角度

- 调 `line_is_white`
- 调阈值和窗口参数
- 调目标过滤参数，避免目标被错误过滤或干扰物被错误保留
- 减小 `roi_y_start_ratio` 扩大 ROI，避免线条没有进入底部统计区域
- 有桌面环境时可用 `show_debug:=true` 观察二值化和滑窗结果
- 无桌面环境时优先查看 `/line_follow/debug_binary`、`/line_follow/debug_detect`、`/line_follow/debug_angle_curve`

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
