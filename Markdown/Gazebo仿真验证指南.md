# Gazebo 仿真验证指南

本文用于验证 `line_follow` 的 Gazebo Classic 初版仿真链路：模型能正常加载，且可以通过 `/cmd_vel` 在仿真场景中运动。

## 1. 环境要求

建议在 Ubuntu + ROS 2 Humble 环境执行：

```bash
source /opt/ros/humble/setup.bash
```

需要具备这些 ROS/Gazebo 包：

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-msgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher
```

## 2. 编译

在工作空间根目录执行：

```bash
colcon build --packages-select line_follow
source install/setup.bash
```

确认仿真入口已经安装：

```bash
ros2 pkg executables line_follow | grep sim
```

正常应看到：

```text
line_follow sim_cmd_vel_controller_node
line_follow sim_motion_smoke_test_node
```

## 3. 启动 Gazebo 模型

```bash
ros2 launch line_follow gazebo_sim.launch.py
```

启动后应看到 Gazebo Classic 窗口、地面、太阳光和 `line_follow_car` 模型。模型由三部分组成：

- 简化蓝色车体和黑色履带，用于保证即使 CAD mesh 未加载也能看到车。
- `meshes/chassis_visual.dae`，用于显示更接近真实车体的外观。
- 简化盒状 collision 和惯性参数，用于稳定仿真。

## 4. 手动发速度验证运动

另开一个终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

持续发布前进加转向命令：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.35}, angular: {z: 0.45}}"
```

车体应在 Gazebo 里向前走并缓慢转向。停止命令：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## 5. 一键冒烟测试

如果只想确认“模型确实会动”，保持 Gazebo 启动，另开终端运行：

```bash
ros2 run line_follow sim_motion_smoke_test_node
```

该节点会：

1. 通过 Gazebo `GetEntityState` 读取 `line_follow_car` 初始位姿。
2. 向 `/cmd_vel` 发布 3 秒速度命令。
3. 再次读取位姿。
4. 判断平移量是否超过 `0.20 m`，偏航变化是否超过 `0.20 rad`。

通过时日志会显示：

```text
Gazebo motion smoke test passed
```

如果只想验证直行，可以覆盖参数：

```bash
ros2 run line_follow sim_motion_smoke_test_node --ros-args \
  -p angular_z:=0.0 \
  -p min_yaw_change_rad:=0.0
```

## 6. 常见问题

### Gazebo 打开但模型没出现

检查 spawn 服务和包资源是否正常：

```bash
ros2 service list | grep spawn
ros2 pkg prefix line_follow
ls install/line_follow/share/line_follow/urdf
ls install/line_follow/share/line_follow/meshes
```

如果没有 `urdf`、`meshes`、`worlds`，重新编译并确认已 `source install/setup.bash`。

### 模型出现但不运动

确认控制服务和话题：

```bash
ros2 service list | grep entity_state
ros2 topic echo /cmd_vel
ros2 node list | grep sim_cmd_vel_controller
```

`sim_cmd_vel_controller_node` 需要看到 Gazebo 的 `SetEntityState` 服务。启动文件会加载 `libgazebo_ros_state.so`，正常服务名通常是 `/set_entity_state` 或 `/gazebo/set_entity_state`。

### 日志提示 SetEntityState 服务不可用

先确认 Gazebo ROS 插件是否安装：

```bash
ros2 pkg prefix gazebo_ros
```

如果包不存在，安装：

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

### DAE mesh 不显示但蓝色简化车体显示

这说明模型主体是正常的，只是 CAD mesh 资源未加载。检查：

```bash
ls install/line_follow/share/line_follow/meshes/chassis_visual.dae
```

如果文件不存在，重新编译。如果文件存在但仍不显示，先以简化车体继续验证运动；CAD 外观不影响 `/cmd_vel` 运动验证。

## 7. 当前仿真边界

当前 Gazebo 仿真是第一阶段模型验证链路：`/cmd_vel` 通过 `SetEntityState` 直接改变车体位姿。它不是完整履带动力学模型，也还没有相机、赛道线和视觉闭环。下一阶段可以继续加入：

- Gazebo 相机插件，发布 `/image`。
- 带黑色巡线的 world。
- RPM 到 `/cmd_vel` 的桥接，接入现有 `line_follow_motor_model_node`。
- 更真实的履带/差速物理模型。
