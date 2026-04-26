# systemd 目录说明

这个目录集中存放当前程序推荐使用的 `systemd` 服务模板。

当前只推荐保留一套常驻服务：

- `line-follow-hybrid.service`

它对应的启动入口是：

```bash
ros2 launch line_follow joystick_remote_hybrid.launch.py \
  device:=/dev/video0 \
  enable_websocket:=true \
  enable_dashboard:=true
```

这套入口会统一启动：

- 相机链路
- `/image -> /hbmem_img` 解码和共享内存链路
- 巡线核心节点
- 电机驱动节点
- 长按遥控节点
- 摇杆节点
- Web 页面和 dashboard

## 目录内容

- `start_line_follow_hybrid.sh`
  板端启动脚本模板
- `line-follow-hybrid.service`
  `systemd` 服务模板

## 使用方法

板端执行：

```bash
cp /userdata/dev_ws/src/originbot/Line_follow/systemd/start_line_follow_hybrid.sh /userdata/dev_ws/start_line_follow_hybrid.sh
chmod +x /userdata/dev_ws/start_line_follow_hybrid.sh

cp /userdata/dev_ws/src/originbot/Line_follow/systemd/line-follow-hybrid.service /etc/systemd/system/line-follow-hybrid.service

systemctl daemon-reload
systemctl disable --now line-follow-remote.service || true
systemctl disable --now line-follow-stack.service || true
systemctl disable --now line-follow-web.service || true
systemctl enable line-follow-hybrid.service
systemctl start line-follow-hybrid.service
```

## 注意事项

- 不要再并行运行 `line-follow-remote.service`
- 不要再并行运行 `line-follow-stack.service`
- 不要再并行运行 `line-follow-web.service`

否则最常见的问题是：

- 出现两个 `/line_follow_motor_model_node`
- 出现两套驱动节点
- 相机链路重复启动
- 串口和节点名冲突

## 可选调整

如果你不想开机自动带 Web，把启动脚本里的：

```bash
enable_websocket:=true
enable_dashboard:=true
```

改成：

```bash
enable_websocket:=false
enable_dashboard:=false
```
