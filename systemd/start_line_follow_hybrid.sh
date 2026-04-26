#!/usr/bin/env bash
set -e

cd /userdata/dev_ws
source /opt/ros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash

exec ros2 launch line_follow joystick_remote_hybrid.launch.py \
  device:=/dev/video0 \
  enable_websocket:=true \
  enable_dashboard:=true
