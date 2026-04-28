#!/usr/bin/env bash
# 保存当前关键 ROS2 节点的运行时参数快照。
# 这样在现场调参后，可以把“实际生效参数”直接落盘保存。
set -euo pipefail

OUTDIR=${1:-runtime_params_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUTDIR"

# 逐个尝试导出关键节点参数；有些环境里节点名可能带 `/` 前缀，
# 因此这里兼容两种写法。
for node in line_follow_angle_node line_follow_motor_model_node motor_driver_control_node; do
  if ros2 param dump "/$node" > "$OUTDIR/$node.yaml" 2>/dev/null; then
    echo "saved $node -> $OUTDIR/$node.yaml"
    continue
  fi

  if ros2 param dump "$node" > "$OUTDIR/$node.yaml" 2>/dev/null; then
    echo "saved $node -> $OUTDIR/$node.yaml"
    continue
  fi

  rm -f "$OUTDIR/$node.yaml"
  echo "skip $node: node not available"
done

echo "runtime parameter snapshot saved in $OUTDIR"
