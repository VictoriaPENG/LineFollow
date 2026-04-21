#!/usr/bin/env bash
set -euo pipefail

OUTDIR=${1:-runtime_params_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUTDIR"

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
