#!/usr/bin/env bash

# 将当前 USB 相机曝光时间按比例缩短。
# 典型用途是在现场画面过曝时快速压低曝光，不需要手工先读再算。

set -euo pipefail

DEVICE="${1:-/dev/video0}"
SCALE="${2:-0.5}"

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "v4l2-ctl not found, skip camera exposure adjustment" >&2
  exit 0
fi

if [[ ! -e "${DEVICE}" ]]; then
  echo "camera device not found: ${DEVICE}" >&2
  exit 0
fi

CURRENT="$(v4l2-ctl -d "${DEVICE}" -C exposure_absolute 2>/dev/null | sed -n 's/^exposure_absolute: //p' | tr -d '[:space:]')"
if [[ -z "${CURRENT}" ]]; then
  echo "camera does not expose exposure_absolute on ${DEVICE}" >&2
  exit 0
fi

TARGET="$(awk -v current="${CURRENT}" -v scale="${SCALE}" 'BEGIN {
  value = int(current * scale)
  if (value < 1) value = 1
  print value
}')"

# 先切到手动曝光模式，再写目标曝光值。
v4l2-ctl -d "${DEVICE}" -c exposure_auto=1 >/dev/null 2>&1 || true
v4l2-ctl -d "${DEVICE}" -c exposure_absolute="${TARGET}" >/dev/null 2>&1 || {
  echo "failed to set exposure_absolute=${TARGET} on ${DEVICE}" >&2
  exit 0
}

echo "camera exposure adjusted on ${DEVICE}: ${CURRENT} -> ${TARGET}" >&2
