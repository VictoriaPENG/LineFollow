#!/usr/bin/env bash

set -euo pipefail

DEVICE="${1:-/dev/video0}"
BRIGHTNESS="${2:-15}"

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "v4l2-ctl not found, skip camera brightness adjustment" >&2
  exit 0
fi

if [[ ! -e "${DEVICE}" ]]; then
  echo "camera device not found: ${DEVICE}" >&2
  exit 0
fi

v4l2-ctl -d "${DEVICE}" -c brightness="${BRIGHTNESS}" >/dev/null 2>&1 || {
  echo "failed to set brightness=${BRIGHTNESS} on ${DEVICE}" >&2
  exit 0
}

echo "camera brightness set on ${DEVICE}: ${BRIGHTNESS}" >&2
