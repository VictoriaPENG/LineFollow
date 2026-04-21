#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

REMOTE_HOST="${REMOTE_HOST:-root@192.168.127.10}"
REMOTE_DIR="${REMOTE_DIR:-/userdata/dev_ws/src/originbot/Line_follow/}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage:
  tools/sync_to_board.sh [remote_host] [remote_dir]

Defaults:
  remote_host = root@192.168.127.10
  remote_dir  = /userdata/dev_ws/src/originbot/Line_follow/

Environment overrides:
  REMOTE_HOST
  REMOTE_DIR
EOF
  exit 0
fi

if [[ $# -ge 1 ]]; then
  REMOTE_HOST="$1"
fi

if [[ $# -ge 2 ]]; then
  REMOTE_DIR="$2"
fi

if [[ $# -gt 2 ]]; then
  echo "too many arguments" >&2
  exit 2
fi

rsync -av --delete \
  --exclude '.vscode/' \
  --exclude '.idea/' \
  --exclude '.git/' \
  --exclude '.codex/' \
  --exclude 'Markdown/' \
  --exclude 'tools/' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '*.pyo' \
  --exclude '*.swp' \
  --exclude '*.swo' \
  "${PROJECT_DIR}/" \
  "${REMOTE_HOST}:${REMOTE_DIR}"
