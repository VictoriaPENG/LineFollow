#!/usr/bin/env python3

import os
import sys


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

try:
    from line_follow.remote_long_press_start_line_follow_node import main
except ModuleNotFoundError:
    from src.remote_long_press_start_line_follow_node import main


if __name__ == "__main__":
    raise SystemExit(main())
