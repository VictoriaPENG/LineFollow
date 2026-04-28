#!/usr/bin/env python3
"""长按遥控节点的命令行启动包装器。"""

import os
import sys


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    # 兼容“源码树直接运行”和“安装后运行”两种场景。
    sys.path.insert(0, REPO_ROOT)

try:
    from line_follow.remote_long_press_start_line_follow_node import main
except ModuleNotFoundError:
    from src.remote_long_press_start_line_follow_node import main


if __name__ == "__main__":
    raise SystemExit(main())
