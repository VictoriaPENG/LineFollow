#!/usr/bin/env python3
"""摇杆驱动节点的命令行启动包装器。

优先从已安装包导入；如果当前直接在源码树中运行，则回退到 `src/`
目录导入。这样开发环境和安装环境都能共用同一个入口脚本。
"""

import os
import sys


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    # 允许在源码目录里直接运行，而不要求先安装成 Python 包。
    sys.path.insert(0, REPO_ROOT)

try:
    from line_follow.joystick_drive_node import main
except ModuleNotFoundError:
    from src.joystick_drive_node import main


if __name__ == "__main__":
    raise SystemExit(main())
