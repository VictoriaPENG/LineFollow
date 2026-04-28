#!/usr/bin/env python3
"""板端运行时数据留存工具。

用于在每次启动任务时自动创建独立运行目录，记录：
- 参数快照
- 事件日志
- 可选 rosbag

这样在现场调试时，即使程序退出，也能回看当次运行的关键上下文。
"""

from __future__ import annotations

import datetime as _dt
import os
import signal
import subprocess
from typing import Iterable, Optional


class RuntimeCaptureManager:
    """创建单次运行目录、保存快照并管理 rosbag 录制。"""

    def __init__(
        self,
        *,
        enabled: bool,
        record_root_dir: str,
        session_prefix: str,
        record_rosbag: bool,
        rosbag_topics: Iterable[str],
        save_params_script: str,
    ) -> None:
        self.enabled = bool(enabled)
        self.record_root_dir = str(record_root_dir)
        self.session_prefix = str(session_prefix)
        self.record_rosbag = bool(record_rosbag)
        self.rosbag_topics = [str(topic) for topic in rosbag_topics if str(topic).strip()]
        self.save_params_script = str(save_params_script)
        self.run_dir: Optional[str] = None
        self.event_log_path: Optional[str] = None
        self.rosbag_child: Optional[subprocess.Popen] = None
        self.rosbag_log_path: Optional[str] = None

    def start(self) -> Optional[str]:
        """启动一次新的运行记录会话。"""
        if not self.enabled:
            return None

        stamp = _dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        run_name = f"{self.session_prefix}_{stamp}" if self.session_prefix else stamp
        self.run_dir = os.path.join(self.record_root_dir, run_name)
        os.makedirs(self.run_dir, exist_ok=True)
        self.event_log_path = os.path.join(self.run_dir, "session.log")
        self.rosbag_log_path = os.path.join(self.run_dir, "rosbag_record.log")
        self.log_event(f"runtime capture started: run_dir={self.run_dir}")
        self.save_params("params_start")
        if self.record_rosbag and self.rosbag_topics:
            self._start_rosbag()
        return self.run_dir

    def log_event(self, message: str) -> None:
        if not self.enabled or not self.event_log_path:
            return
        stamp = _dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.event_log_path, "a", encoding="utf-8") as fp:
            fp.write(f"[{stamp}] {message}\n")

    def open_log_file(self, filename: str):
        if not self.enabled or not self.run_dir:
            return None
        path = os.path.join(self.run_dir, filename)
        return open(path, "ab")

    def save_params(self, name: str) -> None:
        if not self.enabled or not self.run_dir or not self.save_params_script:
            return

        outdir = os.path.join(self.run_dir, name)
        try:
            subprocess.run(
                ["bash", self.save_params_script, outdir],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.log_event(f"saved runtime params: {outdir}")
        except Exception as exc:
            self.log_event(f"save runtime params failed: {exc}")

    def stop(self) -> None:
        if not self.enabled:
            return
        self.save_params("params_final")
        self._stop_rosbag()
        self.log_event("runtime capture stopped")

    def _start_rosbag(self) -> None:
        if self.run_dir is None or self.rosbag_child is not None:
            return

        bag_dir = os.path.join(self.run_dir, "rosbag")
        os.makedirs(bag_dir, exist_ok=True)
        command = [
            "bash",
            "-lc",
            "cd "
            + _shell_quote(self.run_dir)
            + " && ros2 bag record -o rosbag "
            + " ".join(_shell_quote(topic) for topic in self.rosbag_topics),
        ]
        log_fp = self.open_log_file("rosbag_record.log")
        self.rosbag_child = subprocess.Popen(
            command,
            start_new_session=True,
            stdout=log_fp if log_fp is not None else subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        self.log_event("started rosbag recording")

    def _stop_rosbag(self) -> None:
        child = self.rosbag_child
        if child is None:
            return
        self.rosbag_child = None
        _stop_process(child)
        self.log_event("stopped rosbag recording")


def _stop_process(child: subprocess.Popen) -> None:
    if child.poll() is not None:
        return
    try:
        pgid = os.getpgid(child.pid)
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return
    except Exception:
        return

    if _wait_or_signal(child, 2.0, signal.SIGTERM):
        return
    if _wait_or_signal(child, 2.0, signal.SIGKILL):
        return
    try:
        child.wait(timeout=1.0)
    except Exception:
        pass


def _wait_or_signal(child: subprocess.Popen, timeout_sec: float, sig) -> bool:
    try:
        child.wait(timeout=timeout_sec)
        return True
    except subprocess.TimeoutExpired:
        try:
            pgid = os.getpgid(child.pid)
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True
        except Exception:
            return False
        return False
    except Exception:
        return child.poll() is not None


def _shell_quote(text: str) -> str:
    return "'" + str(text).replace("'", "'\"'\"'") + "'"
