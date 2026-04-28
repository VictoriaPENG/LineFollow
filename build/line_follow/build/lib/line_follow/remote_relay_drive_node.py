#!/usr/bin/env python3
"""
继电器遥控直驱节点。

默认行为：
1. 物理 pin 29 收到稳定继电器信号 -> 前进
2. 物理 pin 31 收到稳定继电器信号 -> 后退
3. 没有信号或两个方向同时有效 -> 停车

本节点直接发布 `/motor_speed_cmd`，因此默认要求
`motor_driver_control_node` 已经在运行。
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
from typing import Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from line_follow.gpio_compat import load_gpio_module
from line_follow.gpio_input_filter import DebouncedDigitalInput
from line_follow.runtime_capture import RuntimeCaptureManager

GPIO = load_gpio_module()

FIXED_ACTIVE_LOW = True
FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS = False


class RelayRemoteDriveNode(Node):
    """读取继电器输入并直接发布电机速度命令。"""

    def __init__(self) -> None:
        super().__init__("relay_remote_drive_node")

        self.declare_parameter("speed_topic", "/motor_speed_cmd")
        self.declare_parameter("forward_pin", 29)
        self.declare_parameter("reverse_pin", 31)
        self.declare_parameter("use_board_numbering", True)
        self.declare_parameter("debug_inputs_only", False)
        self.declare_parameter("debounce_activate_count", 5)
        self.declare_parameter("debounce_deactivate_count", 1)
        self.declare_parameter("forward_left_rpm", -900.0)
        self.declare_parameter("forward_right_rpm", 900.0)
        self.declare_parameter("reverse_left_rpm", 900.0)
        self.declare_parameter("reverse_right_rpm", -900.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("autostart_driver_process", True)
        self.declare_parameter(
            "driver_launch_command",
            "ros2 run line_follow motor_driver_control_node",
        )
        self.declare_parameter("auto_record_runtime_data", True)
        self.declare_parameter("record_root_dir", "/userdata/test_logs")
        self.declare_parameter("record_rosbag", True)
        self.declare_parameter(
            "save_params_script",
            "/userdata/dev_ws/src/originbot/Line_follow/board_tools/save_runtime_params.sh",
        )

        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.forward_pin = int(self.get_parameter("forward_pin").value)
        self.reverse_pin = int(self.get_parameter("reverse_pin").value)
        self.use_board_numbering = bool(self.get_parameter("use_board_numbering").value)
        self.active_low = FIXED_ACTIVE_LOW
        self.allow_inputs_without_pull_resistors = FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS
        self.debug_inputs_only = bool(self.get_parameter("debug_inputs_only").value)
        debounce_activate_count = int(self.get_parameter("debounce_activate_count").value)
        debounce_deactivate_count = int(self.get_parameter("debounce_deactivate_count").value)
        self.forward_cmd = (
            float(self.get_parameter("forward_left_rpm").value),
            float(self.get_parameter("forward_right_rpm").value),
        )
        self.reverse_cmd = (
            float(self.get_parameter("reverse_left_rpm").value),
            float(self.get_parameter("reverse_right_rpm").value),
        )
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.autostart_driver_process = bool(
            self.get_parameter("autostart_driver_process").value
        )
        self.driver_launch_command = str(self.get_parameter("driver_launch_command").value)
        self.capture = RuntimeCaptureManager(
            enabled=bool(self.get_parameter("auto_record_runtime_data").value),
            record_root_dir=str(self.get_parameter("record_root_dir").value),
            session_prefix="remote_manual",
            record_rosbag=bool(self.get_parameter("record_rosbag").value),
            rosbag_topics=[
                "/hbmem_img",
                "/image",
                "/line_follow/debug_binary",
                "/line_follow/debug_detect",
                "/line_follow/debug_angle_curve",
                "/line_follow/line_angle_deg",
                "/line_follow/line_offset_norm",
                "/line_follow/line_offset_px",
                "/motor_speed_cmd",
            ],
            save_params_script=str(self.get_parameter("save_params_script").value),
        )

        self.publisher = self.create_publisher(Float32MultiArray, self.speed_topic, 10)
        self.last_state = "stop"
        self.last_cmd = (0.0, 0.0)
        self.last_debug_snapshot = None
        self.forward_filter = DebouncedDigitalInput(
            active_low=self.active_low,
            activate_count=debounce_activate_count,
            deactivate_count=debounce_deactivate_count,
        )
        self.reverse_filter = DebouncedDigitalInput(
            active_low=self.active_low,
            activate_count=debounce_activate_count,
            deactivate_count=debounce_deactivate_count,
        )
        self._driver_child = None
        self._shutting_down = False

        self._setup_gpio()
        run_dir = self.capture.start()
        if self.autostart_driver_process:
            self._start_driver_process()
        self.timer = self.create_timer(1.0 / publish_hz, self._poll_inputs)

        self.get_logger().info(
            f"relay remote drive ready: speed_topic={self.speed_topic}, "
            f"mode={'BOARD' if self.use_board_numbering else 'BCM'}, "
            f"forward_pin={self.forward_pin}, reverse_pin={self.reverse_pin}, "
            f"active_low={self.active_low}, debug_inputs_only={self.debug_inputs_only}, "
            f"debounce_activate_count={debounce_activate_count}, "
            f"debounce_deactivate_count={debounce_deactivate_count}, "
            f"forward_cmd={self.forward_cmd}, reverse_cmd={self.reverse_cmd}, "
            f"autostart_driver_process={self.autostart_driver_process}, "
            f"driver_launch_command={self.driver_launch_command}"
        )
        if run_dir:
            self.get_logger().info(f"runtime capture enabled: run_dir={run_dir}")
        self._publish_command((0.0, 0.0))

    def _setup_gpio(self) -> None:
        mode = GPIO.BOARD if self.use_board_numbering else GPIO.BCM
        GPIO.setwarnings(False)
        GPIO.setmode(mode)

        if hasattr(GPIO, "PUD_UP") and hasattr(GPIO, "PUD_DOWN"):
            pud = GPIO.PUD_UP if self.active_low else GPIO.PUD_DOWN
            GPIO.setup(self.forward_pin, GPIO.IN, pull_up_down=pud)
            GPIO.setup(self.reverse_pin, GPIO.IN, pull_up_down=pud)
        else:
            if not self.allow_inputs_without_pull_resistors:
                raise RuntimeError(
                    "GPIO module does not expose PUD_UP/PUD_DOWN. "
                    "Refusing to start because floating relay inputs can trigger unintended motion. "
                    "Add external pull-up/pull-down resistors or update the fixed GPIO configuration in this script."
                )

            self.get_logger().warn(
                "GPIO module does not expose PUD_UP/PUD_DOWN; proceeding without internal pull resistors because "
                "'allow_inputs_without_pull_resistors' is enabled"
            )
            GPIO.setup(self.forward_pin, GPIO.IN)
            GPIO.setup(self.reverse_pin, GPIO.IN)

    def _read_pin(self, pin: int, filter_state: DebouncedDigitalInput) -> Tuple[int, bool, bool]:
        raw_level = int(GPIO.input(pin))
        raw_active, filtered_active, _ = filter_state.update(raw_level)
        return raw_level, raw_active, filtered_active

    def _start_driver_process(self) -> None:
        if self._shutting_down:
            return
        if self._driver_child is not None and self._driver_child.poll() is None:
            return

        self.get_logger().info(f"starting motor driver process: {self.driver_launch_command}")
        self.capture.log_event(f"starting motor driver process: {self.driver_launch_command}")
        driver_log = self.capture.open_log_file("motor_driver.log")
        self._driver_child = subprocess.Popen(
            ["bash", "-lc", self.driver_launch_command],
            start_new_session=True,
            stdout=driver_log if driver_log is not None else subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )

    def _ensure_driver_process(self) -> bool:
        if self._shutting_down:
            return False
        if not self.autostart_driver_process:
            return True
        if self._driver_child is not None and self._driver_child.poll() is None:
            return True
        self._start_driver_process()
        return self._driver_child is not None and self._driver_child.poll() is None

    def _check_driver_process(self) -> None:
        if self._driver_child is None:
            return

        exit_code = self._driver_child.poll()
        if exit_code is None:
            return

        self.get_logger().error(f"motor driver process exited with code {exit_code}")
        self.capture.log_event(f"motor driver process exited with code {exit_code}")
        self._driver_child = None

    def _stop_process(self, child, label: str) -> None:
        if child is None or child.poll() is not None:
            return

        try:
            self.get_logger().info(f"stopping {label}")
            self.capture.log_event(f"stopping {label}")
            pgid = os.getpgid(child.pid)
            os.killpg(pgid, signal.SIGINT)
            child.wait(timeout=5.0)
        except Exception:
            try:
                pgid = os.getpgid(child.pid)
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                pass

    def _stop_driver_process(self) -> None:
        if self._driver_child is None:
            return
        self._stop_process(self._driver_child, "motor driver process")
        self._driver_child = None

    def _poll_inputs(self) -> None:
        self._check_driver_process()
        if not self._ensure_driver_process():
            self.get_logger().error("motor driver process is not running, forcing stop output")
            self._publish_command((0.0, 0.0))
            return

        forward_raw, forward_raw_active, forward_active = self._read_pin(self.forward_pin, self.forward_filter)
        reverse_raw, reverse_raw_active, reverse_active = self._read_pin(self.reverse_pin, self.reverse_filter)

        if self.debug_inputs_only:
            snapshot = (
                forward_raw,
                forward_raw_active,
                forward_active,
                reverse_raw,
                reverse_raw_active,
                reverse_active,
            )
            if snapshot != self.last_debug_snapshot:
                self.get_logger().info(
                    f"relay inputs: forward_raw={forward_raw}, forward_raw_active={forward_raw_active}, "
                    f"forward_active={forward_active}, reverse_raw={reverse_raw}, "
                    f"reverse_raw_active={reverse_raw_active}, reverse_active={reverse_active}"
                )
                self.capture.log_event(
                    f"relay inputs: forward_raw={forward_raw}, forward_raw_active={forward_raw_active}, "
                    f"forward_active={forward_active}, reverse_raw={reverse_raw}, "
                    f"reverse_raw_active={reverse_raw_active}, reverse_active={reverse_active}"
                )
                self.last_debug_snapshot = snapshot
            self.last_state = "debug"
            self._publish_command((0.0, 0.0))
            return

        if forward_active and not reverse_active:
            state = "forward"
            cmd = self.forward_cmd
        elif reverse_active and not forward_active:
            state = "reverse"
            cmd = self.reverse_cmd
        else:
            state = "stop"
            cmd = (0.0, 0.0)

        if state != self.last_state:
            self.get_logger().info(
                f"relay state changed: forward={forward_active}, reverse={reverse_active}, action={state}"
            )
            self.capture.log_event(
                f"relay state changed: forward={forward_active}, reverse={reverse_active}, action={state}"
            )
            self.last_state = state

        self._publish_command(cmd)

    def _publish_command(self, cmd: Tuple[float, float]) -> None:
        msg = Float32MultiArray()
        msg.data = [float(cmd[0]), float(cmd[1])]
        self.publisher.publish(msg)
        self.last_cmd = cmd

    def destroy_node(self) -> bool:
        self._shutting_down = True
        try:
            self._publish_command((0.0, 0.0))
        except Exception:
            pass
        try:
            self._stop_driver_process()
        except Exception:
            pass
        try:
            self.capture.stop()
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass
        return super().destroy_node()


def main() -> int:
    try:
        rclpy.init()
        node = RelayRemoteDriveNode()
    except Exception as exc:
        print(f"[relay_remote_drive] startup failed: {exc}", file=sys.stderr)
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
