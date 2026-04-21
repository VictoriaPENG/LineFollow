#!/usr/bin/env python3
"""
Long-press remote mode controller for line-follow and manual forward drive.

Default behavior:
- Monitor the forward and reverse relay inputs on the 40-pin header
- Start the full line-follow stack in the background during node startup
- Long-press reverse to enable line-follow mode while the button is held
- Long-press forward to disable line-follow and enter manual forward mode while the button is held
- Releasing either button stops the vehicle immediately
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time

from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray

from line_follow.runtime_capture import RuntimeCaptureManager


def _load_gpio_module():
    """Try common GPIO modules used on embedded Linux boards."""
    module_errors = []
    for module_name in ("Hobot.GPIO", "Jetson.GPIO", "RPi.GPIO"):
        try:
            module = __import__(module_name, fromlist=["GPIO"])
            return module
        except Exception as exc:  # pragma: no cover - depends on board environment
            module_errors.append(f"{module_name}: {exc}")

    raise RuntimeError(
        "No supported GPIO Python module found. Tried: "
        + "; ".join(module_errors)
    )


GPIO = _load_gpio_module()

FIXED_ACTIVE_LOW = True
FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS = True


class RemoteLongPressStartNode(Node):
    """Arbitrate long-press remote modes without command-topic conflicts."""

    def __init__(self) -> None:
        super().__init__("remote_long_press_start_line_follow_node")

        self.declare_parameter("forward_pin", 32)
        self.declare_parameter("reverse_pin", 33)
        self.declare_parameter("use_board_numbering", True)
        self.declare_parameter("hold_seconds", 1.5)
        self.declare_parameter("poll_hz", 20.0)
        self.declare_parameter("autostart_line_follow_stack", True)
        self.declare_parameter(
            "allow_inputs_without_pull_resistors",
            FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS,
        )
        self.declare_parameter(
            "launch_command",
            "ros2 launch line_follow line_follow_system.launch.py line_follow_enabled:=false",
        )
        self.declare_parameter("speed_topic", "/motor_speed_cmd")
        self.declare_parameter("forward_left_rpm", -300.0)
        self.declare_parameter("forward_right_rpm", 300.0)
        self.declare_parameter("line_follow_node_name", "/line_follow_motor_model_node")
        self.declare_parameter("line_follow_enable_timeout_sec", 10.0)
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

        self.forward_pin = int(self.get_parameter("forward_pin").value)
        self.reverse_pin = int(self.get_parameter("reverse_pin").value)
        self.use_board_numbering = bool(self.get_parameter("use_board_numbering").value)
        self.active_low = FIXED_ACTIVE_LOW
        self.allow_inputs_without_pull_resistors = bool(
            self.get_parameter("allow_inputs_without_pull_resistors").value
        )
        self.hold_seconds = max(0.2, float(self.get_parameter("hold_seconds").value))
        self.autostart_line_follow_stack = bool(
            self.get_parameter("autostart_line_follow_stack").value
        )
        self.launch_command = str(self.get_parameter("launch_command").value)
        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.forward_cmd = (
            float(self.get_parameter("forward_left_rpm").value),
            float(self.get_parameter("forward_right_rpm").value),
        )
        self.line_follow_node_name = str(self.get_parameter("line_follow_node_name").value)
        self.line_follow_enable_timeout_sec = max(
            0.2, float(self.get_parameter("line_follow_enable_timeout_sec").value)
        )
        self.autostart_driver_process = bool(
            self.get_parameter("autostart_driver_process").value
        )
        self.driver_launch_command = str(self.get_parameter("driver_launch_command").value)
        self.capture = RuntimeCaptureManager(
            enabled=bool(self.get_parameter("auto_record_runtime_data").value),
            record_root_dir=str(self.get_parameter("record_root_dir").value),
            session_prefix="remote_long_press",
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
        poll_hz = max(2.0, float(self.get_parameter("poll_hz").value))

        self._press_started_at = None
        self._hold_candidate = None
        self._active_mode = "idle"
        self._line_follow_child = None
        self._driver_child = None
        self._line_follow_enabled = False
        self._shutting_down = False
        self.publisher = self.create_publisher(Float32MultiArray, self.speed_topic, 10)
        self.line_follow_param_client = self.create_client(
            SetParameters,
            f"{self.line_follow_node_name.rstrip('/')}/set_parameters",
        )

        self._setup_gpio()
        run_dir = self.capture.start()
        if self._should_manage_driver_process():
            self._start_driver_process()
        if self.autostart_line_follow_stack:
            self._start_line_follow_process()
        self.timer = self.create_timer(1.0 / poll_hz, self._poll_trigger)
        self._publish_command((0.0, 0.0))

        self.get_logger().info(
            f"long-press trigger ready: forward_pin={self.forward_pin}, reverse_pin={self.reverse_pin}, "
            f"mode={'BOARD' if self.use_board_numbering else 'BCM'}, "
            f"active_low={self.active_low}, hold_seconds={self.hold_seconds:.2f}, "
            f"autostart_line_follow_stack={self.autostart_line_follow_stack}, "
            f"autostart_driver_process={self.autostart_driver_process}, "
            f"allow_inputs_without_pull_resistors={self.allow_inputs_without_pull_resistors}, "
            f"launch_command={self.launch_command}, line_follow_node_name={self.line_follow_node_name}, "
            f"driver_launch_command={self.driver_launch_command}, forward_cmd={self.forward_cmd}"
        )
        if run_dir:
            self.get_logger().info(f"runtime capture enabled: run_dir={run_dir}")

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
                    "Refusing to start because a floating trigger input can cause false activation. "
                    "Add an external pull-up/pull-down resistor or update the fixed GPIO configuration in this script."
                )

            self.get_logger().warn(
                "GPIO module does not expose PUD_UP/PUD_DOWN; proceeding without internal pull resistor because "
                "'allow_inputs_without_pull_resistors' is enabled"
            )
            GPIO.setup(self.forward_pin, GPIO.IN)
            GPIO.setup(self.reverse_pin, GPIO.IN)

    def _read_active(self, pin: int) -> bool:
        level = GPIO.input(pin)
        return not bool(level) if self.active_low else bool(level)

    def _start_line_follow_process(self) -> None:
        if self._shutting_down:
            return
        if self._line_follow_child is not None and self._line_follow_child.poll() is None:
            return

        self.get_logger().info(f"starting line-follow stack: {self.launch_command}")
        self.capture.log_event(f"starting line-follow stack: {self.launch_command}")
        launch_log = self.capture.open_log_file("line_follow_launch.log")
        self._line_follow_child = subprocess.Popen(
            ["bash", "-lc", self.launch_command],
            start_new_session=True,
            stdout=launch_log if launch_log is not None else subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )

    def _ensure_line_follow_process(self) -> bool:
        if self._shutting_down:
            return False
        if self._line_follow_child is not None and self._line_follow_child.poll() is None:
            return True
        self._start_line_follow_process()
        return self._line_follow_child is not None and self._line_follow_child.poll() is None

    def _launch_command_starts_driver(self) -> bool:
        return "start_driver:=false" not in self.launch_command.lower()

    def _should_manage_driver_process(self) -> bool:
        if not self.autostart_driver_process:
            return False
        if self.autostart_line_follow_stack and self._launch_command_starts_driver():
            return False
        return True

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
        if not self._should_manage_driver_process():
            return True
        if self._driver_child is not None and self._driver_child.poll() is None:
            return True
        self._start_driver_process()
        return self._driver_child is not None and self._driver_child.poll() is None

    def _set_line_follow_enabled(self, enabled: bool) -> bool:
        desired = bool(enabled)
        if desired == self._line_follow_enabled:
            return True
        if not desired and (self._line_follow_child is None or self._line_follow_child.poll() is not None):
            self._line_follow_enabled = False
            return True

        if not self.line_follow_param_client.service_is_ready():
            if not self.line_follow_param_client.wait_for_service(timeout_sec=self.line_follow_enable_timeout_sec):
                self.get_logger().warn(
                    f"parameter service not ready for {self.line_follow_node_name}, cannot set enabled={desired}"
                )
                return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter("enabled", Parameter.Type.BOOL, desired).to_parameter_msg()
        ]
        future = self.line_follow_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.line_follow_enable_timeout_sec)
        if not future.done():
            self.get_logger().warn(f"set enabled={desired} timed out for {self.line_follow_node_name}")
            return False
        if future.exception() is not None:
            self.get_logger().error(
                f"failed to set enabled={desired} for {self.line_follow_node_name}: {future.exception()}"
            )
            return False

        response = future.result()
        if response is None or not response.results:
            self.get_logger().warn(f"empty parameter result while setting enabled={desired}")
            return False

        result = response.results[0]
        if not result.successful:
            self.get_logger().warn(
                f"line-follow enable switch rejected: enabled={desired}, reason={result.reason}"
            )
            return False

        self._line_follow_enabled = desired
        self.get_logger().info(f"line-follow enabled set to {desired}")
        self.capture.log_event(f"line-follow enabled set to {desired}")
        return True

    def _can_publish(self) -> bool:
        return (
            not self._shutting_down
            and rclpy.ok()
            and getattr(self, "publisher", None) is not None
        )

    def _poll_trigger(self) -> None:
        self._check_driver_process()
        self._check_line_follow_process()
        if not self._ensure_driver_process():
            self.get_logger().error("motor driver process is not running, forcing idle")
            self._transition_to_idle()
            return

        forward_active = self._read_active(self.forward_pin)
        reverse_active = self._read_active(self.reverse_pin)
        now = time.monotonic()

        requested_mode = self._resolve_requested_mode(forward_active, reverse_active)
        if requested_mode is None:
            self._press_started_at = None
            self._hold_candidate = None
            self._transition_to_idle()
            return

        if requested_mode != self._hold_candidate:
            self._hold_candidate = requested_mode
            self._press_started_at = now
            if self._active_mode != requested_mode:
                self._transition_to_idle()
            return

        if self._active_mode == requested_mode:
            self._maintain_active_mode()
            return

        held_for = now - self._press_started_at
        if held_for >= self.hold_seconds:
            self._activate_mode(requested_mode)

    def _resolve_requested_mode(self, forward_active: bool, reverse_active: bool):
        if forward_active and not reverse_active:
            return "forward"
        if reverse_active and not forward_active:
            return "line_follow"
        return None

    def _activate_mode(self, mode: str) -> None:
        if mode == self._active_mode:
            return

        self._transition_to_idle()
        if mode == "forward":
            self.get_logger().warn("long press detected on forward channel, entering manual forward mode")
            self.capture.log_event("long press detected on forward channel, entering manual forward mode")
            self._set_line_follow_enabled(False)
            self._active_mode = "forward"
            self._publish_command(self.forward_cmd)
            return
        if mode == "line_follow":
            self.get_logger().warn("long press detected on reverse channel, enabling line-follow mode")
            self.capture.log_event("long press detected on reverse channel, enabling line-follow mode")
            if self._ensure_line_follow_process():
                self._active_mode = "line_follow_pending"
                if self._set_line_follow_enabled(True):
                    self._active_mode = "line_follow"
            return

        self.get_logger().warn(f"unsupported mode request ignored: {mode}")

    def _maintain_active_mode(self) -> None:
        if self._active_mode == "forward":
            self._set_line_follow_enabled(False)
            self._publish_command(self.forward_cmd)
        elif self._active_mode in {"line_follow", "line_follow_pending"}:
            if not self._ensure_line_follow_process():
                self.get_logger().error("line-follow mode requested but launch process is not running")
                self._transition_to_idle()
                return
            if self._set_line_follow_enabled(True):
                self._active_mode = "line_follow"

    def _transition_to_idle(self) -> None:
        self._set_line_follow_enabled(False)
        self._publish_command((0.0, 0.0))
        self._stop_line_follow()
        self.capture.log_event("transition to idle")
        self._active_mode = "idle"

    def _publish_command(self, cmd) -> None:
        if not self._can_publish():
            return
        msg = Float32MultiArray()
        msg.data = [float(cmd[0]), float(cmd[1])]
        try:
            self.publisher.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"publish speed command failed during shutdown or context teardown: {exc}")

    def _check_line_follow_process(self) -> None:
        if self._line_follow_child is None:
            return

        exit_code = self._line_follow_child.poll()
        if exit_code is None:
            return

        self.get_logger().error(f"line-follow launch exited with code {exit_code}")
        self.capture.log_event(f"line-follow launch exited with code {exit_code}")
        self._line_follow_child = None
        self._line_follow_enabled = False
        if self._active_mode in {"line_follow", "line_follow_pending"}:
            self._active_mode = "idle"
        self._press_started_at = None
        self._hold_candidate = None

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
        if child is None:
            return

        if child.poll() is not None:
            return

        try:
            self.get_logger().info(f"stopping {label} process")
            self.capture.log_event(f"stopping {label} process")
            pgid = os.getpgid(child.pid)
            os.killpg(pgid, signal.SIGINT)
            child.wait(timeout=5.0)
        except Exception:
            try:
                pgid = os.getpgid(child.pid)
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                pass

    def _stop_line_follow(self) -> None:
        if self._line_follow_child is None:
            return
        self._stop_process(self._line_follow_child, "line-follow launch")
        self._line_follow_child = None

    def _stop_driver_process(self) -> None:
        if self._driver_child is None:
            return
        self._stop_process(self._driver_child, "motor driver process")
        self._driver_child = None

    def destroy_node(self) -> bool:
        self._shutting_down = True
        try:
            self._transition_to_idle()
            self._stop_line_follow()
            self._stop_driver_process()
            self.capture.stop()
        finally:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        return super().destroy_node()


def main() -> int:
    node = None
    try:
        rclpy.init()
        node = RemoteLongPressStartNode()
    except Exception as exc:
        print(f"[remote_long_press_start_line_follow] startup failed: {exc}", file=sys.stderr)
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
