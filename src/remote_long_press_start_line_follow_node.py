#!/usr/bin/env python3
"""
长按遥控模式控制节点。

默认行为：
1. 监听 40Pin 排针上的前进/后退继电器输入
2. 默认假设巡线整栈已经由 launch 或 systemd 预先拉起
3. 长按“后退”键时切入巡线模式，并在按住期间保持有效
4. 长按“前进”键时退出巡线模式，改为手动前进
5. 任意按键释放后，车辆立即停车

这个节点的核心职责不是直接驱动底层硬件，而是做模式仲裁：
决定当前到底由自动巡线控制车辆，还是由人工临时接管。
"""

from __future__ import annotations

import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

from line_follow.gpio_compat import load_gpio_module
from line_follow.gpio_input_filter import DebouncedDigitalInput
from line_follow.runtime_capture import RuntimeCaptureManager

GPIO = load_gpio_module()

FIXED_ACTIVE_LOW = True
FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS = False


class RemoteLongPressStartNode(Node):
    """在长按遥控模式下做巡线/手动控制切换。"""

    def __init__(self) -> None:
        super().__init__("remote_long_press_start_line_follow_node")

        pid_file = "/tmp/remote_long_press_line_follow.pid"
        try:
            if os.path.exists(pid_file):
                with open(pid_file, "r") as f:
                    old_pid = int(f.read().strip())
                try:
                    os.kill(old_pid, 0)
                    self.get_logger().error(f"Another instance is already running (PID: {old_pid}). Exiting.")
                    sys.exit(1)
                except OSError:
                    self.get_logger().warn(f"Stale PID file found (PID: {old_pid}), removing it.")
                    os.remove(pid_file)
            with open(pid_file, "w") as f:
                f.write(str(os.getpid()))
            self._pid_file = pid_file
        except Exception as exc:
            self.get_logger().warn(f"Failed to create PID file: {exc}. Continuing without PID lock.")
            self._pid_file = None

        self.declare_parameter("forward_pin", 31)
        self.declare_parameter("reverse_pin", 29)
        self.declare_parameter("use_board_numbering", True)
        self.declare_parameter("debug_inputs_only", False)
        self.declare_parameter("hold_seconds", 0.5)
        self.declare_parameter("poll_hz", 20.0)
        self.declare_parameter("debounce_activate_count", 5)
        self.declare_parameter("debounce_deactivate_count", 1)
        self.declare_parameter(
            "allow_inputs_without_pull_resistors",
            FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS,
        )
        self.declare_parameter("speed_topic", "/motor_speed_cmd")
        self.declare_parameter("override_active_topic", "/joystick_override_active")
        self.declare_parameter("line_follow_enable_topic", "/line_follow/set_enabled")
        self.declare_parameter("forward_left_rpm", -900.0)
        self.declare_parameter("forward_right_rpm", 900.0)
        self.declare_parameter("line_follow_node_name", "/line_follow_motor_model_node")
        self.declare_parameter("auto_record_runtime_data", True)
        self.declare_parameter("record_root_dir", "/userdata/test_logs")
        self.declare_parameter("record_rosbag", True)
        self.declare_parameter(
            "save_params_script",
            "/userdata/dev_ws/src/originbot/Line_follow/board_tools/save_runtime_params.sh",
        )
        self.declare_parameter("gpio_retry_attempts", 10)
        self.declare_parameter("gpio_retry_interval_sec", 0.2)
        self.declare_parameter("gpio_retry_backoff_sec", 5.0)
        self.declare_parameter("gpio_read_error_limit", 3)

        self.forward_pin = int(self.get_parameter("forward_pin").value)
        self.reverse_pin = int(self.get_parameter("reverse_pin").value)
        self.use_board_numbering = bool(self.get_parameter("use_board_numbering").value)
        self.debug_inputs_only = bool(self.get_parameter("debug_inputs_only").value)
        self.active_low = FIXED_ACTIVE_LOW
        debounce_activate_count = int(self.get_parameter("debounce_activate_count").value)
        debounce_deactivate_count = int(self.get_parameter("debounce_deactivate_count").value)
        self.allow_inputs_without_pull_resistors = bool(
            self.get_parameter("allow_inputs_without_pull_resistors").value
        )
        self.hold_seconds = max(0.2, float(self.get_parameter("hold_seconds").value))
        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.override_active_topic = str(self.get_parameter("override_active_topic").value)
        self.line_follow_enable_topic = str(self.get_parameter("line_follow_enable_topic").value)
        self.forward_cmd = (
            float(self.get_parameter("forward_left_rpm").value),
            float(self.get_parameter("forward_right_rpm").value),
        )
        self.line_follow_node_name = str(self.get_parameter("line_follow_node_name").value)
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
        self.gpio_retry_attempts = max(1, int(self.get_parameter("gpio_retry_attempts").value))
        self.gpio_retry_interval_sec = max(0.05, float(self.get_parameter("gpio_retry_interval_sec").value))
        self.gpio_retry_backoff_sec = max(0.5, float(self.get_parameter("gpio_retry_backoff_sec").value))
        self.gpio_read_error_limit = max(1, int(self.get_parameter("gpio_read_error_limit").value))
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

        self._press_started_at = None
        self._hold_candidate = None
        self._active_mode = "idle"
        self._line_follow_enabled = False
        self._last_line_follow_enable_attempt = 0.0
        self._shutting_down = False
        self._joystick_override_active = False
        self._remote_inhibited_by_joystick = False
        self._joystick_override_stop_sent = False
        self._last_input_debug_snapshot = None
        self._gpio_ready = False
        self._gpio_retry_count = 0
        self._next_gpio_retry_monotonic = 0.0
        self._last_gpio_error = ""
        self._gpio_read_error_streak = 0
        self.publisher = self.create_publisher(Float32MultiArray, self.speed_topic, 10)
        self.line_follow_enable_pub = self.create_publisher(
            Bool,
            self.line_follow_enable_topic,
            10,
        )
        self.override_active_sub = self.create_subscription(
            Bool,
            self.override_active_topic,
            self._on_joystick_override_active,
            10,
        )
        self._ensure_gpio_ready(force=True)
        run_dir = self.capture.start()
        self.timer = self.create_timer(1.0 / poll_hz, self._poll_trigger)
        self._publish_command((0.0, 0.0))
        self._publish_line_follow_enabled(False, force=True)

        self.get_logger().info(
            f"long-press trigger ready: forward_pin={self.forward_pin}, reverse_pin={self.reverse_pin}, "
            f"mode={'BOARD' if self.use_board_numbering else 'BCM'}, "
            f"active_low={self.active_low}, debug_inputs_only={self.debug_inputs_only}, "
            f"hold_seconds={self.hold_seconds:.2f}, "
            f"debounce_activate_count={debounce_activate_count}, "
            f"debounce_deactivate_count={debounce_deactivate_count}, "
            f"allow_inputs_without_pull_resistors={self.allow_inputs_without_pull_resistors}, "
            f"gpio_retry_attempts={self.gpio_retry_attempts}, "
            f"gpio_retry_interval_sec={self.gpio_retry_interval_sec:.2f}, "
            f"gpio_retry_backoff_sec={self.gpio_retry_backoff_sec:.2f}, "
            f"gpio_read_error_limit={self.gpio_read_error_limit}, "
            f"line_follow_node_name={self.line_follow_node_name}, "
            f"line_follow_enable_topic={self.line_follow_enable_topic}, "
            f"override_active_topic={self.override_active_topic}, "
            f"forward_cmd={self.forward_cmd}"
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

    def _mark_gpio_unavailable(self, reason: str) -> None:
        self._gpio_ready = False
        self._last_gpio_error = str(reason)
        try:
            GPIO.cleanup()
        except Exception:
            pass
        self._transition_to_idle(force=True)

    def _ensure_gpio_ready(self, force: bool = False) -> bool:
        if self._gpio_ready:
            return True

        now = time.monotonic()
        if not force and now < self._next_gpio_retry_monotonic:
            return False

        try:
            self._setup_gpio()
        except Exception as exc:
            self._gpio_retry_count += 1
            self._last_gpio_error = str(exc)
            if self._gpio_retry_count >= self.gpio_retry_attempts:
                self._next_gpio_retry_monotonic = now + self.gpio_retry_backoff_sec
                self.get_logger().error(
                    f"remote GPIO init failed {self._gpio_retry_count}/{self.gpio_retry_attempts}: {exc}; "
                    f"retry after {self.gpio_retry_backoff_sec:.1f}s"
                )
                self._gpio_retry_count = 0
            else:
                self._next_gpio_retry_monotonic = now + self.gpio_retry_interval_sec
                self.get_logger().warn(
                    f"remote GPIO init failed {self._gpio_retry_count}/{self.gpio_retry_attempts}: {exc}; "
                    f"retry in {self.gpio_retry_interval_sec:.1f}s"
                )
            self._mark_gpio_unavailable(exc)
            return False

        self._gpio_ready = True
        self._gpio_retry_count = 0
        self._next_gpio_retry_monotonic = 0.0
        self._gpio_read_error_streak = 0
        if self._last_gpio_error:
            self.get_logger().info("remote GPIO recovered")
            self._last_gpio_error = ""
        return True

    def _read_active(self, pin: int, filter_state: DebouncedDigitalInput) -> bool:
        level = int(GPIO.input(pin))
        _, filtered_active, _ = filter_state.update(level)
        return filtered_active

    def _line_follow_target_node_running(self) -> bool:
        target_name = self.line_follow_node_name.strip() or "/line_follow_motor_model_node"
        try:
            node_names = self.get_node_names_and_namespaces()
        except Exception:
            return False
        for node_name, namespace in node_names:
            full_name = f"{namespace.rstrip('/')}/{node_name}".replace("//", "/")
            if not full_name.startswith("/"):
                full_name = f"/{full_name}"
            if full_name == target_name:
                return True
        return False

    def _line_follow_stack_available(self) -> bool:
        return self._line_follow_target_node_running()

    def _set_line_follow_enabled(self, enabled: bool, force: bool = False) -> bool:
        desired = bool(enabled)
        if desired == self._line_follow_enabled and not force:
            return True
        if not self._line_follow_stack_available():
            if desired:
                self.get_logger().warn(
                    f"line-follow node {self.line_follow_node_name} is not available, cannot enable"
                )
                return False
            self._line_follow_enabled = False
            return False

        self._publish_line_follow_enabled(desired, force=True)
        self._line_follow_enabled = desired
        self.get_logger().info(f"line-follow enabled set to {desired}")
        self.capture.log_event(f"line-follow enabled set to {desired}")
        return True

    def _publish_line_follow_enabled(self, enabled: bool, force: bool = False) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        count = 3 if force else 1
        for _ in range(count):
            self.line_follow_enable_pub.publish(msg)
            if count > 1:
                time.sleep(0.02)

    def _publish_stop_burst(self, count: int = 3, delay_sec: float = 0.02) -> None:
        for _ in range(max(1, count)):
            self._publish_command((0.0, 0.0))
            if delay_sec > 0.0:
                time.sleep(delay_sec)

    def _can_publish(self) -> bool:
        return (
            rclpy.ok()
            and getattr(self, "publisher", None) is not None
        )

    def _poll_trigger(self) -> None:
        if not self._ensure_gpio_ready():
            return

        try:
            forward_active = self._read_active(self.forward_pin, self.forward_filter)
            reverse_active = self._read_active(self.reverse_pin, self.reverse_filter)
        except Exception as exc:
            self._gpio_read_error_streak += 1
            error_text = f"{type(exc).__name__}: {exc!r}"
            if self._gpio_read_error_streak >= self.gpio_read_error_limit:
                self.get_logger().error(
                    f"remote GPIO read failed {self._gpio_read_error_streak}/{self.gpio_read_error_limit}: "
                    f"{error_text}; marking GPIO unavailable"
                )
                self._mark_gpio_unavailable(error_text)
                self._next_gpio_retry_monotonic = 0.0
            else:
                self.get_logger().warn(
                    f"remote GPIO transient read failure {self._gpio_read_error_streak}/{self.gpio_read_error_limit}: "
                    f"{error_text}; keeping current mode"
                )
            return
        if self._gpio_read_error_streak > 0:
            self.get_logger().info(
                f"remote GPIO read recovered after {self._gpio_read_error_streak} consecutive failures"
            )
            self._gpio_read_error_streak = 0
        now = time.monotonic()

        requested_mode = self._resolve_requested_mode(forward_active, reverse_active)
        self._log_input_state(forward_active, reverse_active, requested_mode)
        if self.debug_inputs_only:
            return

        if self._joystick_override_active:
            self._remote_inhibited_by_joystick = True
            self._press_started_at = None
            self._hold_candidate = None
            if not self._joystick_override_stop_sent:
                self._transition_to_idle(force=True)
                self._joystick_override_stop_sent = True
            return

        if self._remote_inhibited_by_joystick:
            if requested_mode is None:
                self.get_logger().warn("remote inputs released after joystick override, remote control re-armed")
                self._remote_inhibited_by_joystick = False
                self._joystick_override_stop_sent = False
            else:
                self._press_started_at = None
                self._hold_candidate = None
                if not self._joystick_override_stop_sent:
                    self._transition_to_idle(force=True)
                    self._joystick_override_stop_sent = True
                return

        if requested_mode is None:
            self._press_started_at = None
            self._hold_candidate = None
            self._transition_to_idle()
            return

        if requested_mode != self._hold_candidate:
            self._hold_candidate = requested_mode
            self._press_started_at = now
            if not self._active_mode_matches_request(requested_mode):
                self._transition_to_idle()
            return

        if self._active_mode_matches_request(requested_mode):
            self._maintain_active_mode()
            return

        held_for = now - self._press_started_at
        if held_for >= self.hold_seconds:
            self._activate_mode(requested_mode)

    def _log_input_state(self, forward_active: bool, reverse_active: bool, requested_mode) -> None:
        snapshot = (
            bool(forward_active),
            bool(reverse_active),
            requested_mode,
            self._hold_candidate,
            self._active_mode,
            bool(self._line_follow_enabled),
            bool(self._joystick_override_active),
            bool(self._remote_inhibited_by_joystick),
            bool(self._joystick_override_stop_sent),
        )
        if snapshot == self._last_input_debug_snapshot:
            return
        self._last_input_debug_snapshot = snapshot
        self.get_logger().warn(
            "remote input state: "
            f"forward_active={forward_active}, reverse_active={reverse_active}, "
            f"requested_mode={requested_mode}, hold_candidate={self._hold_candidate}, "
            f"active_mode={self._active_mode}, line_follow_enabled={self._line_follow_enabled}, "
            f"joystick_override_active={self._joystick_override_active}, "
            f"remote_inhibited_by_joystick={self._remote_inhibited_by_joystick}, "
            f"joystick_override_stop_sent={self._joystick_override_stop_sent}"
        )

    def _resolve_requested_mode(self, forward_active: bool, reverse_active: bool):
        if forward_active and not reverse_active:
            return "forward"
        if reverse_active and not forward_active:
            return "line_follow"
        return None

    def _active_mode_matches_request(self, requested_mode) -> bool:
        if requested_mode == self._active_mode:
            return True
        return requested_mode == "line_follow" and self._active_mode == "line_follow_pending"

    def _activate_mode(self, mode: str) -> None:
        if self._active_mode_matches_request(mode):
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
            if self._line_follow_stack_available():
                self._active_mode = "line_follow_pending"
                self._last_line_follow_enable_attempt = time.monotonic()
                if self._set_line_follow_enabled(True):
                    self._active_mode = "line_follow"
                else:
                    self.get_logger().warn("line-follow stack is starting, keep waiting while reverse channel is held")
            else:
                self.get_logger().warn("line-follow stack is not available, cannot enable line-follow mode")
            return

        self.get_logger().warn(f"unsupported mode request ignored: {mode}")

    def _maintain_active_mode(self) -> None:
        if self._active_mode == "forward":
            self._set_line_follow_enabled(False)
            self._publish_command(self.forward_cmd)
        elif self._active_mode in {"line_follow", "line_follow_pending"}:
            if not self._line_follow_stack_available():
                self.get_logger().error("line-follow mode requested but line-follow stack is not available")
                self._transition_to_idle()
                return
            now = time.monotonic()
            if (
                self._active_mode == "line_follow_pending"
                and now - self._last_line_follow_enable_attempt < 0.5
            ):
                return
            self._last_line_follow_enable_attempt = now
            if self._set_line_follow_enabled(True):
                self._active_mode = "line_follow"
            else:
                self._active_mode = "line_follow_pending"

    def _transition_to_idle(self, force: bool = False) -> None:
        already_idle = self._active_mode == "idle" and not self._line_follow_enabled
        if already_idle and not force:
            return

        # Publish an immediate stop before toggling line-follow state so the
        # chassis does not wait on parameter-service latency to begin braking.
        self._publish_stop_burst()
        if self._shutting_down:
            self._active_mode = "idle"
            return

        disabled = self._set_line_follow_enabled(False, force=True)
        self._publish_stop_burst()
        if not disabled:
            self.get_logger().warn(
                "line-follow disable failed; keeping idle state but line-follow node may still be active"
            )
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

    def _on_joystick_override_active(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active == self._joystick_override_active:
            return
        self._joystick_override_active = active
        if active:
            self.get_logger().warn("joystick override active, remote and line-follow commands inhibited")
            self.capture.log_event("joystick override active, remote and line-follow commands inhibited")
            self._remote_inhibited_by_joystick = True
            self._press_started_at = None
            self._hold_candidate = None
            if not self._joystick_override_stop_sent:
                self._transition_to_idle(force=True)
                self._joystick_override_stop_sent = True
        else:
            self.get_logger().warn("joystick override inactive, waiting for remote inputs to be released")
            self.capture.log_event("joystick override inactive, waiting for remote inputs to be released")

    def destroy_node(self) -> bool:
        self._shutting_down = True
        try:
            self._transition_to_idle(force=True)
            self.capture.stop()
        finally:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        if hasattr(self, "_pid_file") and self._pid_file and os.path.exists(self._pid_file):
            try:
                os.remove(self._pid_file)
            except Exception:
                pass
        return super().destroy_node()


def main() -> int:
    node = None
    pid_file = "/tmp/remote_long_press_line_follow.pid"

    def cleanup_handler(signum, frame):
        if os.path.exists(pid_file):
            try:
                os.remove(pid_file)
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, cleanup_handler)
    signal.signal(signal.SIGINT, cleanup_handler)

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
