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
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

from line_follow.gpio_compat import load_gpio_module
from line_follow.gpio_input_filter import DebouncedDigitalInput
from line_follow.runtime_capture import RuntimeCaptureManager

GPIO = load_gpio_module()

FIXED_ACTIVE_LOW = True
FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS = True


class RemoteLongPressStartNode(Node):
    """Arbitrate long-press remote modes without command-topic conflicts."""

    def __init__(self) -> None:
        super().__init__("remote_long_press_start_line_follow_node")

        self.declare_parameter("forward_pin", 31)
        self.declare_parameter("reverse_pin", 29)
        self.declare_parameter("use_board_numbering", True)
        self.declare_parameter("debug_inputs_only", False)
        self.declare_parameter("hold_seconds", 0.5)
        self.declare_parameter("poll_hz", 20.0)
        self.declare_parameter("debounce_activate_count", 5)
        self.declare_parameter("debounce_deactivate_count", 1)
        self.declare_parameter("autostart_line_follow_stack", True)
        self.declare_parameter(
            "allow_inputs_without_pull_resistors",
            FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS,
        )
        self.declare_parameter(
            "launch_command",
            "ros2 launch line_follow line_follow_system.launch.py line_follow_enabled:=false start_driver:=false",
        )
        self.declare_parameter("speed_topic", "/motor_speed_cmd")
        self.declare_parameter("override_active_topic", "/joystick_override_active")
        self.declare_parameter("forward_left_rpm", -600.0)
        self.declare_parameter("forward_right_rpm", 600.0)
        self.declare_parameter("line_follow_node_name", "/line_follow_motor_model_node")
        self.declare_parameter("adopt_existing_line_follow_stack", True)
        self.declare_parameter("line_follow_enable_timeout_sec", 2.0)
        self.declare_parameter("line_follow_disable_timeout_sec", 0.2)
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
        self.debug_inputs_only = bool(self.get_parameter("debug_inputs_only").value)
        self.active_low = FIXED_ACTIVE_LOW
        debounce_activate_count = int(self.get_parameter("debounce_activate_count").value)
        debounce_deactivate_count = int(self.get_parameter("debounce_deactivate_count").value)
        self.allow_inputs_without_pull_resistors = bool(
            self.get_parameter("allow_inputs_without_pull_resistors").value
        )
        self.hold_seconds = max(0.2, float(self.get_parameter("hold_seconds").value))
        self.autostart_line_follow_stack = bool(
            self.get_parameter("autostart_line_follow_stack").value
        )
        self.launch_command = str(self.get_parameter("launch_command").value)
        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.override_active_topic = str(self.get_parameter("override_active_topic").value)
        self.forward_cmd = (
            float(self.get_parameter("forward_left_rpm").value),
            float(self.get_parameter("forward_right_rpm").value),
        )
        self.line_follow_node_name = str(self.get_parameter("line_follow_node_name").value)
        self.adopt_existing_line_follow_stack = bool(
            self.get_parameter("adopt_existing_line_follow_stack").value
        )
        self.line_follow_enable_timeout_sec = max(
            0.05, float(self.get_parameter("line_follow_enable_timeout_sec").value)
        )
        self.line_follow_disable_timeout_sec = max(
            0.05, float(self.get_parameter("line_follow_disable_timeout_sec").value)
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
        self._line_follow_child = None
        self._driver_child = None
        self._line_follow_enabled = False
        self._using_external_line_follow_stack = False
        self._last_line_follow_enable_attempt = 0.0
        self._shutting_down = False
        self._joystick_override_active = False
        self._remote_inhibited_by_joystick = False
        self._joystick_override_stop_sent = False
        self._last_input_debug_snapshot = None
        self.publisher = self.create_publisher(Float32MultiArray, self.speed_topic, 10)
        self.override_active_sub = self.create_subscription(
            Bool,
            self.override_active_topic,
            self._on_joystick_override_active,
            10,
        )
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
            f"active_low={self.active_low}, debug_inputs_only={self.debug_inputs_only}, "
            f"hold_seconds={self.hold_seconds:.2f}, "
            f"debounce_activate_count={debounce_activate_count}, "
            f"debounce_deactivate_count={debounce_deactivate_count}, "
            f"autostart_line_follow_stack={self.autostart_line_follow_stack}, "
            f"autostart_driver_process={self.autostart_driver_process}, "
            f"allow_inputs_without_pull_resistors={self.allow_inputs_without_pull_resistors}, "
            f"launch_command={self.launch_command}, line_follow_node_name={self.line_follow_node_name}, "
            f"override_active_topic={self.override_active_topic}, "
            f"adopt_existing_line_follow_stack={self.adopt_existing_line_follow_stack}, "
            f"line_follow_enable_timeout_sec={self.line_follow_enable_timeout_sec:.2f}, "
            f"line_follow_disable_timeout_sec={self.line_follow_disable_timeout_sec:.2f}, "
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

    def _read_active(self, pin: int, filter_state: DebouncedDigitalInput) -> bool:
        level = int(GPIO.input(pin))
        _, filtered_active, _ = filter_state.update(level)
        return filtered_active

    def _start_line_follow_process(self) -> None:
        if self._shutting_down:
            return
        if self._line_follow_child is not None and self._line_follow_child.poll() is None:
            return
        if self.adopt_existing_line_follow_stack and self._line_follow_stack_available():
            if not self._using_external_line_follow_stack:
                self.get_logger().warn(
                    "existing line-follow stack detected, adopting it instead of starting a duplicate launch"
                )
                self.capture.log_event(
                    "existing line-follow stack detected, adopting it instead of starting a duplicate launch"
                )
            self._using_external_line_follow_stack = True
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
        self._using_external_line_follow_stack = False

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

    def _line_follow_service_available(self, timeout_sec: float = 0.0) -> bool:
        try:
            if self.line_follow_param_client.service_is_ready():
                return True
            if timeout_sec <= 0.0:
                return False
            return bool(self.line_follow_param_client.wait_for_service(timeout_sec=timeout_sec))
        except Exception as exc:
            self.get_logger().warn(f"line-follow parameter service check failed: {exc}")
            return False

    def _line_follow_stack_available(self) -> bool:
        if self._line_follow_child is not None and self._line_follow_child.poll() is None:
            return True
        return self._line_follow_target_node_running() or self._line_follow_service_available(
            timeout_sec=0.05
        )

    def _ensure_line_follow_process(self) -> bool:
        if self._shutting_down:
            return False
        if self._line_follow_stack_available():
            return True
        self._start_line_follow_process()
        return self._line_follow_stack_available()

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

    def _set_line_follow_enabled(self, enabled: bool, force: bool = False) -> bool:
        desired = bool(enabled)
        if desired == self._line_follow_enabled and not force:
            return True
        if not desired and not self._line_follow_enabled:
            child_running = self._line_follow_child is not None and self._line_follow_child.poll() is None
            if not child_running and not self._line_follow_service_available(timeout_sec=0.0):
                return True

        service_timeout = (
            self.line_follow_enable_timeout_sec
            if desired
            else self.line_follow_disable_timeout_sec
        )
        if not self._line_follow_service_available(timeout_sec=service_timeout):
            if desired:
                self.get_logger().warn(
                    f"parameter service not ready for {self.line_follow_node_name}, cannot set enabled={desired}"
                )
                return False
            self.get_logger().warn(
                f"parameter service not ready for {self.line_follow_node_name}, cannot confirm line-follow stopped"
            )
            self._line_follow_enabled = False
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter("enabled", Parameter.Type.BOOL, desired).to_parameter_msg()
        ]
        try:
            future = self.line_follow_param_client.call_async(request)
            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=service_timeout,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"set enabled={desired} failed before completion for {self.line_follow_node_name}: {exc}"
            )
            return False
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
        self._check_driver_process()
        self._check_line_follow_process()
        if not self._ensure_driver_process():
            self.get_logger().error("motor driver process is not running, forcing idle")
            self._transition_to_idle()
            return

        forward_active = self._read_active(self.forward_pin, self.forward_filter)
        reverse_active = self._read_active(self.reverse_pin, self.reverse_filter)
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
            if self._ensure_line_follow_process():
                self._active_mode = "line_follow_pending"
                self._last_line_follow_enable_attempt = time.monotonic()
                if self._set_line_follow_enabled(True):
                    self._active_mode = "line_follow"
                else:
                    self.get_logger().warn("line-follow stack is starting, keep waiting while reverse channel is held")
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
        if not disabled and self._line_follow_child is not None:
            if self._using_external_line_follow_stack:
                self.get_logger().warn(
                    "line-follow disable failed, cannot stop external line-follow launch"
                )
            else:
                self.get_logger().warn("line-follow disable failed, stopping managed launch process")
                self._stop_line_follow()
        elif not self.autostart_line_follow_stack:
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
        self._using_external_line_follow_stack = False
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

    def _wait_or_signal(self, child, timeout_sec: float, sig) -> bool:
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
        except ProcessLookupError:
            return
        except Exception as exc:
            self.get_logger().warn(f"failed to signal {label} process group with SIGINT: {exc}")
            return

        if self._wait_or_signal(child, 2.0, signal.SIGTERM):
            return
        self.get_logger().warn(f"{label} did not stop after SIGINT, sent SIGTERM")

        if self._wait_or_signal(child, 2.0, signal.SIGKILL):
            return
        self.get_logger().warn(f"{label} did not stop after SIGTERM, sent SIGKILL")

        try:
            child.wait(timeout=1.0)
        except Exception:
            pass

    def _stop_line_follow(self) -> None:
        if self._line_follow_child is None:
            return
        self._stop_process(self._line_follow_child, "line-follow launch")
        self._line_follow_child = None
        self._using_external_line_follow_stack = False

    def _stop_driver_process(self) -> None:
        if self._driver_child is None:
            return
        self._stop_process(self._driver_child, "motor driver process")
        self._driver_child = None

    def destroy_node(self) -> bool:
        self._shutting_down = True
        try:
            self._transition_to_idle(force=True)
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
