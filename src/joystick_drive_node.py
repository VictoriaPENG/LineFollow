#!/usr/bin/env python3
"""
Joystick drive control for RDK X5 40-pin header.

Default pin mapping with physical BOARD numbering:
- pin 11 -> forward
- pin 15 -> reverse
- pin 13 -> left
- pin 16 -> right

Only one direction is accepted at a time. If no input or multiple inputs are
active, the node publishes a stop command.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

from line_follow.gpio_compat import load_gpio_module
from line_follow.gpio_input_filter import DebouncedDigitalInput


GPIO = load_gpio_module()

FIXED_ACTIVE_LOW = True
FIXED_USE_PULL_UP = True
FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS = True


class JoystickDriveNode(Node):
    """Read four joystick GPIO inputs and publish motor speed commands."""

    def __init__(self) -> None:
        super().__init__("joystick_drive_node")

        self.declare_parameter("speed_topic", "/joystick_motor_speed_cmd")
        self.declare_parameter("override_active_topic", "/joystick_override_active")
        self.declare_parameter("forward_pin", 11)
        self.declare_parameter("reverse_pin", 15)
        self.declare_parameter("left_pin", 13)
        self.declare_parameter("right_pin", 16)
        self.declare_parameter("use_board_numbering", True)
        self.declare_parameter("debug_inputs_only", False)
        self.declare_parameter("debounce_activate_count", 5)
        self.declare_parameter("debounce_deactivate_count", 1)
        self.declare_parameter("forward_left_rpm", 900.0)
        self.declare_parameter("forward_right_rpm", -900.0)
        self.declare_parameter("reverse_left_rpm", -900.0)
        self.declare_parameter("reverse_right_rpm", 900.0)
        self.declare_parameter("turn_left_left_rpm", -900.0)
        self.declare_parameter("turn_left_right_rpm", -900.0)
        self.declare_parameter("turn_right_left_rpm", 900.0)
        self.declare_parameter("turn_right_right_rpm", 900.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("autostart_driver_process", False)
        self.declare_parameter(
            "driver_launch_command",
            "ros2 run line_follow motor_driver_control_node",
        )

        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.override_active_topic = str(self.get_parameter("override_active_topic").value)
        self.forward_pin = int(self.get_parameter("forward_pin").value)
        self.reverse_pin = int(self.get_parameter("reverse_pin").value)
        self.left_pin = int(self.get_parameter("left_pin").value)
        self.right_pin = int(self.get_parameter("right_pin").value)
        self.use_board_numbering = bool(self.get_parameter("use_board_numbering").value)
        self.debug_inputs_only = bool(self.get_parameter("debug_inputs_only").value)
        self.active_low = FIXED_ACTIVE_LOW
        self.allow_inputs_without_pull_resistors = FIXED_ALLOW_INPUTS_WITHOUT_PULL_RESISTORS
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
        self.turn_left_cmd = (
            float(self.get_parameter("turn_left_left_rpm").value),
            float(self.get_parameter("turn_left_right_rpm").value),
        )
        self.turn_right_cmd = (
            float(self.get_parameter("turn_right_left_rpm").value),
            float(self.get_parameter("turn_right_right_rpm").value),
        )
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.autostart_driver_process = bool(
            self.get_parameter("autostart_driver_process").value
        )
        self.driver_launch_command = str(self.get_parameter("driver_launch_command").value)

        self.pin_to_action = {
            self.forward_pin: "forward",
            self.reverse_pin: "reverse",
            self.left_pin: "left",
            self.right_pin: "right",
        }
        self.action_to_cmd = {
            "forward": self.forward_cmd,
            "reverse": self.reverse_cmd,
            "left": self.turn_left_cmd,
            "right": self.turn_right_cmd,
        }
        self.filters = {
            action: DebouncedDigitalInput(
                active_low=self.active_low,
                activate_count=debounce_activate_count,
                deactivate_count=debounce_deactivate_count,
            )
            for action in self.action_to_cmd
        }

        self.publisher = self.create_publisher(Float32MultiArray, self.speed_topic, 10)
        self.override_active_publisher = self.create_publisher(Bool, self.override_active_topic, 10)
        self.last_state = "stop"
        self.last_cmd = (0.0, 0.0)
        self.last_override_active = False
        self.last_debug_snapshot = None
        self._driver_child = None
        self._shutting_down = False
        self._override_publish_period_sec = 0.2
        self._last_override_publish_monotonic = 0.0

        self._setup_gpio()
        if self.autostart_driver_process:
            self._start_driver_process()
        self.timer = self.create_timer(1.0 / publish_hz, self._poll_inputs)

        self.get_logger().info(
            f"joystick drive ready: speed_topic={self.speed_topic}, "
            f"mode={'BOARD' if self.use_board_numbering else 'BCM'}, "
            f"pins={self.pin_to_action}, active_low={self.active_low}, "
            f"debug_inputs_only={self.debug_inputs_only}, "
            f"override_active_topic={self.override_active_topic}, "
            f"debounce_activate_count={debounce_activate_count}, "
            f"debounce_deactivate_count={debounce_deactivate_count}, "
            f"forward_cmd={self.forward_cmd}, reverse_cmd={self.reverse_cmd}, "
            f"turn_left_cmd={self.turn_left_cmd}, turn_right_cmd={self.turn_right_cmd}, "
            f"autostart_driver_process={self.autostart_driver_process}"
        )
        self._publish_command((0.0, 0.0))
        self._publish_override_active(False)

    def _setup_gpio(self) -> None:
        mode = GPIO.BOARD if self.use_board_numbering else GPIO.BCM
        GPIO.setwarnings(False)
        GPIO.setmode(mode)

        pins = (
            self.forward_pin,
            self.reverse_pin,
            self.left_pin,
            self.right_pin,
        )
        if hasattr(GPIO, "PUD_UP") and hasattr(GPIO, "PUD_DOWN"):
            pud = GPIO.PUD_UP if FIXED_USE_PULL_UP else GPIO.PUD_DOWN
            for pin in pins:
                GPIO.setup(pin, GPIO.IN, pull_up_down=pud)
        else:
            if not self.allow_inputs_without_pull_resistors:
                raise RuntimeError(
                    "GPIO module does not expose PUD_UP/PUD_DOWN. "
                    "Refusing to start because floating joystick inputs can trigger unintended motion."
                )

            self.get_logger().warn(
                "GPIO module does not expose PUD_UP/PUD_DOWN; proceeding without internal pull-up resistors"
            )
            for pin in pins:
                GPIO.setup(pin, GPIO.IN)

    def _read_pin(self, pin: int) -> Tuple[int, bool]:
        raw_level = int(GPIO.input(pin))
        raw_active = (raw_level == 0) if self.active_low else (raw_level != 0)
        return raw_level, raw_active

    def _start_driver_process(self) -> None:
        if self._shutting_down:
            return
        if self._driver_child is not None and self._driver_child.poll() is None:
            return

        self.get_logger().info(f"starting motor driver process: {self.driver_launch_command}")
        self._driver_child = subprocess.Popen(
            ["bash", "-lc", self.driver_launch_command],
            start_new_session=True,
            stdout=subprocess.DEVNULL,
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
        self._driver_child = None

    def _stop_process(self, child, label: str) -> None:
        if child is None or child.poll() is not None:
            return

        try:
            self.get_logger().info(f"stopping {label}")
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

    def _read_inputs(self) -> Dict[str, Tuple[int, bool, bool]]:
        snapshot = {}
        for pin, action in self.pin_to_action.items():
            raw_level, raw_active = self._read_pin(pin)
            _, filtered_active, _ = self.filters[action].update(raw_level)
            snapshot[action] = (raw_level, raw_active, filtered_active)
        return snapshot

    def _poll_inputs(self) -> None:
        self._check_driver_process()
        if not self._ensure_driver_process():
            self.get_logger().error("motor driver process is not running, forcing stop output")
            self._publish_command((0.0, 0.0))
            return

        snapshot = self._read_inputs()
        active_actions = [
            action for action, (_, _, filtered_active) in snapshot.items() if filtered_active
        ]

        if self.debug_inputs_only:
            debug_snapshot = tuple((action, *snapshot[action]) for action in sorted(snapshot))
            if debug_snapshot != self.last_debug_snapshot:
                state_text = ", ".join(
                    f"{action}:raw={snapshot[action][0]},raw_active={snapshot[action][1]},filtered_active={snapshot[action][2]}"
                    for action in ("forward", "reverse", "left", "right")
                )
                self.get_logger().info(f"joystick inputs: {state_text}")
                self.last_debug_snapshot = debug_snapshot
            self.last_state = "debug"
            self._publish_command((0.0, 0.0))
            self._publish_override_active(False)
            return

        if len(active_actions) == 1:
            state = active_actions[0]
            cmd = self.action_to_cmd[state]
            override_active = True
        elif len(active_actions) > 1:
            state = "stop"
            cmd = (0.0, 0.0)
            override_active = True
        else:
            state = "stop"
            cmd = (0.0, 0.0)
            override_active = False

        if state != self.last_state:
            self.get_logger().info(
                f"joystick state changed: active_actions={active_actions}, action={state}"
            )
            self.last_state = state

        self._publish_command(cmd)
        self._publish_override_active(override_active)

    def _publish_command(self, cmd: Tuple[float, float]) -> None:
        msg = Float32MultiArray()
        msg.data = [float(cmd[0]), float(cmd[1])]
        self.publisher.publish(msg)
        self.last_cmd = cmd

    def _publish_override_active(self, active: bool, force: bool = False) -> None:
        now = rclpy.clock.Clock().now().nanoseconds / 1e9
        same_value = bool(active) == self.last_override_active
        recently_published = (now - self._last_override_publish_monotonic) < self._override_publish_period_sec
        if same_value and not force and recently_published:
            return
        msg = Bool()
        msg.data = bool(active)
        self.override_active_publisher.publish(msg)
        self.last_override_active = bool(active)
        self._last_override_publish_monotonic = now

    def destroy_node(self) -> bool:
        self._shutting_down = True
        try:
            self._publish_override_active(False)
        except Exception:
            pass
        try:
            self._publish_command((0.0, 0.0))
        except Exception:
            pass
        try:
            self._stop_driver_process()
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
        node = JoystickDriveNode()
    except Exception as exc:
        print(f"[joystick_drive] startup failed: {exc}", file=sys.stderr)
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
