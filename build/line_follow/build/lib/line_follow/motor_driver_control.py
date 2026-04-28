#!/usr/bin/env python3
"""
电机驱动控制节点。

功能概述：
1. 订阅 `/motor_speed_cmd`，消息格式为 `[left_rpm, right_rpm]`
2. 将转速转换为驱动器 Modbus RTU 写寄存器指令
3. 向 `0x8110` 写入速度值
4. 向 `0x8106` 写入启停和方向控制字
5. 支持正转、反转、停止
6. 对写响应和读响应做 CRC 基本校验
7. 提供本节点侧命令超时保护、写失败急停和故障锁定

协议约定：
- 驱动器站号默认是左 `0x06`、右 `0x08`
- 写单寄存器功能码是 `0x06`
- 读保持寄存器功能码是 `0x03`
- 速度寄存器是 `0x8110`
- 模式寄存器是 `0x8106`
"""

import struct
import time

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray


class MotorDriverControlNode(Node):
    """订阅左右转速并通过 Modbus RTU 下发到驱动器。"""

    REG_MODE = 0x8106
    REG_SPEED = 0x8110

    MODE_STOP = 0x0700
    MODE_RUN_FWD = 0x0701
    MODE_RUN_REV = 0x0703

    def __init__(self):
        super().__init__("motor_driver_control_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("serial_timeout_sec", 0.1)
        self.declare_parameter("speed_topic", "/motor_speed_cmd")
        self.declare_parameter("override_speed_topic", "/joystick_motor_speed_cmd")
        self.declare_parameter("override_active_topic", "/joystick_override_active")
        self.declare_parameter("speed_status_topic", "/motor_speed_status")
        self.declare_parameter("override_active_timeout_sec", 0.3)
        self.declare_parameter("left_slave", 6)
        self.declare_parameter("right_slave", 8)
        self.declare_parameter("min_speed_rpm", 0)
        self.declare_parameter("max_speed_rpm", 5000)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("driver_command_timeout_sec", 0.5)
        self.declare_parameter("write_retry_count", 2)
        self.declare_parameter("fail_safe_on_write_error", True)
        self.declare_parameter("max_consecutive_write_errors", 3)
        self.declare_parameter("fault_reset_counter", 0)
        self.declare_parameter("reconnect_retry_attempts", 10)
        self.declare_parameter("reconnect_retry_interval_sec", 0.2)
        self.declare_parameter("reconnect_retry_backoff_sec", 5.0)

        self.ser = None
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.serial_timeout_sec = max(0.01, float(self.get_parameter("serial_timeout_sec").value))
        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.override_speed_topic = str(self.get_parameter("override_speed_topic").value)
        self.override_active_topic = str(self.get_parameter("override_active_topic").value)
        self.speed_status_topic = str(self.get_parameter("speed_status_topic").value)
        self.override_active_timeout_sec = max(
            0.05, float(self.get_parameter("override_active_timeout_sec").value)
        )
        self.left_slave = int(self.get_parameter("left_slave").value)
        self.right_slave = int(self.get_parameter("right_slave").value)
        self.min_speed_rpm = max(0, int(self.get_parameter("min_speed_rpm").value))
        self.max_speed_rpm = max(self.min_speed_rpm, int(self.get_parameter("max_speed_rpm").value))
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.driver_command_timeout_sec = max(0.05, float(self.get_parameter("driver_command_timeout_sec").value))
        self.write_retry_count = max(0, int(self.get_parameter("write_retry_count").value))
        self.fail_safe_on_write_error = bool(self.get_parameter("fail_safe_on_write_error").value)
        self.max_consecutive_write_errors = max(1, int(self.get_parameter("max_consecutive_write_errors").value))
        self.fault_reset_counter = int(self.get_parameter("fault_reset_counter").value)
        self.reconnect_retry_attempts = max(1, int(self.get_parameter("reconnect_retry_attempts").value))
        self.reconnect_retry_interval_sec = max(
            0.05, float(self.get_parameter("reconnect_retry_interval_sec").value)
        )
        self.reconnect_retry_backoff_sec = max(
            0.5, float(self.get_parameter("reconnect_retry_backoff_sec").value)
        )

        self.driver_faulted = False
        self.fault_reason = ""
        self.write_error_streak = 0
        self.last_speed_cmd_stamp = None
        self.last_command_was_stop = True
        self.override_active = False
        self.override_active_stamp = None
        self.base_command = (0.0, 0.0)
        self.override_command = (0.0, 0.0)
        self.override_command_stamp = None
        self.driver_available = False
        self._next_reconnect_monotonic = 0.0
        self._last_reconnect_reason = ""

        self.sub = self.create_subscription(Float32MultiArray, self.speed_topic, self.on_base_speed_cmd, 10)
        self.speed_status_pub = self.create_publisher(Float32MultiArray, self.speed_status_topic, 10)
        self.override_sub = self.create_subscription(
            Float32MultiArray,
            self.override_speed_topic,
            self.on_override_speed_cmd,
            10,
        )
        self.override_active_sub = self.create_subscription(
            Bool,
            self.override_active_topic,
            self.on_override_active,
            10,
        )
        self.timeout_timer = self.create_timer(0.1, self._on_command_timeout_check)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._publish_speed_status(0.0, 0.0)
        self._recover_driver(force=True, reason="startup")
        self.get_logger().info(
            f"Subscribed to base={self.speed_topic}, override={self.override_speed_topic}, "
            f"speed_status={self.speed_status_topic}, "
            f"override_active={self.override_active_topic}, driver left={self.left_slave}, right={self.right_slave}, "
            f"timeout={self.driver_command_timeout_sec:.2f}s, retries={self.write_retry_count}, "
            f"override_active_timeout={self.override_active_timeout_sec:.2f}s, "
            f"max_consecutive_write_errors={self.max_consecutive_write_errors}, "
            f"reconnect_retry_attempts={self.reconnect_retry_attempts}, "
            f"reconnect_retry_interval_sec={self.reconnect_retry_interval_sec:.2f}, "
            f"reconnect_retry_backoff_sec={self.reconnect_retry_backoff_sec:.2f}"
        )

    def _open_serial(self, port: str, baud_rate: int, timeout_sec: float):
        """打开串口并返回串口对象。"""
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud_rate,
                bytesize=8,
                stopbits=1,
                parity=serial.PARITY_NONE,
                timeout=timeout_sec,
            )
            self.get_logger().info(f"Serial opened: {port}, baud={baud_rate}, timeout={timeout_sec}s")
            return ser
        except Exception as e:
            self.get_logger().error(f"Open serial failed: {e}")
            return None

    def _close_serial_quietly(self, ser=None):
        target = self.ser if ser is None else ser
        if target is None:
            return
        try:
            if target.is_open:
                target.close()
        except Exception as e:
            self.get_logger().warn(f"Close serial failed: {e}")
        finally:
            if ser is None:
                self.ser = None

    def _modbus_crc16(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return struct.pack("<H", crc)

    def _check_crc(self, frame: bytes) -> bool:
        if len(frame) < 3:
            return False
        return frame[-2:] == self._modbus_crc16(frame[:-2])

    def _read_exactly_from(self, ser, size: int) -> bytes:
        if ser is None or not ser.is_open:
            return b""
        data = ser.read(size)
        return data if len(data) == size else b""

    def _write_modbus_register_on(self, ser, slave_addr: int, reg_addr: int, value: int, attempts=None) -> bool:
        """向指定串口对象写单寄存器，支持有限次重试。"""
        if ser is None or not ser.is_open:
            self.get_logger().error("Serial is not open, cannot send command")
            return False

        total_attempts = max(1, int(attempts if attempts is not None else (self.write_retry_count + 1)))
        cmd_frame = bytes(
            [
                slave_addr & 0xFF,
                0x06,
                (reg_addr >> 8) & 0xFF,
                reg_addr & 0xFF,
                (value >> 8) & 0xFF,
                value & 0xFF,
            ]
        )
        cmd = cmd_frame + self._modbus_crc16(cmd_frame)
        last_issue = "unknown error"

        for attempt in range(1, total_attempts + 1):
            try:
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                try:
                    ser.reset_output_buffer()
                except Exception:
                    pass
                ser.write(cmd)
                resp = self._read_exactly_from(ser, 8)
                if resp == cmd:
                    return True
                if resp and not self._check_crc(resp):
                    last_issue = f"bad crc resp={resp.hex()}"
                elif resp:
                    last_issue = f"bad echo expect={cmd.hex()} resp={resp.hex()}"
                else:
                    last_issue = "no response"
            except Exception as e:
                last_issue = str(e)

            if attempt < total_attempts:
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                try:
                    ser.reset_output_buffer()
                except Exception:
                    pass
                self.get_logger().warn(
                    f"modbus write retry {attempt}/{total_attempts - 1}: slave={slave_addr}, reg=0x{reg_addr:04X}, issue={last_issue}"
                )

        self.get_logger().error(
            f"modbus write failed after {total_attempts} attempts: slave={slave_addr}, reg=0x{reg_addr:04X}, issue={last_issue}"
        )
        return False

    def _read_holding_register(self, slave_addr: int, reg_addr: int):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("Serial is not open, cannot read register")
            return None

        cmd_frame = bytes(
            [
                slave_addr & 0xFF,
                0x03,
                (reg_addr >> 8) & 0xFF,
                reg_addr & 0xFF,
                0x00,
                0x01,
            ]
        )
        cmd = cmd_frame + self._modbus_crc16(cmd_frame)

        try:
            self.ser.write(cmd)
            resp = self._read_exactly_from(self.ser, 7)
            if not resp:
                self.get_logger().warn(
                    f"modbus no response on read: slave={slave_addr}, reg=0x{reg_addr:04X}"
                )
                return None
            if not self._check_crc(resp):
                self.get_logger().warn(
                    f"modbus read bad crc: slave={slave_addr}, reg=0x{reg_addr:04X}, resp={resp.hex()}"
                )
                return None
            if resp[0] != (slave_addr & 0xFF) or resp[1] != 0x03 or resp[2] != 0x02:
                self.get_logger().warn(
                    f"modbus read bad response: slave={slave_addr}, reg=0x{reg_addr:04X}, resp={resp.hex()}"
                )
                return None
            return (resp[3] << 8) | resp[4]
        except Exception as e:
            self.get_logger().error(f"modbus read failed: {e}")
            return None

    def _send_stop_to_serial(self, ser, left_slave: int, right_slave: int) -> bool:
        """向指定串口的左右驱动发送停机命令。"""
        left_speed_ok = self._write_modbus_register_on(ser, left_slave, self.REG_SPEED, 0, attempts=1)
        left_mode_ok = self._write_modbus_register_on(ser, left_slave, self.REG_MODE, self.MODE_STOP, attempts=1)
        right_speed_ok = self._write_modbus_register_on(ser, right_slave, self.REG_SPEED, 0, attempts=1)
        right_mode_ok = self._write_modbus_register_on(ser, right_slave, self.REG_MODE, self.MODE_STOP, attempts=1)
        left_ok = left_speed_ok and left_mode_ok
        right_ok = right_speed_ok and right_mode_ok
        return bool(left_ok and right_ok)

    def _close_serial(self, ser, left_slave: int, right_slave: int):
        """关闭指定串口，关闭前尽量先停机。"""
        if ser is None:
            return
        try:
            if ser.is_open:
                self._send_stop_to_serial(ser, left_slave, right_slave)
                ser.close()
        except Exception as e:
            self.get_logger().warn(f"Close serial failed: {e}")

    def _init_driver_on(self, ser, slave_addr: int) -> bool:
        ok = self._write_modbus_register_on(ser, slave_addr, self.REG_MODE, self.MODE_STOP, attempts=1)
        if ok:
            self.get_logger().info(f"Driver init success, slave={slave_addr}")
        else:
            self.get_logger().error(f"Driver init failed, slave={slave_addr}")
        return ok

    def _init_driver_pair_on(self, ser, left_slave: int, right_slave: int) -> bool:
        return bool(
            self._init_driver_on(ser, left_slave)
            and self._init_driver_on(ser, right_slave)
        )

    def _init_driver(self, slave_addr: int) -> bool:
        return self._init_driver_on(self.ser, slave_addr)

    def _init_driver_pair(self, left_slave: int, right_slave: int) -> bool:
        return self._init_driver_pair_on(self.ser, left_slave, right_slave)

    def _speed_to_mode_and_value(self, speed_rpm: float):
        v = int(round(speed_rpm))
        if v == 0:
            return self.MODE_STOP, 0

        sign = 1 if v > 0 else -1
        magnitude = abs(v)
        magnitude = max(self.min_speed_rpm, min(self.max_speed_rpm, magnitude))
        mode = self.MODE_RUN_FWD if sign > 0 else self.MODE_RUN_REV
        return mode, magnitude

    def _apply_motor_command(self, slave_addr: int, speed_rpm: float):
        """对单个电机应用一条速度命令。"""
        mode, value = self._speed_to_mode_and_value(speed_rpm)
        if mode == self.MODE_STOP:
            speed_ok = self._write_modbus_register_on(self.ser, slave_addr, self.REG_SPEED, 0)
            if not speed_ok:
                return False
            return self._write_modbus_register_on(self.ser, slave_addr, self.REG_MODE, self.MODE_STOP)

        speed_ok = self._write_modbus_register_on(self.ser, slave_addr, self.REG_SPEED, value)
        if not speed_ok:
            return False
        if self.auto_start:
            return self._write_modbus_register_on(self.ser, slave_addr, self.REG_MODE, mode)
        return True

    def _try_stop_motors(self, reason: str) -> bool:
        """尽量让左右驱动都停机。"""
        ok = self._send_stop_to_serial(self.ser, self.left_slave, self.right_slave)
        if ok:
            self.get_logger().warn(f"motors stopped: {reason}")
            self._publish_speed_status(0.0, 0.0)
        else:
            self.get_logger().error(f"failed to stop motors cleanly: {reason}")
        self.last_command_was_stop = True
        self.last_speed_cmd_stamp = None
        return ok

    def _enter_fault(self, reason: str):
        """进入故障锁定，拒绝继续下发速度直到人工复位。"""
        if self.driver_faulted:
            return
        self.driver_faulted = True
        self.fault_reason = str(reason)
        self.get_logger().error(f"driver fault latched: {self.fault_reason}")
        self._try_stop_motors(f"fault latch: {self.fault_reason}")

    def _clear_fault(self):
        """人工复位故障锁定。"""
        self.driver_faulted = False
        self.fault_reason = ""
        self.write_error_streak = 0
        self.last_speed_cmd_stamp = None
        self.last_command_was_stop = True
        self.get_logger().warn("driver fault reset by operator")
        self._next_reconnect_monotonic = 0.0

    def _mark_driver_unavailable(self, reason: str, immediate: bool = False) -> None:
        self.driver_available = False
        self._last_reconnect_reason = str(reason)
        self.last_speed_cmd_stamp = None
        self.last_command_was_stop = True
        self._close_serial_quietly()
        self._publish_speed_status(0.0, 0.0)
        if immediate:
            self._next_reconnect_monotonic = 0.0

    def _recover_driver(self, force: bool = False, reason: str = "runtime") -> bool:
        if self.driver_faulted:
            return False

        now = time.monotonic()
        if self.driver_available and self.ser is not None and self.ser.is_open:
            return True
        if not force and now < self._next_reconnect_monotonic:
            return False

        self._close_serial_quietly()
        last_issue = "unknown error"
        for attempt in range(1, self.reconnect_retry_attempts + 1):
            candidate = self._open_serial(self.serial_port, self.baud_rate, self.serial_timeout_sec)
            if candidate is not None:
                if self._init_driver_pair_on(candidate, self.left_slave, self.right_slave):
                    self.ser = candidate
                    self.driver_available = True
                    self.write_error_streak = 0
                    self._next_reconnect_monotonic = 0.0
                    self._last_reconnect_reason = ""
                    self.get_logger().warn(
                        f"driver connection recovered after {attempt}/{self.reconnect_retry_attempts} attempts ({reason})"
                    )
                    return True
                last_issue = "driver init failed"
                self._close_serial_quietly(candidate)
            else:
                last_issue = "open serial failed"

            if attempt < self.reconnect_retry_attempts:
                time.sleep(self.reconnect_retry_interval_sec)

        self.driver_available = False
        self._next_reconnect_monotonic = time.monotonic() + self.reconnect_retry_backoff_sec
        self._last_reconnect_reason = f"{reason}: {last_issue}"
        self.get_logger().error(
            f"driver recovery failed after {self.reconnect_retry_attempts} attempts ({reason}); "
            f"retry after {self.reconnect_retry_backoff_sec:.1f}s"
        )
        return False

    def _record_write_failure(self, reason: str):
        """记录下发失败，并在必要时停机后尝试重连。"""
        self.write_error_streak += 1
        self.get_logger().error(
            f"driver write failure streak={self.write_error_streak}/{self.max_consecutive_write_errors}: {reason}"
        )
        if self.fail_safe_on_write_error:
            self._try_stop_motors(f"write failure: {reason}")
        if self.write_error_streak >= self.max_consecutive_write_errors:
            self._mark_driver_unavailable(reason, immediate=True)
            self._recover_driver(force=True, reason=f"write failure: {reason}")

    def _on_command_timeout_check(self):
        """如果驱动节点长时间没收到新速度命令，则主动停车。"""
        override_expired = self._refresh_override_active()
        if override_expired:
            self._apply_selected_command()
        if not self.driver_available:
            self._recover_driver(reason=self._last_reconnect_reason or "periodic reconnect")
            return
        if self.driver_faulted or self.last_speed_cmd_stamp is None or self.last_command_was_stop:
            return

        elapsed = (self.get_clock().now() - self.last_speed_cmd_stamp).nanoseconds / 1e9
        if elapsed > self.driver_command_timeout_sec:
            self._try_stop_motors(f"driver command timeout ({elapsed:.2f}s)")

    def _on_set_parameters(self, params):
        """允许运行时修改限幅、超时、重试和串口参数。"""
        static_params = {"speed_topic", "override_speed_topic", "override_active_topic", "speed_status_topic"}
        next_values = {
            "serial_port": self.serial_port,
            "baud_rate": self.baud_rate,
            "serial_timeout_sec": self.serial_timeout_sec,
            "left_slave": self.left_slave,
            "right_slave": self.right_slave,
            "min_speed_rpm": self.min_speed_rpm,
            "max_speed_rpm": self.max_speed_rpm,
            "auto_start": self.auto_start,
            "driver_command_timeout_sec": self.driver_command_timeout_sec,
            "write_retry_count": self.write_retry_count,
            "fail_safe_on_write_error": self.fail_safe_on_write_error,
            "max_consecutive_write_errors": self.max_consecutive_write_errors,
            "override_active_timeout_sec": self.override_active_timeout_sec,
            "fault_reset_counter": self.fault_reset_counter,
        }

        for param in params:
            if param.name in static_params:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} requires node restart because subscription is already created",
                )
            next_values[param.name] = param.value

        try:
            next_values["serial_port"] = str(next_values["serial_port"])
            next_values["baud_rate"] = int(next_values["baud_rate"])
            next_values["serial_timeout_sec"] = max(0.01, float(next_values["serial_timeout_sec"]))
            next_values["left_slave"] = int(next_values["left_slave"])
            next_values["right_slave"] = int(next_values["right_slave"])
            next_values["min_speed_rpm"] = max(0, int(next_values["min_speed_rpm"]))
            next_values["max_speed_rpm"] = max(next_values["min_speed_rpm"], int(next_values["max_speed_rpm"]))
            next_values["auto_start"] = bool(next_values["auto_start"])
            next_values["driver_command_timeout_sec"] = max(0.05, float(next_values["driver_command_timeout_sec"]))
            next_values["write_retry_count"] = max(0, int(next_values["write_retry_count"]))
            next_values["fail_safe_on_write_error"] = bool(next_values["fail_safe_on_write_error"])
            next_values["max_consecutive_write_errors"] = max(1, int(next_values["max_consecutive_write_errors"]))
            next_values["override_active_timeout_sec"] = max(
                0.05, float(next_values["override_active_timeout_sec"])
            )
            next_values["fault_reset_counter"] = int(next_values["fault_reset_counter"])
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        reopen_needed = any(
            next_values[key] != getattr(self, key)
            for key in ("serial_port", "baud_rate", "serial_timeout_sec")
        )
        slave_changed = any(
            next_values[key] != getattr(self, key)
            for key in ("left_slave", "right_slave")
        )
        reset_requested = next_values["fault_reset_counter"] > self.fault_reset_counter
        candidate_ser = None

        if reopen_needed:
            candidate_ser = self._open_serial(
                next_values["serial_port"],
                next_values["baud_rate"],
                next_values["serial_timeout_sec"],
            )
            if candidate_ser is None:
                return SetParametersResult(successful=False, reason="failed to reopen serial with new parameters")

        old_ser = self.ser
        old_left_slave = self.left_slave
        old_right_slave = self.right_slave

        if reopen_needed:
            prev_ser = self.ser
            self.ser = candidate_ser
            init_ok = self._init_driver_pair(next_values["left_slave"], next_values["right_slave"])
            if not init_ok:
                self.ser = prev_ser
                self._close_serial(candidate_ser, next_values["left_slave"], next_values["right_slave"])
                return SetParametersResult(successful=False, reason="new serial opened but driver init failed")
        elif slave_changed and self.ser is not None and self.ser.is_open:
            prev_left = self.left_slave
            prev_right = self.right_slave
            self.left_slave = next_values["left_slave"]
            self.right_slave = next_values["right_slave"]
            init_ok = self._init_driver_pair(self.left_slave, self.right_slave)
            if not init_ok:
                self.left_slave = prev_left
                self.right_slave = prev_right
                return SetParametersResult(successful=False, reason="driver init failed for new slave ids")

        self.serial_port = next_values["serial_port"]
        self.baud_rate = next_values["baud_rate"]
        self.serial_timeout_sec = next_values["serial_timeout_sec"]
        self.left_slave = next_values["left_slave"]
        self.right_slave = next_values["right_slave"]
        self.min_speed_rpm = next_values["min_speed_rpm"]
        self.max_speed_rpm = next_values["max_speed_rpm"]
        self.auto_start = next_values["auto_start"]
        self.driver_command_timeout_sec = next_values["driver_command_timeout_sec"]
        self.write_retry_count = next_values["write_retry_count"]
        self.fail_safe_on_write_error = next_values["fail_safe_on_write_error"]
        self.max_consecutive_write_errors = next_values["max_consecutive_write_errors"]
        self.override_active_timeout_sec = next_values["override_active_timeout_sec"]
        self.fault_reset_counter = next_values["fault_reset_counter"]

        if reopen_needed:
            self._close_serial(old_ser, old_left_slave, old_right_slave)

        if reset_requested:
            self._clear_fault()
            self._recover_driver(force=True, reason="fault reset")

        if params:
            changed = ", ".join(sorted(param.name for param in params))
            self.get_logger().info(f"runtime parameters updated: {changed}")
        return SetParametersResult(successful=True)

    def _extract_command(self, msg: Float32MultiArray, topic_name: str):
        if len(msg.data) != 2:
            self.get_logger().warn(f"Invalid {topic_name}, expect [left_rpm, right_rpm]")
            return None
        return float(msg.data[0]), float(msg.data[1])

    def _publish_speed_status(self, left_rpm: float, right_rpm: float):
        msg = Float32MultiArray()
        msg.data = [float(left_rpm), float(right_rpm)]
        self.speed_status_pub.publish(msg)

    def _refresh_override_active(self):
        if not self.override_active or self.override_active_stamp is None:
            return False
        elapsed = (self.get_clock().now() - self.override_active_stamp).nanoseconds / 1e9
        if elapsed > self.override_active_timeout_sec:
            self.override_active = False
            self.override_command = (0.0, 0.0)
            self.override_command_stamp = None
            return True
        return False

    def _apply_command_pair(self, left_rpm: float, right_rpm: float, source: str):
        if self.driver_faulted:
            self.get_logger().error(f"driver fault latched, ignore speed command until reset: {self.fault_reason}")
            return
        if not self.driver_available and not self._recover_driver(reason=f"command from {source}"):
            return

        left_ok = self._apply_motor_command(self.left_slave, left_rpm)
        right_ok = self._apply_motor_command(self.right_slave, right_rpm)

        if left_ok and right_ok:
            self.write_error_streak = 0
            self.last_speed_cmd_stamp = self.get_clock().now()
            self.last_command_was_stop = abs(left_rpm) < 1e-6 and abs(right_rpm) < 1e-6
            self._publish_speed_status(left_rpm, right_rpm)
            return

        self._record_write_failure(
            f"source={source}, left_ok={left_ok}, right_ok={right_ok}, left_rpm={left_rpm:.1f}, right_rpm={right_rpm:.1f}"
        )

    def _apply_selected_command(self):
        self._refresh_override_active()
        if self.override_active:
            left_rpm, right_rpm = self.override_command
            source = "override"
        else:
            left_rpm, right_rpm = self.base_command
            source = "base"
        self._apply_command_pair(left_rpm, right_rpm, source)

    def on_base_speed_cmd(self, msg: Float32MultiArray):
        cmd = self._extract_command(msg, self.speed_topic)
        if cmd is None:
            return
        self.base_command = cmd
        if not self.override_active:
            self._apply_selected_command()

    def on_override_speed_cmd(self, msg: Float32MultiArray):
        cmd = self._extract_command(msg, self.override_speed_topic)
        if cmd is None:
            return
        self.override_command = cmd
        self.override_command_stamp = self.get_clock().now()
        if self.override_active:
            self._apply_selected_command()

    def on_override_active(self, msg: Bool):
        active = bool(msg.data)
        self.override_active = active
        self.override_active_stamp = self.get_clock().now()
        if not active:
            self.override_command = (0.0, 0.0)
            self.override_command_stamp = None
        self._apply_selected_command()

    def destroy_node(self):
        """节点退出时确保停机并关闭串口。"""
        try:
            if self.ser is not None and self.ser.is_open:
                self.get_logger().info("Stopping motors before shutdown")
                self._try_stop_motors("shutdown")
                self.ser.close()
        except Exception as e:
            self.get_logger().warn(f"Cleanup failed: {e}")

        super().destroy_node()


def main():
    """节点入口。"""
    rclpy.init()
    node = MotorDriverControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
