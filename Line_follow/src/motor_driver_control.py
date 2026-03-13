#!/usr/bin/env python3
"""电机驱动控制节点。"""

import struct

import rclpy
from rclpy.node import Node
import serial
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
        self.declare_parameter("left_slave", 6)
        self.declare_parameter("right_slave", 8)
        self.declare_parameter("min_speed_rpm", 0)
        self.declare_parameter("max_speed_rpm", 5000)
        self.declare_parameter("auto_start", True)

        self.left_slave = int(self.get_parameter("left_slave").value)
        self.right_slave = int(self.get_parameter("right_slave").value)
        self.min_speed_rpm = int(self.get_parameter("min_speed_rpm").value)
        self.max_speed_rpm = int(self.get_parameter("max_speed_rpm").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)

        serial_port = str(self.get_parameter("serial_port").value)
        baud_rate = int(self.get_parameter("baud_rate").value)
        timeout_sec = float(self.get_parameter("serial_timeout_sec").value)
        self.ser = self._open_serial(serial_port, baud_rate, timeout_sec)

        if self.ser is not None:
            self._init_driver(self.left_slave)
            self._init_driver(self.right_slave)

        speed_topic = str(self.get_parameter("speed_topic").value)
        self.sub = self.create_subscription(Float32MultiArray, speed_topic, self.on_speed_cmd, 10)
        self.get_logger().info(
            f"Subscribed to {speed_topic}, driver left={self.left_slave}, right={self.right_slave}"
        )

    def _open_serial(self, port: str, baud_rate: int, timeout_sec: float):
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

    def _read_exactly(self, size: int) -> bytes:
        if self.ser is None or not self.ser.is_open:
            return b""
        data = self.ser.read(size)
        return data if len(data) == size else b""

    def _write_modbus_register(self, slave_addr: int, reg_addr: int, value: int) -> bool:
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("Serial is not open, cannot send command")
            return False

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

        try:
            self.ser.write(cmd)
            resp = self._read_exactly(8)
            if resp == cmd:
                return True
            if resp and not self._check_crc(resp):
                self.get_logger().warn(
                    f"modbus bad crc: slave={slave_addr}, reg=0x{reg_addr:04X}, resp={resp.hex()}"
                )
                return False
            if resp:
                self.get_logger().warn(
                    f"modbus bad echo: slave={slave_addr}, reg=0x{reg_addr:04X}, "
                    f"expect={cmd.hex()}, resp={resp.hex()}"
                )
                return False
            self.get_logger().warn(
                f"modbus no response: slave={slave_addr}, reg=0x{reg_addr:04X}, value=0x{value:04X}"
            )
            return False
        except Exception as e:
            self.get_logger().error(f"modbus write failed: {e}")
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
            resp = self._read_exactly(7)
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

    def _init_driver(self, slave_addr: int):
        ok = self._write_modbus_register(slave_addr, self.REG_MODE, self.MODE_STOP)
        if ok:
            self.get_logger().info(f"Driver init success, slave={slave_addr}")
        else:
            self.get_logger().error(f"Driver init failed, slave={slave_addr}")

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
        mode, value = self._speed_to_mode_and_value(speed_rpm)
        if mode == self.MODE_STOP:
            self._write_modbus_register(slave_addr, self.REG_MODE, self.MODE_STOP)
            return

        self._write_modbus_register(slave_addr, self.REG_SPEED, value)
        if self.auto_start:
            self._write_modbus_register(slave_addr, self.REG_MODE, mode)

    def on_speed_cmd(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn("Invalid /motor_speed_cmd, expect [left_rpm, right_rpm]")
            return

        self._apply_motor_command(self.left_slave, float(msg.data[0]))
        self._apply_motor_command(self.right_slave, float(msg.data[1]))

    def _stop_motors(self):
        self._write_modbus_register(self.left_slave, self.REG_MODE, self.MODE_STOP)
        self._write_modbus_register(self.right_slave, self.REG_MODE, self.MODE_STOP)

    def destroy_node(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.get_logger().info("Stopping motors before shutdown")
                self._stop_motors()
                self.ser.close()
        except Exception as e:
            self.get_logger().warn(f"Cleanup failed: {e}")

        super().destroy_node()


def main():
    rclpy.init()
    node = MotorDriverControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
