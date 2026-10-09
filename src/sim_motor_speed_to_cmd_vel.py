#!/usr/bin/env python3
"""Bridge motor RPM commands to cmd_vel for Gazebo simulation."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SimMotorSpeedToCmdVel(Node):
    """Convert signed left/right motor RPM commands into a Twist command."""

    def __init__(self):
        super().__init__("sim_motor_speed_to_cmd_vel_node")

        self.declare_parameter("speed_topic", "/motor_speed_cmd")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("track_width_m", 0.577)
        self.declare_parameter("drive_wheel_diameter_m", 0.18)
        self.declare_parameter("gear_ratio", 14.0 / 1.27)
        self.declare_parameter("left_motor_sign", 1.0)
        self.declare_parameter("right_motor_sign", -1.0)
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("max_linear_mps", 1.2)
        self.declare_parameter("max_angular_radps", 3.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.track_width_m = max(0.05, float(self.get_parameter("track_width_m").value))
        self.drive_wheel_diameter_m = max(
            0.01,
            float(self.get_parameter("drive_wheel_diameter_m").value),
        )
        self.gear_ratio = max(1.0, float(self.get_parameter("gear_ratio").value))
        self.left_motor_sign = self._sign(self.get_parameter("left_motor_sign").value)
        self.right_motor_sign = self._sign(self.get_parameter("right_motor_sign").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.max_linear_mps = max(0.0, float(self.get_parameter("max_linear_mps").value))
        self.max_angular_radps = max(
            0.0,
            float(self.get_parameter("max_angular_radps").value),
        )
        self.command_timeout_sec = max(
            0.05,
            float(self.get_parameter("command_timeout_sec").value),
        )
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        self.latest_twist = Twist()
        self.last_command_time: Optional[rclpy.time.Time] = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.speed_sub = self.create_subscription(
            Float32MultiArray,
            self.speed_topic,
            self.on_speed_command,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.on_timer)

        self.get_logger().info(
            "sim motor-speed bridge ready: "
            f"speed_topic={self.speed_topic}, cmd_vel_topic={self.cmd_vel_topic}, "
            f"track_width_m={self.track_width_m:.3f}, "
            f"wheel_diameter_m={self.drive_wheel_diameter_m:.3f}, "
            f"gear_ratio={self.gear_ratio:.2f}, "
            f"left_motor_sign={self.left_motor_sign:.0f}, "
            f"right_motor_sign={self.right_motor_sign:.0f}"
        )

    @staticmethod
    def _sign(value) -> float:
        return 1.0 if float(value) >= 0.0 else -1.0

    @staticmethod
    def _finite(value: float, default: float = 0.0) -> float:
        try:
            value = float(value)
        except (TypeError, ValueError):
            return default
        return value if math.isfinite(value) else default

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if limit <= 0.0:
            return value
        return max(-limit, min(limit, value))

    def _motor_rpm_to_track_mps(self, motor_rpm: float) -> float:
        output_rpm = motor_rpm / self.gear_ratio
        wheel_circumference_m = math.pi * self.drive_wheel_diameter_m
        return output_rpm * wheel_circumference_m / 60.0

    def on_speed_command(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warn("invalid motor speed command, expected [left_rpm, right_rpm]")
            return

        signed_left_rpm = self._finite(msg.data[0])
        signed_right_rpm = self._finite(msg.data[1])
        left_rpm = signed_left_rpm * self.left_motor_sign
        right_rpm = signed_right_rpm * self.right_motor_sign

        left_mps = self._motor_rpm_to_track_mps(left_rpm)
        right_mps = self._motor_rpm_to_track_mps(right_rpm)
        linear_x = 0.5 * (left_mps + right_mps) * self.linear_scale
        angular_z = ((right_mps - left_mps) / self.track_width_m) * self.angular_scale

        twist = Twist()
        twist.linear.x = self._clamp(linear_x, self.max_linear_mps)
        twist.angular.z = self._clamp(angular_z, self.max_angular_radps)
        self.latest_twist = twist
        self.last_command_time = self.get_clock().now()

    def on_timer(self) -> None:
        twist = self.latest_twist
        if self.last_command_time is None:
            twist = Twist()
        else:
            age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
            if age > self.command_timeout_sec:
                twist = Twist()
                self.latest_twist = twist
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SimMotorSpeedToCmdVel()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
