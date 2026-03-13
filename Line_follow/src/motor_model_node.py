#!/usr/bin/env python3
"""视觉角度到电机差速的运动模型节点。"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class LineFollowMotorModelNode(Node):
    """将视觉角度映射为左右电机差额转速。"""

    def __init__(self):
        super().__init__("line_follow_motor_model_node")

        self.declare_parameter("angle_topic", "/line_follow/line_angle_deg")
        self.declare_parameter("speed_topic", "/motor_speed_cmd")
        self.declare_parameter("publish_debug_topic", True)
        self.declare_parameter("max_visual_angle_deg", 45.0)
        self.declare_parameter("heading_gain", 1.8)
        self.declare_parameter("base_speed_ratio", 0.32)
        self.declare_parameter("speed_reduce_gain", 0.55)
        self.declare_parameter("min_output_rpm", 18.0)
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("track_width_m", 0.577)
        self.declare_parameter("drive_wheel_diameter_m", 0.18)
        self.declare_parameter("track_pitch_m", 0.06)
        self.declare_parameter("track_link_count", 24)
        self.declare_parameter("rated_voltage_v", 48.0)
        self.declare_parameter("rated_power_w", 400.0)
        self.declare_parameter("rated_current_a", 11.0)
        self.declare_parameter("rated_motor_rpm", 3000.0)
        self.declare_parameter("rated_motor_torque_nm", 1.27)
        self.declare_parameter("no_load_current_a", 2.0)
        self.declare_parameter("no_load_motor_rpm", 3300.0)
        self.declare_parameter("rated_output_torque_nm", 14.0)

        self.angle_topic = str(self.get_parameter("angle_topic").value)
        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.publish_debug_topic = bool(self.get_parameter("publish_debug_topic").value)
        self.max_visual_angle_deg = max(1.0, float(self.get_parameter("max_visual_angle_deg").value))
        self.heading_gain = max(0.0, float(self.get_parameter("heading_gain").value))
        self.base_speed_ratio = min(1.0, max(0.0, float(self.get_parameter("base_speed_ratio").value)))
        self.speed_reduce_gain = min(1.0, max(0.0, float(self.get_parameter("speed_reduce_gain").value)))
        self.min_output_rpm = max(0.0, float(self.get_parameter("min_output_rpm").value))
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.command_timeout_sec = max(0.1, float(self.get_parameter("command_timeout_sec").value))
        self.track_width_m = max(0.05, float(self.get_parameter("track_width_m").value))
        self.drive_wheel_diameter_m = max(0.01, float(self.get_parameter("drive_wheel_diameter_m").value))
        self.track_pitch_m = max(0.001, float(self.get_parameter("track_pitch_m").value))
        self.track_link_count = max(1, int(self.get_parameter("track_link_count").value))
        self.rated_motor_rpm = max(1.0, float(self.get_parameter("rated_motor_rpm").value))
        self.rated_motor_torque_nm = max(0.01, float(self.get_parameter("rated_motor_torque_nm").value))
        self.no_load_motor_rpm = max(self.rated_motor_rpm, float(self.get_parameter("no_load_motor_rpm").value))
        self.rated_output_torque_nm = max(0.01, float(self.get_parameter("rated_output_torque_nm").value))

        self.gear_ratio = self.rated_output_torque_nm / self.rated_motor_torque_nm
        self.wheel_circumference_m = math.pi * self.drive_wheel_diameter_m
        self.track_loop_length_m = self.track_pitch_m * float(self.track_link_count)
        self.rated_output_rpm = self.rated_motor_rpm / self.gear_ratio
        self.no_load_output_rpm = self.no_load_motor_rpm / self.gear_ratio
        self.last_angle_stamp = None

        self.speed_pub = self.create_publisher(Float32MultiArray, self.speed_topic, 10)
        self.debug_center_pub = None
        self.debug_delta_pub = None
        if self.publish_debug_topic:
            self.debug_center_pub = self.create_publisher(Float32, "/line_follow/center_motor_rpm", 10)
            self.debug_delta_pub = self.create_publisher(Float32, "/line_follow/delta_motor_rpm", 10)

        self.sub = self.create_subscription(Float32, self.angle_topic, self.on_angle, 10)
        self.timeout_timer = self.create_timer(0.1, self.on_timeout_check)

        self.get_logger().info(
            "motor model ready: "
            f"angle_topic={self.angle_topic}, speed_topic={self.speed_topic}, "
            f"gear_ratio={self.gear_ratio:.2f}, rated_output_rpm={self.rated_output_rpm:.1f}, "
            f"track_width_m={self.track_width_m:.3f}, wheel_diameter_m={self.drive_wheel_diameter_m:.3f}, "
            f"allow_reverse={self.allow_reverse}"
        )

    def angle_to_motor_rpm(self, angle_deg: float):
        angle_deg = max(-self.max_visual_angle_deg, min(self.max_visual_angle_deg, angle_deg))
        angle_rad = math.radians(angle_deg)
        angle_norm = min(1.0, abs(angle_deg) / self.max_visual_angle_deg)

        center_output_rpm = self.rated_output_rpm * self.base_speed_ratio * (1.0 - self.speed_reduce_gain * angle_norm)
        center_output_rpm = max(self.min_output_rpm, center_output_rpm)

        target_yaw_rate = -self.heading_gain * angle_rad
        delta_output_rpm = target_yaw_rate * self.track_width_m * 30.0 / self.wheel_circumference_m

        left_output_rpm = center_output_rpm - delta_output_rpm
        right_output_rpm = center_output_rpm + delta_output_rpm

        max_output_rpm = self.no_load_output_rpm * 0.92
        if self.allow_reverse:
            left_output_rpm = max(-max_output_rpm, min(max_output_rpm, left_output_rpm))
            right_output_rpm = max(-max_output_rpm, min(max_output_rpm, right_output_rpm))
        else:
            delta_output_rpm = max(-center_output_rpm, min(center_output_rpm, delta_output_rpm))
            left_output_rpm = max(0.0, min(max_output_rpm, center_output_rpm - delta_output_rpm))
            right_output_rpm = max(0.0, min(max_output_rpm, center_output_rpm + delta_output_rpm))

        left_motor_rpm = left_output_rpm * self.gear_ratio
        right_motor_rpm = right_output_rpm * self.gear_ratio
        center_motor_rpm = 0.5 * (left_motor_rpm + right_motor_rpm)
        delta_motor_rpm = 0.5 * (right_motor_rpm - left_motor_rpm)
        return left_motor_rpm, right_motor_rpm, center_motor_rpm, delta_motor_rpm

    def publish_speed(self, left_rpm: float, right_rpm: float):
        msg = Float32MultiArray()
        msg.data = [float(left_rpm), float(right_rpm)]
        self.speed_pub.publish(msg)

    def on_angle(self, msg: Float32):
        angle_deg = float(msg.data)
        left_rpm, right_rpm, center_motor_rpm, delta_motor_rpm = self.angle_to_motor_rpm(angle_deg)
        self.publish_speed(left_rpm, right_rpm)
        self.last_angle_stamp = self.get_clock().now()

        if self.debug_center_pub is not None:
            debug_msg = Float32()
            debug_msg.data = float(center_motor_rpm)
            self.debug_center_pub.publish(debug_msg)

        if self.debug_delta_pub is not None:
            debug_msg = Float32()
            debug_msg.data = float(delta_motor_rpm)
            self.debug_delta_pub.publish(debug_msg)

    def on_timeout_check(self):
        if self.last_angle_stamp is None:
            return

        elapsed = (self.get_clock().now() - self.last_angle_stamp).nanoseconds / 1e9
        if elapsed > self.command_timeout_sec:
            self.publish_speed(0.0, 0.0)
            self.last_angle_stamp = None
            self.get_logger().warn(
                f"angle command timeout ({elapsed:.2f}s), publish stop to {self.speed_topic}"
            )


def main():
    rclpy.init()
    node = LineFollowMotorModelNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_speed(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
