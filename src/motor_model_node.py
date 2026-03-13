#!/usr/bin/env python3
"""
视觉角度到电机差速的运动模型节点。

功能概述：
1. 订阅视觉节点发布的线角度 `/line_follow/line_angle_deg`
2. 订阅视觉节点发布的横向偏移 `/line_follow/line_offset_norm`
3. 根据航向误差和横向误差计算左右电机目标转速
4. 将结果发布到 `/motor_speed_cmd`
5. 允许根据参数选择“只前进差速”或“允许一侧反转”
6. 当视觉输入超时丢失时主动发布停车命令

模型核心：
- 视觉角度越大，底盘期望角速度越大
- 横向偏移越大，底盘也会附加一个回中修正量
- 综合修正量再通过履带中心距 `track_width_m` 换算成左右差速
- 结合驱动轮直径与减速比，将输出轴转速转换为电机侧 RPM
"""

import math

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class LineFollowMotorModelNode(Node):
    """将视觉角度映射为左右电机差额转速。"""

    def __init__(self):
        super().__init__("line_follow_motor_model_node")

        # 输入/输出话题。
        self.declare_parameter("angle_topic", "/line_follow/line_angle_deg")
        self.declare_parameter("offset_topic", "/line_follow/line_offset_norm")
        self.declare_parameter("speed_topic", "/motor_speed_cmd")
        self.declare_parameter("publish_debug_topic", True)

        # 控制参数：决定角度变化时底盘如何响应。
        self.declare_parameter("max_visual_angle_deg", 45.0)
        self.declare_parameter("heading_gain", 1.8)
        self.declare_parameter("lateral_gain", 18.0)
        self.declare_parameter("base_speed_ratio", 0.32)
        self.declare_parameter("speed_reduce_gain", 0.55)
        self.declare_parameter("min_output_rpm", 18.0)
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("offset_timeout_sec", 0.3)
        self.declare_parameter("angle_lowpass_alpha", 0.35)
        self.declare_parameter("angle_deadband_deg", 1.5)
        self.declare_parameter("max_motor_rpm_step_per_sec", 1200.0)

        # 底盘机械参数。
        self.declare_parameter("track_width_m", 0.577)
        self.declare_parameter("drive_wheel_diameter_m", 0.18)
        self.declare_parameter("track_pitch_m", 0.06)
        self.declare_parameter("track_link_count", 24)

        # 电机与减速系统参数。
        self.declare_parameter("rated_voltage_v", 48.0)
        self.declare_parameter("rated_power_w", 400.0)
        self.declare_parameter("rated_current_a", 11.0)
        self.declare_parameter("rated_motor_rpm", 3000.0)
        self.declare_parameter("rated_motor_torque_nm", 1.27)
        self.declare_parameter("no_load_current_a", 2.0)
        self.declare_parameter("no_load_motor_rpm", 3300.0)
        self.declare_parameter("rated_output_torque_nm", 14.0)
        self.declare_parameter("left_motor_sign", 1.0)
        self.declare_parameter("right_motor_sign", -1.0)

        self._load_static_topics()
        self._refresh_runtime_parameters()
        self.last_angle_stamp = None
        self.last_offset_stamp = None
        self.latest_offset_norm = 0.0
        self.filtered_angle_deg = None
        self.last_command_stamp = None
        self.last_left_rpm = 0.0
        self.last_right_rpm = 0.0

        # 主输出：左右电机目标速度。
        self.speed_pub = self.create_publisher(Float32MultiArray, self.speed_topic, 10)
        self.debug_center_pub = None
        self.debug_delta_pub = None
        if self.publish_debug_topic:
            # 额外调试输出，便于在现场观察“基础速度”和“差速量”。
            self.debug_center_pub = self.create_publisher(Float32, "/line_follow/center_motor_rpm", 10)
            self.debug_delta_pub = self.create_publisher(Float32, "/line_follow/delta_motor_rpm", 10)

        self.sub = self.create_subscription(Float32, self.angle_topic, self.on_angle, 10)
        self.offset_sub = self.create_subscription(Float32, self.offset_topic, self.on_offset, 10)

        # 定时器用于输入超时保护，避免相机或视觉异常时电机继续保持旧命令。
        self.timeout_timer = self.create_timer(0.1, self.on_timeout_check)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            "motor model ready: "
            f"angle_topic={self.angle_topic}, speed_topic={self.speed_topic}, "
            f"gear_ratio={self.gear_ratio:.2f}, rated_output_rpm={self.rated_output_rpm:.1f}, "
            f"track_width_m={self.track_width_m:.3f}, wheel_diameter_m={self.drive_wheel_diameter_m:.3f}, "
            f"allow_reverse={self.allow_reverse}, left_motor_sign={self.left_motor_sign:.0f}, "
            f"right_motor_sign={self.right_motor_sign:.0f}, offset_topic={self.offset_topic}, "
            f"lateral_gain={self.lateral_gain:.2f}, offset_timeout_sec={self.offset_timeout_sec:.2f}, "
            f"angle_lowpass_alpha={self.angle_lowpass_alpha:.2f}, angle_deadband_deg={self.angle_deadband_deg:.2f}, "
            f"max_motor_rpm_step_per_sec={self.max_motor_rpm_step_per_sec:.1f}"
        )

    def _load_static_topics(self):
        """加载需要在启动阶段固定下来的话题类参数。"""
        self.angle_topic = str(self.get_parameter("angle_topic").value)
        self.offset_topic = str(self.get_parameter("offset_topic").value)
        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.publish_debug_topic = bool(self.get_parameter("publish_debug_topic").value)

    def _refresh_runtime_parameters(self, overrides=None):
        """从参数服务器刷新当前控制/机械参数，支持运行时调参。"""
        values = {
            "max_visual_angle_deg": float(self.get_parameter("max_visual_angle_deg").value),
            "heading_gain": float(self.get_parameter("heading_gain").value),
            "lateral_gain": float(self.get_parameter("lateral_gain").value),
            "base_speed_ratio": float(self.get_parameter("base_speed_ratio").value),
            "speed_reduce_gain": float(self.get_parameter("speed_reduce_gain").value),
            "min_output_rpm": float(self.get_parameter("min_output_rpm").value),
            "allow_reverse": bool(self.get_parameter("allow_reverse").value),
            "command_timeout_sec": float(self.get_parameter("command_timeout_sec").value),
            "offset_timeout_sec": float(self.get_parameter("offset_timeout_sec").value),
            "angle_lowpass_alpha": float(self.get_parameter("angle_lowpass_alpha").value),
            "angle_deadband_deg": float(self.get_parameter("angle_deadband_deg").value),
            "max_motor_rpm_step_per_sec": float(self.get_parameter("max_motor_rpm_step_per_sec").value),
            "track_width_m": float(self.get_parameter("track_width_m").value),
            "drive_wheel_diameter_m": float(self.get_parameter("drive_wheel_diameter_m").value),
            "track_pitch_m": float(self.get_parameter("track_pitch_m").value),
            "track_link_count": int(self.get_parameter("track_link_count").value),
            "rated_motor_rpm": float(self.get_parameter("rated_motor_rpm").value),
            "rated_motor_torque_nm": float(self.get_parameter("rated_motor_torque_nm").value),
            "no_load_motor_rpm": float(self.get_parameter("no_load_motor_rpm").value),
            "rated_output_torque_nm": float(self.get_parameter("rated_output_torque_nm").value),
            "left_motor_sign": float(self.get_parameter("left_motor_sign").value),
            "right_motor_sign": float(self.get_parameter("right_motor_sign").value),
        }
        if overrides:
            values.update(overrides)

        self.max_visual_angle_deg = max(1.0, float(values["max_visual_angle_deg"]))
        self.heading_gain = max(0.0, float(values["heading_gain"]))
        self.lateral_gain = max(0.0, float(values["lateral_gain"]))
        self.base_speed_ratio = min(1.0, max(0.0, float(values["base_speed_ratio"])))
        self.speed_reduce_gain = min(1.0, max(0.0, float(values["speed_reduce_gain"])))
        self.min_output_rpm = max(0.0, float(values["min_output_rpm"]))
        self.allow_reverse = bool(values["allow_reverse"])
        self.command_timeout_sec = max(0.1, float(values["command_timeout_sec"]))
        self.offset_timeout_sec = max(0.0, float(values["offset_timeout_sec"]))
        self.angle_lowpass_alpha = min(1.0, max(0.0, float(values["angle_lowpass_alpha"])))
        self.angle_deadband_deg = max(0.0, float(values["angle_deadband_deg"]))
        self.max_motor_rpm_step_per_sec = max(0.0, float(values["max_motor_rpm_step_per_sec"]))
        self.track_width_m = max(0.05, float(values["track_width_m"]))
        self.drive_wheel_diameter_m = max(0.01, float(values["drive_wheel_diameter_m"]))
        self.track_pitch_m = max(0.001, float(values["track_pitch_m"]))
        self.track_link_count = max(1, int(values["track_link_count"]))
        self.rated_motor_rpm = max(1.0, float(values["rated_motor_rpm"]))
        self.rated_motor_torque_nm = max(0.01, float(values["rated_motor_torque_nm"]))
        self.no_load_motor_rpm = max(self.rated_motor_rpm, float(values["no_load_motor_rpm"]))
        self.rated_output_torque_nm = max(0.01, float(values["rated_output_torque_nm"]))
        self.left_motor_sign = 1.0 if float(values["left_motor_sign"]) >= 0.0 else -1.0
        self.right_motor_sign = 1.0 if float(values["right_motor_sign"]) >= 0.0 else -1.0

        # 这里使用“输出额定力矩 / 电机额定力矩”近似估算减速比。
        self.gear_ratio = self.rated_output_torque_nm / self.rated_motor_torque_nm
        self.wheel_circumference_m = math.pi * self.drive_wheel_diameter_m
        self.track_loop_length_m = self.track_pitch_m * float(self.track_link_count)
        self.rated_output_rpm = self.rated_motor_rpm / self.gear_ratio
        self.no_load_output_rpm = self.no_load_motor_rpm / self.gear_ratio

    def _on_set_parameters(self, params):
        """允许运行时刷新控制参数，话题结构类参数仍要求重启节点。"""
        static_params = {"angle_topic", "offset_topic", "speed_topic", "publish_debug_topic"}
        overrides = {}

        for param in params:
            if param.name in static_params:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} requires node restart because publishers/subscriptions are already created",
                )
            overrides[param.name] = param.value

        try:
            self._refresh_runtime_parameters(overrides)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if overrides:
            changed = ", ".join(sorted(overrides.keys()))
            self.get_logger().info(f"runtime parameters updated: {changed}")
        return SetParametersResult(successful=True)

    def angle_to_motor_rpm(self, angle_deg: float, offset_norm: float = 0.0):
        """
        将视觉角度和横向偏移转换为左右电机 RPM。

        返回：
        - 左电机 RPM
        - 右电机 RPM
        - 平均电机 RPM（调试用）
        - 差额电机 RPM（调试用）
        """
        # 先限制视觉输入角度，防止异常值让下游速度发散。
        angle_deg = max(-self.max_visual_angle_deg, min(self.max_visual_angle_deg, angle_deg))
        offset_norm = max(-1.0, min(1.0, float(offset_norm)))
        command_angle_deg = angle_deg + self.lateral_gain * offset_norm
        command_angle_deg = max(-self.max_visual_angle_deg, min(self.max_visual_angle_deg, command_angle_deg))
        angle_rad = math.radians(command_angle_deg)
        angle_norm = min(1.0, abs(command_angle_deg) / self.max_visual_angle_deg)

        # 角度越大，基础前进速度越低，避免急弯时仍高速前冲。
        center_output_rpm = self.rated_output_rpm * self.base_speed_ratio * (1.0 - self.speed_reduce_gain * angle_norm)
        center_output_rpm = max(self.min_output_rpm, center_output_rpm)

        # 将视觉角度映射为目标角速度，再换算成履带左右差速。
        target_yaw_rate = -self.heading_gain * angle_rad
        delta_output_rpm = target_yaw_rate * self.track_width_m * 30.0 / self.wheel_circumference_m

        # 当前底盘的左右履带转向与理想模型相反，这里交换差速方向。
        left_output_rpm = center_output_rpm + delta_output_rpm
        right_output_rpm = center_output_rpm - delta_output_rpm

        # 速度保护：不允许超过空载输出转速的 92%。
        max_output_rpm = self.no_load_output_rpm * 0.92
        if self.allow_reverse:
            # 允许一侧反转时，左右速度都允许进入负值区间。
            left_output_rpm = max(-max_output_rpm, min(max_output_rpm, left_output_rpm))
            right_output_rpm = max(-max_output_rpm, min(max_output_rpm, right_output_rpm))
        else:
            # 不允许反转时，把差速限制到“慢侧至少为 0”。
            delta_output_rpm = max(-center_output_rpm, min(center_output_rpm, delta_output_rpm))
            left_output_rpm = max(0.0, min(max_output_rpm, center_output_rpm + delta_output_rpm))
            right_output_rpm = max(0.0, min(max_output_rpm, center_output_rpm - delta_output_rpm))

        # 输出轴转速通过减速比换算成电机侧转速。
        left_motor_rpm = left_output_rpm * self.gear_ratio * self.left_motor_sign
        right_motor_rpm = right_output_rpm * self.gear_ratio * self.right_motor_sign
        # 调试量使用绝对值，避免相对安装电机时“直行中心速度接近 0”的假象。
        center_motor_rpm = 0.5 * (abs(left_motor_rpm) + abs(right_motor_rpm))
        delta_motor_rpm = 0.5 * (abs(right_motor_rpm) - abs(left_motor_rpm))
        return left_motor_rpm, right_motor_rpm, center_motor_rpm, delta_motor_rpm

    def _filter_angle(self, raw_angle_deg: float) -> float:
        """对视觉角度做一阶低通和小角度死区，减少履带抖动。"""
        if self.filtered_angle_deg is None or self.angle_lowpass_alpha >= 1.0:
            filtered = raw_angle_deg
        elif self.angle_lowpass_alpha <= 0.0:
            filtered = self.filtered_angle_deg
        else:
            filtered = (
                self.angle_lowpass_alpha * raw_angle_deg
                + (1.0 - self.angle_lowpass_alpha) * self.filtered_angle_deg
            )
        if abs(filtered) < self.angle_deadband_deg:
            filtered = 0.0
        self.filtered_angle_deg = filtered
        return filtered

    def _apply_slew_rate(self, target_left_rpm: float, target_right_rpm: float):
        """限制左右电机 RPM 每秒变化量，避免速度阶跃过大。"""
        now = self.get_clock().now()
        if self.last_command_stamp is None or self.max_motor_rpm_step_per_sec <= 0.0:
            self.last_command_stamp = now
            self.last_left_rpm = float(target_left_rpm)
            self.last_right_rpm = float(target_right_rpm)
            return float(target_left_rpm), float(target_right_rpm)

        dt = max(1e-3, (now - self.last_command_stamp).nanoseconds / 1e9)
        max_step = self.max_motor_rpm_step_per_sec * dt

        def limit(prev, target):
            delta = target - prev
            if delta > max_step:
                return prev + max_step
            if delta < -max_step:
                return prev - max_step
            return target

        left_rpm = limit(self.last_left_rpm, float(target_left_rpm))
        right_rpm = limit(self.last_right_rpm, float(target_right_rpm))
        self.last_command_stamp = now
        self.last_left_rpm = left_rpm
        self.last_right_rpm = right_rpm
        return left_rpm, right_rpm

    def on_offset(self, msg: Float32):
        """保存最近一次横向偏移，供角度回调合成控制量。"""
        self.latest_offset_norm = max(-1.0, min(1.0, float(msg.data)))
        self.last_offset_stamp = self.get_clock().now()

    def _current_offset_norm(self) -> float:
        """获取当前可用的横向偏移，超时后自动退化为仅角度控制。"""
        if self.last_offset_stamp is None:
            return 0.0
        if self.offset_timeout_sec <= 0.0:
            return self.latest_offset_norm
        elapsed = (self.get_clock().now() - self.last_offset_stamp).nanoseconds / 1e9
        if elapsed > self.offset_timeout_sec:
            return 0.0
        return self.latest_offset_norm

    def publish_speed(self, left_rpm: float, right_rpm: float):
        """将左右电机速度封装成 ROS 消息并发布。"""
        msg = Float32MultiArray()
        msg.data = [float(left_rpm), float(right_rpm)]
        self.speed_pub.publish(msg)

    def on_angle(self, msg: Float32):
        """角度回调：视觉节点每发一次角度，这里就生成一次速度命令。"""
        raw_angle_deg = float(msg.data)
        angle_deg = self._filter_angle(raw_angle_deg)
        offset_norm = self._current_offset_norm()
        left_rpm, right_rpm, center_motor_rpm, delta_motor_rpm = self.angle_to_motor_rpm(angle_deg, offset_norm)
        left_rpm, right_rpm = self._apply_slew_rate(left_rpm, right_rpm)
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
        """如果一段时间没有收到新角度，则主动停车。"""
        if self.last_angle_stamp is None:
            return

        elapsed = (self.get_clock().now() - self.last_angle_stamp).nanoseconds / 1e9
        if elapsed > self.command_timeout_sec:
            self.publish_speed(0.0, 0.0)
            self.last_angle_stamp = None
            self.last_command_stamp = None
            self.last_left_rpm = 0.0
            self.last_right_rpm = 0.0
            self.filtered_angle_deg = None
            self.last_offset_stamp = None
            self.latest_offset_norm = 0.0
            self.get_logger().warn(
                f"angle command timeout ({elapsed:.2f}s), publish stop to {self.speed_topic}"
            )


def main():
    """节点入口。"""
    rclpy.init()
    node = LineFollowMotorModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_speed(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
