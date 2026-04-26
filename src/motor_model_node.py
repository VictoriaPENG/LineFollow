#!/usr/bin/env python3
"""
视觉角度到电机差速的运动模型节点。

功能概述：
1. 订阅视觉节点发布的线角度 `/line_follow/line_angle_deg`
2. 根据线相对车体前向的夹角计算左右电机目标转速
3. 以固定前进基准速度叠加差速修正，保持车辆持续向前
4. 将结果发布到 `/motor_speed_cmd`
5. 允许根据参数选择“只前进差速”或“允许一侧反转”
6. 当视觉输入暂时丢失时按直行固定速度继续发布命令

模型核心：
- 视觉角度越大，左右电机差速越大
- 车辆始终以固定基准 RPM 前进，转向时只在这个基准上做加减速
- 不再使用横向偏移参与控制，避免“角度正确但被偏移项额外拉偏”
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class LineFollowMotorModelNode(Node):
    """将视觉角度映射为左右电机差额转速。"""

    def __init__(self):
        super().__init__("line_follow_motor_model_node")

        # 输入/输出话题。
        self.declare_parameter("angle_topic", "/line_follow/line_angle_deg")
        self.declare_parameter("line_detected_topic", "/line_follow/line_detected")
        self.declare_parameter("speed_topic", "/motor_speed_cmd")
        self.declare_parameter("enable_topic", "/line_follow/set_enabled")
        self.declare_parameter("publish_debug_topic", True)
        self.declare_parameter("enabled", True)

        # 控制参数：决定角度变化时底盘如何响应。
        self.declare_parameter("max_visual_angle_deg", 50.0)
        self.declare_parameter("heading_gain", 20.0)
        self.declare_parameter("base_motor_rpm", 900.0)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("offset_timeout_sec", 0.3)
        self.declare_parameter("fallback_straight_when_no_angle", True)
        self.declare_parameter("speed_update_period_sec", 0.1)
        self.declare_parameter("angle_lowpass_alpha", 0.35)
        self.declare_parameter("angle_deadband_deg", 2.5)
        self.declare_parameter("straight_angle_epsilon_deg", 1.5)
        self.declare_parameter("max_motor_rpm_step_per_sec", 1500.0)
        self.declare_parameter("max_motor_rpm", 3000.0)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("arm_on_enable_detection_count", 3)

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
        self.latest_angle_deg = None
        self.filtered_angle_deg = None
        self.last_command_stamp = None
        self.last_left_rpm = 0.0
        self.last_right_rpm = 0.0
        self.last_speed_update_stamp = None
        self.held_left_rpm = 0.0
        self.held_right_rpm = 0.0
        self.have_held_command = False
        self.line_detected = False
        self.enable_arm_required = self.enabled and self.arm_on_enable_detection_count > 0
        self.enable_detect_streak = 0

        # 主输出：左右电机目标速度。
        self.speed_pub = self.create_publisher(Float32MultiArray, self.speed_topic, 10)
        self.debug_center_pub = None
        self.debug_delta_pub = None
        if self.publish_debug_topic:
            # 额外调试输出，便于在现场观察“基础速度”和“差速量”。
            self.debug_center_pub = self.create_publisher(Float32, "/line_follow/center_motor_rpm", 10)
            self.debug_delta_pub = self.create_publisher(Float32, "/line_follow/delta_motor_rpm", 10)

        self.sub = self.create_subscription(Float32, self.angle_topic, self.on_angle, 10)
        self.line_detected_sub = self.create_subscription(
            Bool,
            self.line_detected_topic,
            self.on_line_detected,
            10,
        )
        self.enable_sub = self.create_subscription(
            Bool,
            self.enable_topic,
            self.on_enable_command,
            10,
        )
        # 定时器用于输入超时保护，避免相机或视觉异常时电机继续保持旧命令。
        self.timeout_timer = self.create_timer(0.1, self.on_timeout_check)

        self.get_logger().info(
            "motor model ready: "
            f"angle_topic={self.angle_topic}, line_detected_topic={self.line_detected_topic}, "
            f"speed_topic={self.speed_topic}, enable_topic={self.enable_topic}, "
            f"gear_ratio={self.gear_ratio:.2f}, base_motor_rpm={self.base_motor_rpm:.1f}, "
            f"track_width_m={self.track_width_m:.3f}, wheel_diameter_m={self.drive_wheel_diameter_m:.3f}, "
            f"allow_reverse={self.allow_reverse}, left_motor_sign={self.left_motor_sign:.0f}, "
            f"right_motor_sign={self.right_motor_sign:.0f}, heading_gain={self.heading_gain:.2f}, "
            f"steering_sign={self.steering_sign:.0f}, "
            f"angle_lowpass_alpha={self.angle_lowpass_alpha:.2f}, angle_deadband_deg={self.angle_deadband_deg:.2f}, "
            f"speed_update_period_sec={self.speed_update_period_sec:.2f}, "
            f"max_motor_rpm_step_per_sec={self.max_motor_rpm_step_per_sec:.1f}, "
            f"arm_on_enable_detection_count={self.arm_on_enable_detection_count}"
        )

    def _reset_control_state(self) -> None:
        """重置控制缓存，确保重新使能后不会沿用旧速度或旧滤波状态。"""
        self.last_angle_stamp = None
        self.latest_angle_deg = None
        self.filtered_angle_deg = None
        self.last_command_stamp = None
        self.last_left_rpm = 0.0
        self.last_right_rpm = 0.0
        self.last_speed_update_stamp = None
        self.held_left_rpm = 0.0
        self.held_right_rpm = 0.0
        self.have_held_command = False

    def _set_enable_arm_state(self, armed: bool) -> None:
        """控制重新使能后的角度放行状态。"""
        if armed:
            self.enable_arm_required = False
            self.enable_detect_streak = 0
        else:
            self.enable_arm_required = self.arm_on_enable_detection_count > 0
            self.enable_detect_streak = 0

    def _load_static_topics(self):
        """加载需要在启动阶段固定下来的话题类参数。"""
        self.angle_topic = str(self.get_parameter("angle_topic").value)
        self.line_detected_topic = str(self.get_parameter("line_detected_topic").value)
        self.speed_topic = str(self.get_parameter("speed_topic").value)
        self.enable_topic = str(self.get_parameter("enable_topic").value)
        self.publish_debug_topic = bool(self.get_parameter("publish_debug_topic").value)
        self.enabled = bool(self.get_parameter("enabled").value)

    def _refresh_runtime_parameters(self, overrides=None):
        """从参数服务器刷新当前控制/机械参数，支持运行时调参。"""
        values = {
            "max_visual_angle_deg": float(self.get_parameter("max_visual_angle_deg").value),
            "heading_gain": float(self.get_parameter("heading_gain").value),
            "base_motor_rpm": float(self.get_parameter("base_motor_rpm").value),
            "allow_reverse": bool(self.get_parameter("allow_reverse").value),
            "command_timeout_sec": float(self.get_parameter("command_timeout_sec").value),
            "fallback_straight_when_no_angle": bool(
                self.get_parameter("fallback_straight_when_no_angle").value
            ),
            "speed_update_period_sec": float(self.get_parameter("speed_update_period_sec").value),
            "angle_lowpass_alpha": float(self.get_parameter("angle_lowpass_alpha").value),
            "angle_deadband_deg": float(self.get_parameter("angle_deadband_deg").value),
            "straight_angle_epsilon_deg": float(self.get_parameter("straight_angle_epsilon_deg").value),
            "max_motor_rpm_step_per_sec": float(self.get_parameter("max_motor_rpm_step_per_sec").value),
            "max_motor_rpm": float(self.get_parameter("max_motor_rpm").value),
            "steering_sign": float(self.get_parameter("steering_sign").value),
            "arm_on_enable_detection_count": int(
                self.get_parameter("arm_on_enable_detection_count").value
            ),
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
        self.base_motor_rpm = max(0.0, float(values["base_motor_rpm"]))
        self.allow_reverse = bool(values["allow_reverse"])
        self.command_timeout_sec = max(0.1, float(values["command_timeout_sec"]))
        self.fallback_straight_when_no_angle = bool(values["fallback_straight_when_no_angle"])
        self.speed_update_period_sec = max(0.1, float(values["speed_update_period_sec"]))
        self.angle_lowpass_alpha = min(1.0, max(0.0, float(values["angle_lowpass_alpha"])))
        self.angle_deadband_deg = max(0.0, float(values["angle_deadband_deg"]))
        self.straight_angle_epsilon_deg = max(0.0, float(values["straight_angle_epsilon_deg"]))
        self.max_motor_rpm_step_per_sec = max(0.0, float(values["max_motor_rpm_step_per_sec"]))
        self.max_motor_rpm = max(0.0, float(values["max_motor_rpm"]))
        self.steering_sign = 1.0 if float(values["steering_sign"]) >= 0.0 else -1.0
        self.arm_on_enable_detection_count = max(0, int(values["arm_on_enable_detection_count"]))
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

    def _sanitize_scalar(self, value: float, default: float = 0.0, limit: float = None) -> float:
        """过滤 NaN/Inf，并在需要时追加对称限幅。"""
        try:
            value = float(value)
        except (TypeError, ValueError):
            value = default
        if not math.isfinite(value):
            value = default
        if limit is not None:
            value = max(-limit, min(limit, value))
        return value

    def _set_enabled_state(self, enabled: bool) -> None:
        previous_enabled = self.enabled
        self.enabled = bool(enabled)
        if not self.enabled:
            self.publish_speed(0.0, 0.0)
            self._reset_control_state()
            self._set_enable_arm_state(armed=False)
        elif not previous_enabled:
            # 重新使能时先清空旧角度/旧速度，让新的视觉输入重新接管。
            self._reset_control_state()
            self._set_enable_arm_state(armed=False)
            self.publish_speed(0.0, 0.0)
        elif self.arm_on_enable_detection_count > 0:
            self._set_enable_arm_state(armed=False)

    def on_enable_command(self, msg: Bool) -> None:
        desired = bool(msg.data)
        if desired == self.enabled:
            return
        self._set_enabled_state(desired)
        self.get_logger().info(f"line-follow enabled set to {desired} by topic command")

    def angle_to_motor_rpm(self, angle_deg: float):
        """
        将视觉角度转换为左右电机 RPM。

        返回：
        - 左电机 RPM
        - 右电机 RPM
        - 平均电机 RPM（调试用）
        - 差额电机 RPM（调试用）
        """
        angle_deg = self._sanitize_scalar(angle_deg, default=0.0, limit=self.max_visual_angle_deg)
        if abs(angle_deg) <= self.straight_angle_epsilon_deg:
            angle_deg = 0.0

        center_motor_rpm = self.base_motor_rpm
        delta_motor_rpm = self.steering_sign * self.heading_gain * angle_deg
        left_motor_rpm = center_motor_rpm + delta_motor_rpm
        right_motor_rpm = center_motor_rpm - delta_motor_rpm

        if self.allow_reverse:
            if self.max_motor_rpm > 0.0:
                left_motor_rpm = max(-self.max_motor_rpm, min(self.max_motor_rpm, left_motor_rpm))
                right_motor_rpm = max(-self.max_motor_rpm, min(self.max_motor_rpm, right_motor_rpm))
        else:
            left_motor_rpm = max(0.0, left_motor_rpm)
            right_motor_rpm = max(0.0, right_motor_rpm)
            if self.max_motor_rpm > 0.0:
                left_motor_rpm = min(self.max_motor_rpm, left_motor_rpm)
                right_motor_rpm = min(self.max_motor_rpm, right_motor_rpm)

        signed_left_motor_rpm = left_motor_rpm * self.left_motor_sign
        signed_right_motor_rpm = right_motor_rpm * self.right_motor_sign
        return (
            signed_left_motor_rpm,
            signed_right_motor_rpm,
            0.5 * (abs(signed_left_motor_rpm) + abs(signed_right_motor_rpm)),
            0.5 * (abs(signed_right_motor_rpm) - abs(signed_left_motor_rpm)),
        )

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

    def publish_speed(self, left_rpm: float, right_rpm: float):
        """将左右电机速度封装成 ROS 消息并发布。"""
        msg = Float32MultiArray()
        msg.data = [float(left_rpm), float(right_rpm)]
        self.speed_pub.publish(msg)

    def update_held_angle_command(self, angle_deg: float, *, filtered: bool = True) -> None:
        """按给定角度更新锁定速度；无视觉输入时 angle=0 表示直行。"""
        if filtered:
            angle_deg = self._filter_angle(angle_deg)
        left_rpm, right_rpm, center_motor_rpm, delta_motor_rpm = self.angle_to_motor_rpm(angle_deg)
        left_rpm, right_rpm = self._apply_slew_rate(left_rpm, right_rpm)
        self.held_left_rpm = left_rpm
        self.held_right_rpm = right_rpm
        self.have_held_command = True
        self.last_speed_update_stamp = self.get_clock().now()
        self.publish_speed(left_rpm, right_rpm)

        if self.debug_center_pub is not None:
            debug_msg = Float32()
            debug_msg.data = float(center_motor_rpm)
            self.debug_center_pub.publish(debug_msg)

        if self.debug_delta_pub is not None:
            debug_msg = Float32()
            debug_msg.data = float(delta_motor_rpm)
            self.debug_delta_pub.publish(debug_msg)

    def on_angle(self, msg: Float32):
        """角度回调：只缓存最新角度，速度命令按固定周期更新。"""
        if not self.enabled:
            self.last_angle_stamp = None
            self.latest_angle_deg = None
            return

        self.latest_angle_deg = self._sanitize_scalar(msg.data, default=0.0, limit=self.max_visual_angle_deg)
        self.last_angle_stamp = self.get_clock().now()

        if self.enable_arm_required:
            return

        if not self.have_held_command:
            self.update_held_angle_command(self.latest_angle_deg, filtered=True)

    def on_line_detected(self, msg: Bool):
        """重新使能后要求连续若干帧检测成功，再允许角度控制放行。"""
        self.line_detected = bool(msg.data)
        if not self.enabled or not self.enable_arm_required:
            return

        if self.line_detected:
            self.enable_detect_streak += 1
        else:
            self.enable_detect_streak = 0

        if self.enable_detect_streak >= self.arm_on_enable_detection_count:
            self._set_enable_arm_state(armed=True)
            if self.latest_angle_deg is not None:
                self.update_held_angle_command(self.latest_angle_deg, filtered=True)

    def _latest_angle_is_fresh(self, now) -> bool:
        if self.last_angle_stamp is None or self.latest_angle_deg is None:
            return False
        elapsed = (now - self.last_angle_stamp).nanoseconds / 1e9
        return elapsed <= self.command_timeout_sec

    def _update_command_if_due(self, now) -> None:
        if (
            self.have_held_command
            and self.last_speed_update_stamp is not None
            and (now - self.last_speed_update_stamp).nanoseconds / 1e9 < self.speed_update_period_sec
        ):
            return

        if self.enable_arm_required:
            self.publish_speed(0.0, 0.0)
            return

        if self._latest_angle_is_fresh(now):
            self.update_held_angle_command(self.latest_angle_deg, filtered=True)
            return

        self.latest_angle_deg = None
        self.last_angle_stamp = None
        self.filtered_angle_deg = None
        self.last_command_stamp = None
        self.last_left_rpm = 0.0
        self.last_right_rpm = 0.0
        if self.fallback_straight_when_no_angle:
            self.update_held_angle_command(0.0, filtered=False)
        else:
            self.held_left_rpm = 0.0
            self.held_right_rpm = 0.0
            self.have_held_command = True
            self.last_speed_update_stamp = now
            self.publish_speed(0.0, 0.0)

    def on_timeout_check(self):
        """按固定周期更新速度目标，并重复发布当前目标给驱动保活。"""
        if not self.enabled:
            return

        now = self.get_clock().now()
        self._update_command_if_due(now)
        if self.have_held_command:
            self.publish_speed(self.held_left_rpm, self.held_right_rpm)


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
