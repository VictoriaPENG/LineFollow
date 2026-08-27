#!/usr/bin/env python3
"""Smoke-test Gazebo motion for the line-follow simulation model."""

import math
import sys
from typing import Optional

import rclpy
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node


def yaw_from_quaternion(orientation) -> float:
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


class SimMotionSmokeTest(Node):
    """Publish a short cmd_vel command and verify that Gazebo pose changes."""

    def __init__(self):
        super().__init__("sim_motion_smoke_test_node")

        self.declare_parameter("entity_name", "line_follow_car")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "state_service_candidates",
            ["/get_entity_state", "/gazebo/get_entity_state"],
        )
        self.declare_parameter("linear_x", 0.35)
        self.declare_parameter("angular_z", 0.45)
        self.declare_parameter("duration_sec", 3.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("min_translation_m", 0.20)
        self.declare_parameter("min_yaw_change_rad", 0.20)

        self.entity_name = str(self.get_parameter("entity_name").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.service_candidates = [
            str(item) for item in self.get_parameter("state_service_candidates").value
        ]
        self.linear_x = float(self.get_parameter("linear_x").value)
        self.angular_z = float(self.get_parameter("angular_z").value)
        self.duration_sec = max(0.1, float(self.get_parameter("duration_sec").value))
        self.publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.min_translation_m = max(
            0.0,
            float(self.get_parameter("min_translation_m").value),
        )
        self.min_yaw_change_rad = max(
            0.0,
            float(self.get_parameter("min_yaw_change_rad").value),
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_clients = {
            name: self.create_client(GetEntityState, name)
            for name in self.service_candidates
        }

    def _get_ready_state_client(self, timeout_sec: float):
        service_type = "gazebo_msgs/srv/GetEntityState"
        available = {
            name: service_types for name, service_types in self.get_service_names_and_types()
        }
        for candidate in self.service_candidates:
            if service_type in available.get(candidate, []):
                self.get_logger().info(f"using Gazebo GetEntityState service: {candidate}")
                return self.state_clients[candidate]

        wait_per_candidate = max(0.1, timeout_sec / max(1, len(self.service_candidates)))
        for candidate, client in self.state_clients.items():
            self.get_logger().info(f"waiting for Gazebo GetEntityState service: {candidate}")
            if client.wait_for_service(timeout_sec=wait_per_candidate):
                return client
        return None

    def get_state(self, timeout_sec: float = 5.0) -> Optional[GetEntityState.Response]:
        state_client = self._get_ready_state_client(timeout_sec)
        if state_client is None:
            self.get_logger().error("Gazebo GetEntityState service is not available")
            return None

        request = GetEntityState.Request()
        request.name = self.entity_name
        request.reference_frame = "world"
        future = state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error("timed out while reading Gazebo entity state")
            return None

        result = future.result()
        if result is None or not result.success:
            self.get_logger().error("failed to read entity state")
            return None
        return result

    def publish_motion_command(self) -> None:
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.angular.z = self.angular_z

        stop = Twist()
        period = 1.0 / self.publish_hz
        end_time = self.get_clock().now().nanoseconds * 1e-9 + self.duration_sec

        while rclpy.ok():
            now = self.get_clock().now().nanoseconds * 1e-9
            if now >= end_time:
                break
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=period)

        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self) -> bool:
        start = self.get_state()
        if start is None:
            return False

        self.get_logger().info(
            "start pose: "
            f"x={start.state.pose.position.x:.3f}, "
            f"y={start.state.pose.position.y:.3f}"
        )
        self.publish_motion_command()

        end = self.get_state()
        if end is None:
            return False

        dx = end.state.pose.position.x - start.state.pose.position.x
        dy = end.state.pose.position.y - start.state.pose.position.y
        translation = math.hypot(dx, dy)
        yaw_start = yaw_from_quaternion(start.state.pose.orientation)
        yaw_end = yaw_from_quaternion(end.state.pose.orientation)
        yaw_delta = abs(shortest_angle_delta(yaw_start, yaw_end))

        self.get_logger().info(
            "end pose: "
            f"x={end.state.pose.position.x:.3f}, "
            f"y={end.state.pose.position.y:.3f}, "
            f"translation={translation:.3f} m, "
            f"yaw_delta={yaw_delta:.3f} rad"
        )

        translation_ok = translation >= self.min_translation_m
        yaw_ok = yaw_delta >= self.min_yaw_change_rad
        if translation_ok and yaw_ok:
            self.get_logger().info("Gazebo motion smoke test passed")
            return True

        self.get_logger().error(
            "Gazebo motion smoke test failed: "
            f"translation_ok={translation_ok}, yaw_ok={yaw_ok}"
        )
        return False


def main(args=None):
    rclpy.init(args=args)
    node = SimMotionSmokeTest()
    try:
        return 0 if node.run() else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
