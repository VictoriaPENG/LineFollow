#!/usr/bin/env python3
"""Simple Gazebo pose controller for first-pass line-follow simulation.

This node is intentionally lightweight: it subscribes to ``/cmd_vel`` and moves
the Gazebo model through ``gazebo_msgs/srv/SetEntityState``. It is not a full
physics drivetrain, but it gives the vision/control pipeline a controllable
robot before detailed track dynamics are modeled.
"""

import math
from typing import Optional

import rclpy
from rclpy.task import Future
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node


def yaw_to_quaternion(yaw: float):
    """Return a z-axis quaternion tuple for yaw."""
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class SimCmdVelController(Node):
    """Move a Gazebo entity from Twist commands."""

    def __init__(self):
        super().__init__("sim_cmd_vel_controller_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("entity_name", "line_follow_car")
        self.declare_parameter("service_name", "/set_entity_state")
        self.declare_parameter(
            "service_candidates",
            ["/set_entity_state", "/gazebo/set_entity_state"],
        )
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 0.05)
        self.declare_parameter("initial_yaw", 0.0)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.entity_name = str(self.get_parameter("entity_name").value)
        self.service_name = str(self.get_parameter("service_name").value)
        self.service_candidates = [
            str(item) for item in self.get_parameter("service_candidates").value
        ]
        self.update_rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.command_timeout_sec = max(
            0.0,
            float(self.get_parameter("command_timeout_sec").value),
        )

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.z = float(self.get_parameter("initial_z").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)

        self.latest_cmd = Twist()
        self.last_cmd_time: Optional[rclpy.time.Time] = None
        self.last_update_time: Optional[rclpy.time.Time] = None
        self.last_service_warn_time: Optional[rclpy.time.Time] = None
        self.pending_request: Optional[Future] = None

        self.client = None
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.on_timer)

        self.get_logger().info(
            "sim cmd_vel controller ready: "
            f"entity={self.entity_name}, cmd_vel_topic={self.cmd_vel_topic}, "
            f"service_candidates={self.service_candidates}, "
            f"update_rate_hz={self.update_rate_hz:.1f}"
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self) -> None:
        now = self.get_clock().now()
        if self.last_update_time is None:
            self.last_update_time = now
            return

        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        linear_x = 0.0
        angular_z = 0.0
        if self.last_cmd_time is not None:
            cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9
            if cmd_age <= self.command_timeout_sec:
                linear_x = float(self.latest_cmd.linear.x)
                angular_z = float(self.latest_cmd.angular.z)

        self.yaw += angular_z * dt
        self.x += linear_x * math.cos(self.yaw) * dt
        self.y += linear_x * math.sin(self.yaw) * dt

        if not self._ensure_service_client(now):
            self._warn_service_unavailable(now)
            return

        if self.pending_request is not None and not self.pending_request.done():
            return

        request = SetEntityState.Request()
        request.state = self._build_state(linear_x, angular_z)
        self.pending_request = self.client.call_async(request)
        self.pending_request.add_done_callback(self._on_set_state_done)

    def _ensure_service_client(self, now: rclpy.time.Time) -> bool:
        if self.client is not None and self.client.service_is_ready():
            return True

        available_services = {
            name: service_types
            for name, service_types in self.get_service_names_and_types()
        }
        set_state_type = "gazebo_msgs/srv/SetEntityState"
        for candidate in [self.service_name] + self.service_candidates:
            if set_state_type in available_services.get(candidate, []):
                self.client = self.create_client(SetEntityState, candidate)
                self.service_name = candidate
                self.get_logger().info(f"using Gazebo SetEntityState service: {candidate}")
                return self.client.service_is_ready()

        if self.client is None:
            self.client = self.create_client(SetEntityState, self.service_name)
        return self.client.service_is_ready()

    def _warn_service_unavailable(self, now: rclpy.time.Time) -> None:
        if self.last_service_warn_time is not None:
            age = (now - self.last_service_warn_time).nanoseconds * 1e-9
            if age < 2.0:
                return
        self.last_service_warn_time = now
        services = [name for name, _ in self.get_service_names_and_types()]
        entity_services = [name for name in services if "entity" in name or "state" in name]
        self.get_logger().warn(
            "Gazebo SetEntityState service is not available yet. "
            f"state/entity services currently visible: {entity_services}"
        )

    def _build_state(self, linear_x: float, angular_z: float) -> EntityState:
        state = EntityState()
        state.name = self.entity_name
        state.reference_frame = "world"
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        state.twist.linear.x = linear_x
        state.twist.angular.z = angular_z
        return state

    def _on_set_state_done(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - depends on Gazebo runtime
            self.get_logger().warn(f"set entity state failed: {exc}")
            return
        if not result.success:
            self.get_logger().warn("set entity state rejected")


def main(args=None):
    rclpy.init(args=args)
    node = SimCmdVelController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
