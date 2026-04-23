#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    runtime_params_file_default = PathJoinSubstitution(
        [FindPackageShare("line_follow"), "config", "runtime_params.yaml"]
    )

    runtime_params_file_arg = DeclareLaunchArgument(
        "runtime_params_file",
        default_value=runtime_params_file_default,
        description="Runtime parameter YAML for remote, joystick, and motor driver nodes",
    )
    remote_forward_pin_arg = DeclareLaunchArgument("remote_forward_pin", default_value="31")
    remote_reverse_pin_arg = DeclareLaunchArgument("remote_reverse_pin", default_value="29")
    joystick_forward_pin_arg = DeclareLaunchArgument("joystick_forward_pin", default_value="11")
    joystick_reverse_pin_arg = DeclareLaunchArgument("joystick_reverse_pin", default_value="15")
    joystick_left_pin_arg = DeclareLaunchArgument("joystick_left_pin", default_value="13")
    joystick_right_pin_arg = DeclareLaunchArgument("joystick_right_pin", default_value="16")
    remote_hold_seconds_arg = DeclareLaunchArgument("remote_hold_seconds", default_value="0.5")
    remote_poll_hz_arg = DeclareLaunchArgument("remote_poll_hz", default_value="20.0")
    joystick_publish_hz_arg = DeclareLaunchArgument("joystick_publish_hz", default_value="20.0")
    debounce_activate_count_arg = DeclareLaunchArgument("debounce_activate_count", default_value="5")
    remote_debounce_deactivate_count_arg = DeclareLaunchArgument(
        "remote_debounce_deactivate_count", default_value="1"
    )
    joystick_debounce_deactivate_count_arg = DeclareLaunchArgument(
        "joystick_debounce_deactivate_count", default_value="1"
    )
    launch_command_arg = DeclareLaunchArgument(
        "launch_command",
        default_value="ros2 launch line_follow line_follow_system.launch.py line_follow_enabled:=false start_driver:=false",
    )

    remote_node = Node(
        package="line_follow",
        executable="remote_long_press_start_line_follow_node",
        output="screen",
        parameters=[
            LaunchConfiguration("runtime_params_file"),
            {
                "forward_pin": LaunchConfiguration("remote_forward_pin"),
                "reverse_pin": LaunchConfiguration("remote_reverse_pin"),
                "hold_seconds": LaunchConfiguration("remote_hold_seconds"),
                "poll_hz": LaunchConfiguration("remote_poll_hz"),
                "debounce_activate_count": LaunchConfiguration("debounce_activate_count"),
                "debounce_deactivate_count": LaunchConfiguration("remote_debounce_deactivate_count"),
                "launch_command": LaunchConfiguration("launch_command"),
                "autostart_line_follow_stack": True,
                "autostart_driver_process": True,
            },
        ],
    )

    joystick_node = Node(
        package="line_follow",
        executable="joystick_drive_node",
        output="screen",
        parameters=[
            LaunchConfiguration("runtime_params_file"),
            {
                "forward_pin": LaunchConfiguration("joystick_forward_pin"),
                "reverse_pin": LaunchConfiguration("joystick_reverse_pin"),
                "left_pin": LaunchConfiguration("joystick_left_pin"),
                "right_pin": LaunchConfiguration("joystick_right_pin"),
                "publish_hz": LaunchConfiguration("joystick_publish_hz"),
                "debounce_activate_count": LaunchConfiguration("debounce_activate_count"),
                "debounce_deactivate_count": LaunchConfiguration("joystick_debounce_deactivate_count"),
                "autostart_driver_process": False,
            },
        ],
    )

    return LaunchDescription(
        [
            runtime_params_file_arg,
            remote_forward_pin_arg,
            remote_reverse_pin_arg,
            joystick_forward_pin_arg,
            joystick_reverse_pin_arg,
            joystick_left_pin_arg,
            joystick_right_pin_arg,
            remote_hold_seconds_arg,
            remote_poll_hz_arg,
            joystick_publish_hz_arg,
            debounce_activate_count_arg,
            remote_debounce_deactivate_count_arg,
            joystick_debounce_deactivate_count_arg,
            launch_command_arg,
            remote_node,
            joystick_node,
        ]
    )
