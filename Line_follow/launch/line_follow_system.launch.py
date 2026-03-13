#!/usr/bin/env python3
"""巡线整系统启动文件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    usb_launch = PathJoinSubstitution([FindPackageShare("line_follow"), "launch", "usb_cam_web.launch.py"])

    device_arg = DeclareLaunchArgument(
        "device",
        default_value="/dev/video8",
        description="USB camera device path",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/image",
        description="Image topic used by line follow angle node",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Motor driver serial device",
    )
    track_width_arg = DeclareLaunchArgument(
        "track_width_m",
        default_value="0.577",
        description="Distance between left and right track center lines",
    )
    show_debug_arg = DeclareLaunchArgument(
        "show_debug",
        default_value="false",
        description="Whether to show OpenCV debug windows",
    )
    line_is_white_arg = DeclareLaunchArgument(
        "line_is_white",
        default_value="false",
        description="Whether the guide line is white",
    )
    heading_gain_arg = DeclareLaunchArgument(
        "heading_gain",
        default_value="1.8",
        description="Yaw rate gain for visual angle control",
    )
    base_speed_ratio_arg = DeclareLaunchArgument(
        "base_speed_ratio",
        default_value="0.32",
        description="Base speed ratio relative to rated output rpm",
    )
    speed_reduce_gain_arg = DeclareLaunchArgument(
        "speed_reduce_gain",
        default_value="0.55",
        description="Slow down ratio when visual angle grows",
    )
    allow_reverse_arg = DeclareLaunchArgument(
        "allow_reverse",
        default_value="true",
        description="Allow tracked chassis to reverse one side on sharp turns",
    )
    left_slave_arg = DeclareLaunchArgument(
        "left_slave",
        default_value="6",
        description="Modbus slave id for left motor driver",
    )
    right_slave_arg = DeclareLaunchArgument(
        "right_slave",
        default_value="8",
        description="Modbus slave id for right motor driver",
    )
    min_speed_rpm_arg = DeclareLaunchArgument(
        "min_speed_rpm",
        default_value="0",
        description="Minimum driver rpm limit",
    )
    max_speed_rpm_arg = DeclareLaunchArgument(
        "max_speed_rpm",
        default_value="5000",
        description="Maximum driver rpm limit",
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(usb_launch),
        launch_arguments={
            "device": LaunchConfiguration("device"),
        }.items(),
    )

    angle_process = Node(
        package="line_follow",
        executable="line_follow_angle_node",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "line_is_white": LaunchConfiguration("line_is_white"),
                "show_debug": LaunchConfiguration("show_debug"),
            }
        ],
    )

    model_process = Node(
        package="line_follow",
        executable="line_follow_motor_model_node",
        output="screen",
        parameters=[
            {
                "track_width_m": LaunchConfiguration("track_width_m"),
                "heading_gain": LaunchConfiguration("heading_gain"),
                "base_speed_ratio": LaunchConfiguration("base_speed_ratio"),
                "speed_reduce_gain": LaunchConfiguration("speed_reduce_gain"),
                "allow_reverse": LaunchConfiguration("allow_reverse"),
            }
        ],
    )

    driver_process = Node(
        package="line_follow",
        executable="motor_driver_control_node",
        output="screen",
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "left_slave": LaunchConfiguration("left_slave"),
                "right_slave": LaunchConfiguration("right_slave"),
                "min_speed_rpm": LaunchConfiguration("min_speed_rpm"),
                "max_speed_rpm": LaunchConfiguration("max_speed_rpm"),
            }
        ],
    )

    return LaunchDescription(
        [
            device_arg,
            image_topic_arg,
            serial_port_arg,
            track_width_arg,
            show_debug_arg,
            line_is_white_arg,
            heading_gain_arg,
            base_speed_ratio_arg,
            speed_reduce_gain_arg,
            allow_reverse_arg,
            left_slave_arg,
            right_slave_arg,
            min_speed_rpm_arg,
            max_speed_rpm_arg,
            camera_launch,
            angle_process,
            model_process,
            driver_process,
        ]
    )
