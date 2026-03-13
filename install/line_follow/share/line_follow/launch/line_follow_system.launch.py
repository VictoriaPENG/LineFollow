#!/usr/bin/env python3
"""
巡线整系统启动文件。

本 launch 的职责是把“相机 -> 视觉检测 -> 运动模型 -> 驱动控制”整条链路一次拉起：
1. 启动 USB 相机与 Web 图像显示
2. 启动视觉角度检测节点
3. 启动视觉角度到电机转速的运动模型节点
4. 启动电机驱动控制节点

这样在现场调试时，只需要一条 `ros2 launch` 命令即可完成整系统联调。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 在安装后的包目录中找到复用的相机 launch 文件。
    usb_launch = PathJoinSubstitution([FindPackageShare("line_follow"), "launch", "usb_cam_web.launch.py"])

    # 以下都是可以在命令行覆盖的 launch 参数。
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="/dev/video8",
        description="USB camera device path",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/hbmem_img",
        description="Image topic used by line follow angle node",
    )
    image_msg_type_arg = DeclareLaunchArgument(
        "image_msg_type",
        default_value="hbmem",
        description="Input image message type: hbmem or raw",
    )
    offset_px_topic_arg = DeclareLaunchArgument(
        "offset_px_topic",
        default_value="/line_follow/line_offset_px",
        description="Pixel-space lateral offset topic published by line follow angle node",
    )
    offset_norm_topic_arg = DeclareLaunchArgument(
        "offset_norm_topic",
        default_value="/line_follow/line_offset_norm",
        description="Normalized lateral offset topic published by line follow angle node",
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
    publish_debug_image_arg = DeclareLaunchArgument(
        "publish_debug_image",
        default_value="true",
        description="Whether to publish debug image topics",
    )
    show_angle_curve_arg = DeclareLaunchArgument(
        "show_angle_curve",
        default_value="true",
        description="Whether to generate angle curve debug image/window",
    )
    angle_curve_history_size_arg = DeclareLaunchArgument(
        "angle_curve_history_size",
        default_value="240",
        description="How many frames to keep in angle curve history",
    )
    angle_curve_limit_deg_arg = DeclareLaunchArgument(
        "angle_curve_limit_deg",
        default_value="45.0",
        description="Y-axis limit for angle curve visualization",
    )
    line_is_white_arg = DeclareLaunchArgument(
        "line_is_white",
        default_value="false",
        description="Whether the guide line is white",
    )
    blur_ksize_arg = DeclareLaunchArgument(
        "blur_ksize",
        default_value="5",
        description="Gaussian blur kernel size used before threshold",
    )
    thresh_arg = DeclareLaunchArgument(
        "thresh",
        default_value="-1",
        description="Binary threshold, -1 means Otsu auto threshold",
    )
    morph_ksize_arg = DeclareLaunchArgument(
        "morph_ksize",
        default_value="3",
        description="Morphology kernel size for denoise and gap filling",
    )
    n_windows_arg = DeclareLaunchArgument(
        "n_windows",
        default_value="9",
        description="Sliding window count from bottom to top",
    )
    margin_arg = DeclareLaunchArgument(
        "margin",
        default_value="300",
        description="Sliding window half width in pixels",
    )
    minpix_arg = DeclareLaunchArgument(
        "minpix",
        default_value="50",
        description="Minimum non-zero pixels required per window",
    )
    kp_arg = DeclareLaunchArgument(
        "kp",
        default_value="1.0",
        description="Proportional gain used to generate steer debug value",
    )
    roi_y_start_ratio_arg = DeclareLaunchArgument(
        "roi_y_start_ratio",
        default_value="0.5",
        description="Histogram ROI start height ratio, smaller means larger ROI",
    )
    min_component_area_arg = DeclareLaunchArgument(
        "min_component_area",
        default_value="0",
        description="Minimum connected-component area kept after thresholding",
    )
    max_component_area_arg = DeclareLaunchArgument(
        "max_component_area",
        default_value="0",
        description="Maximum connected-component area kept after thresholding, 0 disables limit",
    )
    max_component_width_px_arg = DeclareLaunchArgument(
        "max_component_width_px",
        default_value="0",
        description="Maximum connected-component width in pixels, 0 disables limit",
    )
    max_component_width_ratio_arg = DeclareLaunchArgument(
        "max_component_width_ratio",
        default_value="0.0",
        description="Maximum connected-component width ratio relative to image width, 0 disables limit",
    )
    component_intensity_limit_arg = DeclareLaunchArgument(
        "component_intensity_limit",
        default_value="-1.0",
        description="Gray-intensity filter for connected components, negative disables filtering",
    )
    max_visual_angle_deg_arg = DeclareLaunchArgument(
        "max_visual_angle_deg",
        default_value="45.0",
        description="Clamp visual angle input before motor model conversion",
    )
    heading_gain_arg = DeclareLaunchArgument(
        "heading_gain",
        default_value="1.8",
        description="Yaw rate gain for visual angle control",
    )
    lateral_gain_arg = DeclareLaunchArgument(
        "lateral_gain",
        default_value="18.0",
        description="Extra angle-equivalent gain applied to normalized lateral offset",
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
    min_output_rpm_arg = DeclareLaunchArgument(
        "min_output_rpm",
        default_value="18.0",
        description="Minimum output shaft rpm maintained by motor model",
    )
    command_timeout_sec_arg = DeclareLaunchArgument(
        "command_timeout_sec",
        default_value="0.5",
        description="Timeout before motor model publishes stop command",
    )
    offset_timeout_sec_arg = DeclareLaunchArgument(
        "offset_timeout_sec",
        default_value="0.3",
        description="How long the motor model keeps using the last lateral offset sample",
    )
    angle_lowpass_alpha_arg = DeclareLaunchArgument(
        "angle_lowpass_alpha",
        default_value="0.35",
        description="Low-pass filter alpha for visual angle, 0-1",
    )
    angle_deadband_deg_arg = DeclareLaunchArgument(
        "angle_deadband_deg",
        default_value="1.5",
        description="Deadband applied to filtered visual angle in degrees",
    )
    max_motor_rpm_step_per_sec_arg = DeclareLaunchArgument(
        "max_motor_rpm_step_per_sec",
        default_value="1200.0",
        description="Maximum allowed RPM change per second for each motor",
    )
    drive_wheel_diameter_arg = DeclareLaunchArgument(
        "drive_wheel_diameter_m",
        default_value="0.18",
        description="Drive wheel diameter used by motor model",
    )
    track_pitch_arg = DeclareLaunchArgument(
        "track_pitch_m",
        default_value="0.06",
        description="Track pitch used by motor model",
    )
    track_link_count_arg = DeclareLaunchArgument(
        "track_link_count",
        default_value="24",
        description="Track link count used by motor model",
    )
    left_motor_sign_arg = DeclareLaunchArgument(
        "left_motor_sign",
        default_value="1.0",
        description="Motor direction sign for left side, use 1.0 or -1.0",
    )
    right_motor_sign_arg = DeclareLaunchArgument(
        "right_motor_sign",
        default_value="-1.0",
        description="Motor direction sign for right side, use 1.0 or -1.0",
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
    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="9600",
        description="Motor driver serial baud rate",
    )
    serial_timeout_arg = DeclareLaunchArgument(
        "serial_timeout_sec",
        default_value="0.1",
        description="Motor driver serial timeout",
    )
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Whether to write mode register after speed register",
    )
    driver_command_timeout_sec_arg = DeclareLaunchArgument(
        "driver_command_timeout_sec",
        default_value="0.5",
        description="Timeout before driver node actively sends stop command",
    )
    write_retry_count_arg = DeclareLaunchArgument(
        "write_retry_count",
        default_value="2",
        description="How many retries are allowed for each Modbus write",
    )
    fail_safe_on_write_error_arg = DeclareLaunchArgument(
        "fail_safe_on_write_error",
        default_value="true",
        description="Whether to immediately stop both motors on write error",
    )
    max_consecutive_write_errors_arg = DeclareLaunchArgument(
        "max_consecutive_write_errors",
        default_value="3",
        description="How many consecutive write failures are allowed before latching fault",
    )

    # 相机与网页显示链路，复用已有 launch。
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(usb_launch),
        launch_arguments={
            "device": LaunchConfiguration("device"),
        }.items(),
    )

    # 视觉角度检测节点。
    angle_process = Node(
        package="line_follow",
        executable="line_follow_angle_node",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "image_msg_type": LaunchConfiguration("image_msg_type"),
                "offset_px_topic": LaunchConfiguration("offset_px_topic"),
                "offset_norm_topic": LaunchConfiguration("offset_norm_topic"),
                "line_is_white": LaunchConfiguration("line_is_white"),
                "blur_ksize": LaunchConfiguration("blur_ksize"),
                "thresh": LaunchConfiguration("thresh"),
                "morph_ksize": LaunchConfiguration("morph_ksize"),
                "n_windows": LaunchConfiguration("n_windows"),
                "margin": LaunchConfiguration("margin"),
                "minpix": LaunchConfiguration("minpix"),
                "kp": LaunchConfiguration("kp"),
                "roi_y_start_ratio": LaunchConfiguration("roi_y_start_ratio"),
                "min_component_area": LaunchConfiguration("min_component_area"),
                "max_component_area": LaunchConfiguration("max_component_area"),
                "max_component_width_px": LaunchConfiguration("max_component_width_px"),
                "max_component_width_ratio": LaunchConfiguration("max_component_width_ratio"),
                "component_intensity_limit": LaunchConfiguration("component_intensity_limit"),
                "show_debug": LaunchConfiguration("show_debug"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                "show_angle_curve": LaunchConfiguration("show_angle_curve"),
                "angle_curve_history_size": LaunchConfiguration("angle_curve_history_size"),
                "angle_curve_limit_deg": LaunchConfiguration("angle_curve_limit_deg"),
            }
        ],
    )

    # 视觉角度 -> 电机转速模型节点。
    model_process = Node(
        package="line_follow",
        executable="line_follow_motor_model_node",
        output="screen",
        parameters=[
            {
                "track_width_m": LaunchConfiguration("track_width_m"),
                "max_visual_angle_deg": LaunchConfiguration("max_visual_angle_deg"),
                "heading_gain": LaunchConfiguration("heading_gain"),
                "lateral_gain": LaunchConfiguration("lateral_gain"),
                "offset_topic": LaunchConfiguration("offset_norm_topic"),
                "base_speed_ratio": LaunchConfiguration("base_speed_ratio"),
                "speed_reduce_gain": LaunchConfiguration("speed_reduce_gain"),
                "min_output_rpm": LaunchConfiguration("min_output_rpm"),
                "allow_reverse": LaunchConfiguration("allow_reverse"),
                "command_timeout_sec": LaunchConfiguration("command_timeout_sec"),
                "offset_timeout_sec": LaunchConfiguration("offset_timeout_sec"),
                "angle_lowpass_alpha": LaunchConfiguration("angle_lowpass_alpha"),
                "angle_deadband_deg": LaunchConfiguration("angle_deadband_deg"),
                "max_motor_rpm_step_per_sec": LaunchConfiguration("max_motor_rpm_step_per_sec"),
                "drive_wheel_diameter_m": LaunchConfiguration("drive_wheel_diameter_m"),
                "track_pitch_m": LaunchConfiguration("track_pitch_m"),
                "track_link_count": LaunchConfiguration("track_link_count"),
                "left_motor_sign": LaunchConfiguration("left_motor_sign"),
                "right_motor_sign": LaunchConfiguration("right_motor_sign"),
            }
        ],
    )

    # Modbus 电机驱动控制节点。
    driver_process = Node(
        package="line_follow",
        executable="motor_driver_control_node",
        output="screen",
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": LaunchConfiguration("baud_rate"),
                "serial_timeout_sec": LaunchConfiguration("serial_timeout_sec"),
                "left_slave": LaunchConfiguration("left_slave"),
                "right_slave": LaunchConfiguration("right_slave"),
                "min_speed_rpm": LaunchConfiguration("min_speed_rpm"),
                "max_speed_rpm": LaunchConfiguration("max_speed_rpm"),
                "auto_start": LaunchConfiguration("auto_start"),
                "driver_command_timeout_sec": LaunchConfiguration("driver_command_timeout_sec"),
                "write_retry_count": LaunchConfiguration("write_retry_count"),
                "fail_safe_on_write_error": LaunchConfiguration("fail_safe_on_write_error"),
                "max_consecutive_write_errors": LaunchConfiguration("max_consecutive_write_errors"),
            }
        ],
    )

    # 最终返回完整的启动描述列表。
    return LaunchDescription(
        [
            device_arg,
            image_topic_arg,
            image_msg_type_arg,
            offset_px_topic_arg,
            offset_norm_topic_arg,
            serial_port_arg,
            track_width_arg,
            show_debug_arg,
            publish_debug_image_arg,
            show_angle_curve_arg,
            angle_curve_history_size_arg,
            angle_curve_limit_deg_arg,
            line_is_white_arg,
            blur_ksize_arg,
            thresh_arg,
            morph_ksize_arg,
            n_windows_arg,
            margin_arg,
            minpix_arg,
            kp_arg,
            roi_y_start_ratio_arg,
            min_component_area_arg,
            max_component_area_arg,
            max_component_width_px_arg,
            max_component_width_ratio_arg,
            component_intensity_limit_arg,
            max_visual_angle_deg_arg,
            heading_gain_arg,
            lateral_gain_arg,
            base_speed_ratio_arg,
            speed_reduce_gain_arg,
            allow_reverse_arg,
            min_output_rpm_arg,
            command_timeout_sec_arg,
            offset_timeout_sec_arg,
            angle_lowpass_alpha_arg,
            angle_deadband_deg_arg,
            max_motor_rpm_step_per_sec_arg,
            drive_wheel_diameter_arg,
            track_pitch_arg,
            track_link_count_arg,
            left_motor_sign_arg,
            right_motor_sign_arg,
            left_slave_arg,
            right_slave_arg,
            min_speed_rpm_arg,
            max_speed_rpm_arg,
            baud_rate_arg,
            serial_timeout_arg,
            auto_start_arg,
            driver_command_timeout_sec_arg,
            write_retry_count_arg,
            fail_safe_on_write_error_arg,
            max_consecutive_write_errors_arg,
            camera_launch,
            angle_process,
            model_process,
            driver_process,
        ]
    )
