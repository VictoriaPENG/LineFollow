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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 在安装后的包目录中找到复用的相机 launch 文件。
    usb_launch = PathJoinSubstitution([FindPackageShare("line_follow"), "launch", "usb_cam_web.launch.py"])
    line_follow_params_file_default = PathJoinSubstitution(
        [FindPackageShare("line_follow"), "config", "line_follow_params.yaml"]
    )
    runtime_params_file_default = PathJoinSubstitution(
        [FindPackageShare("line_follow"), "config", "runtime_params.yaml"]
    )

    # 以下都是可以在命令行覆盖的 launch 参数。
    line_follow_params_file_arg = DeclareLaunchArgument(
        "line_follow_params_file",
        default_value=line_follow_params_file_default,
        description="Line-follow vision and motor-model parameter YAML",
    )
    runtime_params_file_arg = DeclareLaunchArgument(
        "runtime_params_file",
        default_value=runtime_params_file_default,
        description="Runtime parameter YAML for motor driver and remote-control nodes",
    )
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="/dev/video2",
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
        default_value="50.0",
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
        default_value="180",
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
    angle_bias_deg_arg = DeclareLaunchArgument(
        "angle_bias_deg",
        default_value="1.0",
        description="Manual bias added to detected line angle before motor control",
    )
    roi_bottom_offset_ratio_arg = DeclareLaunchArgument(
        "roi_bottom_offset_ratio",
        default_value="0.25",
        description="Detect ROI bottom edge offset ratio measured upward from image bottom",
    )
    roi_height_ratio_arg = DeclareLaunchArgument(
        "roi_height_ratio",
        default_value="0.25",
        description="Detect ROI height ratio relative to image height",
    )
    min_component_area_arg = DeclareLaunchArgument(
        "min_component_area",
        default_value="20",
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
        default_value="50.0",
        description="Clamp visual angle input before motor model conversion",
    )
    heading_gain_arg = DeclareLaunchArgument(
        "heading_gain",
        default_value="20.0",
        description="How many RPM are added/removed per degree of heading error",
    )
    base_motor_rpm_arg = DeclareLaunchArgument(
        "base_motor_rpm",
        default_value="900.0",
        description="Fixed forward motor RPM used during line following",
    )
    line_follow_enabled_arg = DeclareLaunchArgument(
        "line_follow_enabled",
        default_value="true",
        description="Whether the motor model should actively publish line-follow speed commands",
    )
    allow_reverse_arg = DeclareLaunchArgument(
        "allow_reverse",
        default_value="false",
        description="Allow tracked chassis to reverse one side on sharp turns",
    )
    command_timeout_sec_arg = DeclareLaunchArgument(
        "command_timeout_sec",
        default_value="0.5",
        description="Timeout before motor model publishes stop command",
    )
    speed_update_period_sec_arg = DeclareLaunchArgument(
        "speed_update_period_sec",
        default_value="2.0",
        description="How often the motor model samples the latest angle and changes speed",
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
        default_value="2.5",
        description="Deadband applied to filtered visual angle in degrees",
    )
    straight_angle_epsilon_deg_arg = DeclareLaunchArgument(
        "straight_angle_epsilon_deg",
        default_value="1.5",
        description="Treat very small command angles as straight driving",
    )
    max_motor_rpm_step_per_sec_arg = DeclareLaunchArgument(
        "max_motor_rpm_step_per_sec",
        default_value="900.0",
        description="Maximum allowed RPM change per second for each motor",
    )
    max_motor_rpm_arg = DeclareLaunchArgument(
        "max_motor_rpm",
        default_value="1200.0",
        description="Hard cap for each motor rpm during line following, 0 disables limit",
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
    speed_status_topic_arg = DeclareLaunchArgument(
        "speed_status_topic",
        default_value="/motor_speed_status",
        description="Topic where the motor driver publishes applied motor RPM",
    )
    start_driver_arg = DeclareLaunchArgument(
        "start_driver",
        default_value="true",
        description="Whether to start the motor driver control node in this launch",
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
            "dashboard_speed_status_topic": LaunchConfiguration("speed_status_topic"),
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
                "angle_bias_deg": LaunchConfiguration("angle_bias_deg"),
                "roi_bottom_offset_ratio": LaunchConfiguration("roi_bottom_offset_ratio"),
                "roi_height_ratio": LaunchConfiguration("roi_height_ratio"),
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
            },
            LaunchConfiguration("line_follow_params_file"),
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
                "base_motor_rpm": LaunchConfiguration("base_motor_rpm"),
                "enabled": LaunchConfiguration("line_follow_enabled"),
                "allow_reverse": LaunchConfiguration("allow_reverse"),
                "command_timeout_sec": LaunchConfiguration("command_timeout_sec"),
                "speed_update_period_sec": LaunchConfiguration("speed_update_period_sec"),
                "angle_lowpass_alpha": LaunchConfiguration("angle_lowpass_alpha"),
                "angle_deadband_deg": LaunchConfiguration("angle_deadband_deg"),
                "straight_angle_epsilon_deg": LaunchConfiguration("straight_angle_epsilon_deg"),
                "max_motor_rpm_step_per_sec": LaunchConfiguration("max_motor_rpm_step_per_sec"),
                "max_motor_rpm": LaunchConfiguration("max_motor_rpm"),
                "drive_wheel_diameter_m": LaunchConfiguration("drive_wheel_diameter_m"),
                "track_pitch_m": LaunchConfiguration("track_pitch_m"),
                "track_link_count": LaunchConfiguration("track_link_count"),
                "left_motor_sign": LaunchConfiguration("left_motor_sign"),
                "right_motor_sign": LaunchConfiguration("right_motor_sign"),
            },
            LaunchConfiguration("line_follow_params_file"),
        ],
    )

    # Modbus 电机驱动控制节点。
    driver_process = Node(
        package="line_follow",
        executable="motor_driver_control_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_driver")),
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
                "speed_status_topic": LaunchConfiguration("speed_status_topic"),
                "driver_command_timeout_sec": LaunchConfiguration("driver_command_timeout_sec"),
                "write_retry_count": LaunchConfiguration("write_retry_count"),
                "fail_safe_on_write_error": LaunchConfiguration("fail_safe_on_write_error"),
                "max_consecutive_write_errors": LaunchConfiguration("max_consecutive_write_errors"),
            },
            LaunchConfiguration("runtime_params_file"),
        ],
    )

    # 最终返回完整的启动描述列表。
    return LaunchDescription(
        [
            line_follow_params_file_arg,
            runtime_params_file_arg,
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
            angle_bias_deg_arg,
            roi_bottom_offset_ratio_arg,
            roi_height_ratio_arg,
            min_component_area_arg,
            max_component_area_arg,
            max_component_width_px_arg,
            max_component_width_ratio_arg,
            component_intensity_limit_arg,
            max_visual_angle_deg_arg,
            heading_gain_arg,
            base_motor_rpm_arg,
            line_follow_enabled_arg,
            allow_reverse_arg,
            command_timeout_sec_arg,
            speed_update_period_sec_arg,
            angle_lowpass_alpha_arg,
            angle_deadband_deg_arg,
            straight_angle_epsilon_deg_arg,
            max_motor_rpm_step_per_sec_arg,
            max_motor_rpm_arg,
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
            speed_status_topic_arg,
            start_driver_arg,
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
