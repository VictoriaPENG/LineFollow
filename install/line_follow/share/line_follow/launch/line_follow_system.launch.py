#!/usr/bin/env python3
"""
巡线整系统启动文件。

本 launch 负责组合：
1. USB 相机与图像链路
2. 巡线核心链路（视觉检测 -> 运动模型 -> 驱动控制）

这样可以复用 `usb_cam_web.launch.py` 与 `line_follow_core.launch.py`，
避免参数声明和节点定义在多个 launch 文件里重复维护。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    usb_launch = PathJoinSubstitution([FindPackageShare("line_follow"), "launch", "usb_cam_web.launch.py"])
    core_launch = PathJoinSubstitution([FindPackageShare("line_follow"), "launch", "line_follow_core.launch.py"])

    line_follow_params_file_default = PathJoinSubstitution(
        [FindPackageShare("line_follow"), "config", "line_follow_params.yaml"]
    )
    runtime_params_file_default = PathJoinSubstitution(
        [FindPackageShare("line_follow"), "config", "runtime_params.yaml"]
    )

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
        default_value="/dev/video0",
        description="USB camera device path",
    )
    enable_websocket_arg = DeclareLaunchArgument(
        "enable_websocket",
        default_value="false",
        description="Whether to start websocket image page in the system stack",
    )
    enable_dashboard_arg = DeclareLaunchArgument(
        "enable_dashboard",
        default_value="false",
        description="Whether to start web debug dashboard in the system stack",
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
    visual_heading_error_topic_arg = DeclareLaunchArgument(
        "visual_heading_error_topic",
        default_value="/line_follow/visual_heading_error_deg",
        description="Calibrated visual heading error topic published by line follow angle node",
    )
    visual_lateral_error_px_topic_arg = DeclareLaunchArgument(
        "visual_lateral_error_px_topic",
        default_value="/line_follow/visual_lateral_error_px",
        description="Calibrated pixel-space lateral error topic published by line follow angle node",
    )
    visual_lateral_error_norm_topic_arg = DeclareLaunchArgument(
        "visual_lateral_error_norm_topic",
        default_value="/line_follow/visual_lateral_error_norm",
        description="Calibrated normalized lateral error topic used by motor model",
    )
    line_detected_topic_arg = DeclareLaunchArgument(
        "line_detected_topic",
        default_value="/line_follow/line_detected",
        description="Boolean topic published when line detection succeeds",
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
    debug_publish_every_n_arg = DeclareLaunchArgument(
        "debug_publish_every_n",
        default_value="3",
        description="Publish debug image topics once every N input frames",
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
        default_value="3.36",
        description="Manual bias added to detected line angle before motor control",
    )
    target_offset_px_arg = DeclareLaunchArgument(
        "target_offset_px",
        default_value="67.7",
        description="Reference pixel offset subtracted from raw line offset",
    )
    roi_bottom_offset_ratio_arg = DeclareLaunchArgument(
        "roi_bottom_offset_ratio",
        default_value="0.10",
        description="Detect ROI bottom edge offset ratio measured upward from image bottom",
    )
    roi_height_ratio_arg = DeclareLaunchArgument(
        "roi_height_ratio",
        default_value="0.5",
        description="Detect ROI height ratio relative to image height",
    )
    roi_side_margin_ratio_arg = DeclareLaunchArgument(
        "roi_side_margin_ratio",
        default_value="0.05",
        description="Detect ROI side margin ratio cropped from both left and right image edges",
    )
    roi_left_margin_ratio_arg = DeclareLaunchArgument(
        "roi_left_margin_ratio",
        default_value="0.15",
        description="Detect ROI margin cropped from the left image edge",
    )
    roi_right_margin_ratio_arg = DeclareLaunchArgument(
        "roi_right_margin_ratio",
        default_value="0.05",
        description="Detect ROI margin cropped from the right image edge",
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
        default_value="70.0",
        description="Clamp visual angle input before motor model conversion",
    )
    heading_gain_arg = DeclareLaunchArgument(
        "heading_gain",
        default_value="20.0",
        description="How many RPM are added/removed per degree of heading error",
    )
    offset_gain_deg_arg = DeclareLaunchArgument(
        "offset_gain_deg",
        default_value="8.0",
        description="Degrees of equivalent heading correction per normalized lateral error",
    )
    offset_priority_threshold_norm_arg = DeclareLaunchArgument(
        "offset_priority_threshold_norm",
        default_value="0.18",
        description="Below this normalized offset, use the full visual heading term",
    )
    offset_priority_full_norm_arg = DeclareLaunchArgument(
        "offset_priority_full_norm",
        default_value="0.30",
        description="Above this normalized offset, prioritize lateral correction",
    )
    offset_priority_min_heading_scale_arg = DeclareLaunchArgument(
        "offset_priority_min_heading_scale",
        default_value="0.25",
        description="Minimum heading term scale when lateral offset is large",
    )
    base_motor_rpm_arg = DeclareLaunchArgument(
        "base_motor_rpm",
        default_value="400.0",
        description="Fixed forward motor RPM used during line following",
    )
    slow_base_motor_rpm_arg = DeclareLaunchArgument(
        "slow_base_motor_rpm",
        default_value="400.0",
        description="Compatibility parameter; center RPM is fixed by base_motor_rpm",
    )
    severe_base_motor_rpm_arg = DeclareLaunchArgument(
        "severe_base_motor_rpm",
        default_value="400.0",
        description="Compatibility parameter; center RPM is fixed by base_motor_rpm",
    )
    slow_speed_offset_enter_norm_arg = DeclareLaunchArgument(
        "slow_speed_offset_enter_norm",
        default_value="0.20",
        description="Normalized lateral offset that enters reduced-speed line following",
    )
    slow_speed_offset_exit_norm_arg = DeclareLaunchArgument(
        "slow_speed_offset_exit_norm",
        default_value="0.10",
        description="Normalized lateral offset that exits reduced-speed line following",
    )
    slow_speed_angle_enter_deg_arg = DeclareLaunchArgument(
        "slow_speed_angle_enter_deg",
        default_value="15.0",
        description="Heading angle that enters reduced-speed line following",
    )
    slow_speed_angle_exit_deg_arg = DeclareLaunchArgument(
        "slow_speed_angle_exit_deg",
        default_value="8.0",
        description="Heading angle that exits reduced-speed line following",
    )
    severe_speed_offset_enter_norm_arg = DeclareLaunchArgument(
        "severe_speed_offset_enter_norm",
        default_value="0.40",
        description="Normalized lateral offset that enters severe reduced-speed line following",
    )
    severe_speed_offset_exit_norm_arg = DeclareLaunchArgument(
        "severe_speed_offset_exit_norm",
        default_value="0.25",
        description="Normalized lateral offset that exits severe reduced-speed line following",
    )
    severe_speed_angle_enter_deg_arg = DeclareLaunchArgument(
        "severe_speed_angle_enter_deg",
        default_value="28.0",
        description="Heading angle that enters severe reduced-speed line following",
    )
    severe_speed_angle_exit_deg_arg = DeclareLaunchArgument(
        "severe_speed_angle_exit_deg",
        default_value="18.0",
        description="Heading angle that exits severe reduced-speed line following",
    )
    slow_speed_turn_boost_arg = DeclareLaunchArgument(
        "slow_speed_turn_boost",
        default_value="2.4",
        description="Differential steering multiplier while reduced-speed line following is active",
    )
    max_delta_motor_rpm_arg = DeclareLaunchArgument(
        "max_delta_motor_rpm",
        default_value="1600.0",
        description="Clamp for differential steering RPM, 0 disables the dedicated clamp",
    )
    severe_speed_turn_boost_arg = DeclareLaunchArgument(
        "severe_speed_turn_boost",
        default_value="3.4",
        description="Differential steering multiplier while severe reduced-speed line following is active",
    )
    stability_angle_rate_slow_deg_s_arg = DeclareLaunchArgument(
        "stability_angle_rate_slow_deg_s",
        default_value="80.0",
        description="Compatibility parameter; rate no longer changes center speed",
    )
    stability_angle_rate_full_deg_s_arg = DeclareLaunchArgument(
        "stability_angle_rate_full_deg_s",
        default_value="180.0",
        description="Compatibility parameter; rate no longer changes center speed",
    )
    stability_offset_rate_slow_norm_s_arg = DeclareLaunchArgument(
        "stability_offset_rate_slow_norm_s",
        default_value="0.8",
        description="Compatibility parameter; rate no longer changes center speed",
    )
    stability_offset_rate_full_norm_s_arg = DeclareLaunchArgument(
        "stability_offset_rate_full_norm_s",
        default_value="1.8",
        description="Compatibility parameter; rate no longer changes center speed",
    )
    adaptive_speed_alpha_arg = DeclareLaunchArgument(
        "adaptive_speed_alpha",
        default_value="0.55",
        description="Low-pass alpha for adaptive center speed",
    )
    steering_nonlinearity_arg = DeclareLaunchArgument(
        "steering_nonlinearity",
        default_value="1.25",
        description="Nonlinear steering exponent used for large turns",
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
        default_value="0.015",
        description="How often the motor model samples the latest angle and changes speed",
    )
    offset_timeout_sec_arg = DeclareLaunchArgument(
        "offset_timeout_sec",
        default_value="0.3",
        description="How long the motor model keeps using the last lateral offset sample",
    )
    angle_lowpass_alpha_arg = DeclareLaunchArgument(
        "angle_lowpass_alpha",
        default_value="0.55",
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
        default_value="4000.0",
        description="Maximum allowed RPM change per second for each motor",
    )
    max_motor_rpm_arg = DeclareLaunchArgument(
        "max_motor_rpm",
        default_value="1200.0",
        description="Hard cap for each motor rpm during line following, 0 disables limit",
    )
    arm_on_enable_detection_count_arg = DeclareLaunchArgument(
        "arm_on_enable_detection_count",
        default_value="0",
        description="Consecutive detected frames required before motor output is enabled",
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

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(usb_launch),
        launch_arguments={
            "device": LaunchConfiguration("device"),
            "enable_websocket": LaunchConfiguration("enable_websocket"),
            "enable_dashboard": LaunchConfiguration("enable_dashboard"),
            "dashboard_speed_status_topic": LaunchConfiguration("speed_status_topic"),
        }.items(),
    )

    core_process = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_launch),
        launch_arguments={
            "line_follow_params_file": LaunchConfiguration("line_follow_params_file"),
            "runtime_params_file": LaunchConfiguration("runtime_params_file"),
            "image_topic": LaunchConfiguration("image_topic"),
            "image_msg_type": LaunchConfiguration("image_msg_type"),
            "offset_px_topic": LaunchConfiguration("offset_px_topic"),
            "offset_norm_topic": LaunchConfiguration("offset_norm_topic"),
            "visual_heading_error_topic": LaunchConfiguration("visual_heading_error_topic"),
            "visual_lateral_error_px_topic": LaunchConfiguration("visual_lateral_error_px_topic"),
            "visual_lateral_error_norm_topic": LaunchConfiguration("visual_lateral_error_norm_topic"),
            "line_detected_topic": LaunchConfiguration("line_detected_topic"),
            "serial_port": LaunchConfiguration("serial_port"),
            "track_width_m": LaunchConfiguration("track_width_m"),
            "show_debug": LaunchConfiguration("show_debug"),
            "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            "debug_publish_every_n": LaunchConfiguration("debug_publish_every_n"),
            "show_angle_curve": LaunchConfiguration("show_angle_curve"),
            "angle_curve_history_size": LaunchConfiguration("angle_curve_history_size"),
            "angle_curve_limit_deg": LaunchConfiguration("angle_curve_limit_deg"),
            "line_is_white": LaunchConfiguration("line_is_white"),
            "blur_ksize": LaunchConfiguration("blur_ksize"),
            "thresh": LaunchConfiguration("thresh"),
            "morph_ksize": LaunchConfiguration("morph_ksize"),
            "n_windows": LaunchConfiguration("n_windows"),
            "margin": LaunchConfiguration("margin"),
            "minpix": LaunchConfiguration("minpix"),
            "kp": LaunchConfiguration("kp"),
            "angle_bias_deg": LaunchConfiguration("angle_bias_deg"),
            "target_offset_px": LaunchConfiguration("target_offset_px"),
            "roi_bottom_offset_ratio": LaunchConfiguration("roi_bottom_offset_ratio"),
            "roi_height_ratio": LaunchConfiguration("roi_height_ratio"),
            "roi_side_margin_ratio": LaunchConfiguration("roi_side_margin_ratio"),
            "roi_left_margin_ratio": LaunchConfiguration("roi_left_margin_ratio"),
            "roi_right_margin_ratio": LaunchConfiguration("roi_right_margin_ratio"),
            "min_component_area": LaunchConfiguration("min_component_area"),
            "max_component_area": LaunchConfiguration("max_component_area"),
            "max_component_width_px": LaunchConfiguration("max_component_width_px"),
            "max_component_width_ratio": LaunchConfiguration("max_component_width_ratio"),
            "component_intensity_limit": LaunchConfiguration("component_intensity_limit"),
            "max_visual_angle_deg": LaunchConfiguration("max_visual_angle_deg"),
            "heading_gain": LaunchConfiguration("heading_gain"),
            "offset_gain_deg": LaunchConfiguration("offset_gain_deg"),
            "offset_priority_threshold_norm": LaunchConfiguration("offset_priority_threshold_norm"),
            "offset_priority_full_norm": LaunchConfiguration("offset_priority_full_norm"),
            "offset_priority_min_heading_scale": LaunchConfiguration("offset_priority_min_heading_scale"),
            "base_motor_rpm": LaunchConfiguration("base_motor_rpm"),
            "slow_base_motor_rpm": LaunchConfiguration("slow_base_motor_rpm"),
            "severe_base_motor_rpm": LaunchConfiguration("severe_base_motor_rpm"),
            "slow_speed_offset_enter_norm": LaunchConfiguration("slow_speed_offset_enter_norm"),
            "slow_speed_offset_exit_norm": LaunchConfiguration("slow_speed_offset_exit_norm"),
            "slow_speed_angle_enter_deg": LaunchConfiguration("slow_speed_angle_enter_deg"),
            "slow_speed_angle_exit_deg": LaunchConfiguration("slow_speed_angle_exit_deg"),
            "severe_speed_offset_enter_norm": LaunchConfiguration("severe_speed_offset_enter_norm"),
            "severe_speed_offset_exit_norm": LaunchConfiguration("severe_speed_offset_exit_norm"),
            "severe_speed_angle_enter_deg": LaunchConfiguration("severe_speed_angle_enter_deg"),
            "severe_speed_angle_exit_deg": LaunchConfiguration("severe_speed_angle_exit_deg"),
            "slow_speed_turn_boost": LaunchConfiguration("slow_speed_turn_boost"),
            "severe_speed_turn_boost": LaunchConfiguration("severe_speed_turn_boost"),
            "stability_angle_rate_slow_deg_s": LaunchConfiguration("stability_angle_rate_slow_deg_s"),
            "stability_angle_rate_full_deg_s": LaunchConfiguration("stability_angle_rate_full_deg_s"),
            "stability_offset_rate_slow_norm_s": LaunchConfiguration("stability_offset_rate_slow_norm_s"),
            "stability_offset_rate_full_norm_s": LaunchConfiguration("stability_offset_rate_full_norm_s"),
            "adaptive_speed_alpha": LaunchConfiguration("adaptive_speed_alpha"),
            "steering_nonlinearity": LaunchConfiguration("steering_nonlinearity"),
            "max_delta_motor_rpm": LaunchConfiguration("max_delta_motor_rpm"),
            "line_follow_enabled": LaunchConfiguration("line_follow_enabled"),
            "allow_reverse": LaunchConfiguration("allow_reverse"),
            "command_timeout_sec": LaunchConfiguration("command_timeout_sec"),
            "speed_update_period_sec": LaunchConfiguration("speed_update_period_sec"),
            "offset_timeout_sec": LaunchConfiguration("offset_timeout_sec"),
            "angle_lowpass_alpha": LaunchConfiguration("angle_lowpass_alpha"),
            "angle_deadband_deg": LaunchConfiguration("angle_deadband_deg"),
            "straight_angle_epsilon_deg": LaunchConfiguration("straight_angle_epsilon_deg"),
            "max_motor_rpm_step_per_sec": LaunchConfiguration("max_motor_rpm_step_per_sec"),
            "max_motor_rpm": LaunchConfiguration("max_motor_rpm"),
            "arm_on_enable_detection_count": LaunchConfiguration("arm_on_enable_detection_count"),
            "drive_wheel_diameter_m": LaunchConfiguration("drive_wheel_diameter_m"),
            "track_pitch_m": LaunchConfiguration("track_pitch_m"),
            "track_link_count": LaunchConfiguration("track_link_count"),
            "left_motor_sign": LaunchConfiguration("left_motor_sign"),
            "right_motor_sign": LaunchConfiguration("right_motor_sign"),
            "left_slave": LaunchConfiguration("left_slave"),
            "right_slave": LaunchConfiguration("right_slave"),
            "min_speed_rpm": LaunchConfiguration("min_speed_rpm"),
            "max_speed_rpm": LaunchConfiguration("max_speed_rpm"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "serial_timeout_sec": LaunchConfiguration("serial_timeout_sec"),
            "auto_start": LaunchConfiguration("auto_start"),
            "speed_status_topic": LaunchConfiguration("speed_status_topic"),
            "start_driver": LaunchConfiguration("start_driver"),
            "driver_command_timeout_sec": LaunchConfiguration("driver_command_timeout_sec"),
            "write_retry_count": LaunchConfiguration("write_retry_count"),
            "fail_safe_on_write_error": LaunchConfiguration("fail_safe_on_write_error"),
            "max_consecutive_write_errors": LaunchConfiguration("max_consecutive_write_errors"),
        }.items(),
    )

    return LaunchDescription(
        [
            line_follow_params_file_arg,
            runtime_params_file_arg,
            device_arg,
            enable_websocket_arg,
            enable_dashboard_arg,
            image_topic_arg,
            image_msg_type_arg,
            offset_px_topic_arg,
            offset_norm_topic_arg,
            visual_heading_error_topic_arg,
            visual_lateral_error_px_topic_arg,
            visual_lateral_error_norm_topic_arg,
            line_detected_topic_arg,
            serial_port_arg,
            track_width_arg,
            show_debug_arg,
            publish_debug_image_arg,
            debug_publish_every_n_arg,
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
            target_offset_px_arg,
            roi_bottom_offset_ratio_arg,
            roi_height_ratio_arg,
            roi_side_margin_ratio_arg,
            roi_left_margin_ratio_arg,
            roi_right_margin_ratio_arg,
            min_component_area_arg,
            max_component_area_arg,
            max_component_width_px_arg,
            max_component_width_ratio_arg,
            component_intensity_limit_arg,
            max_visual_angle_deg_arg,
            heading_gain_arg,
            offset_gain_deg_arg,
            offset_priority_threshold_norm_arg,
            offset_priority_full_norm_arg,
            offset_priority_min_heading_scale_arg,
            base_motor_rpm_arg,
            slow_base_motor_rpm_arg,
            severe_base_motor_rpm_arg,
            slow_speed_offset_enter_norm_arg,
            slow_speed_offset_exit_norm_arg,
            slow_speed_angle_enter_deg_arg,
            slow_speed_angle_exit_deg_arg,
            severe_speed_offset_enter_norm_arg,
            severe_speed_offset_exit_norm_arg,
            severe_speed_angle_enter_deg_arg,
            severe_speed_angle_exit_deg_arg,
            slow_speed_turn_boost_arg,
            severe_speed_turn_boost_arg,
            stability_angle_rate_slow_deg_s_arg,
            stability_angle_rate_full_deg_s_arg,
            stability_offset_rate_slow_norm_s_arg,
            stability_offset_rate_full_norm_s_arg,
            adaptive_speed_alpha_arg,
            steering_nonlinearity_arg,
            max_delta_motor_rpm_arg,
            line_follow_enabled_arg,
            allow_reverse_arg,
            command_timeout_sec_arg,
            offset_timeout_sec_arg,
            speed_update_period_sec_arg,
            angle_lowpass_alpha_arg,
            angle_deadband_deg_arg,
            straight_angle_epsilon_deg_arg,
            max_motor_rpm_step_per_sec_arg,
            max_motor_rpm_arg,
            arm_on_enable_detection_count_arg,
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
            core_process,
        ]
    )
