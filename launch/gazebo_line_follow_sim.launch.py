#!/usr/bin/env python3
"""Launch the Gazebo visual line-follow closed-loop simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def float_config(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def int_config(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x")
    y_pose = LaunchConfiguration("y")
    z_pose = LaunchConfiguration("z")
    world = LaunchConfiguration("world")

    pkg_share = FindPackageShare("line_follow")
    gazebo_ros_share = FindPackageShare("gazebo_ros")

    robot_description_file = PathJoinSubstitution(
        [pkg_share, "urdf", "line_follow_car.urdf.xacro"]
    )
    world_file = PathJoinSubstitution([pkg_share, "worlds", "line_follow_track.world"])
    line_follow_params_file = PathJoinSubstitution(
        [pkg_share, "config", "line_follow_params.yaml"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_description_file]),
            value_type=str,
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": world,
            "extra_gazebo_args": "-s libgazebo_ros_state.so",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "line_follow_car",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
            "-timeout",
            "60",
        ],
        output="screen",
    )

    angle_process = Node(
        package="line_follow",
        executable="line_follow_angle_node",
        name="line_follow_angle_node",
        output="screen",
        parameters=[
            line_follow_params_file,
            {
                "use_sim_time": use_sim_time,
                "image_topic": LaunchConfiguration("sim_image_topic"),
                "image_msg_type": "raw",
                "enable_camera_undistort": False,
                "detection_method": LaunchConfiguration("detection_method"),
                "line_is_white": False,
                "thresh": LaunchConfiguration("thresh"),
                "morph_ksize": LaunchConfiguration("morph_ksize"),
                "margin": LaunchConfiguration("margin"),
                "minpix": LaunchConfiguration("minpix"),
                "angle_bias_deg": float_config("angle_bias_deg"),
                "target_offset_px": float_config("target_offset_px"),
                "roi_bottom_offset_ratio": float_config("roi_bottom_offset_ratio"),
                "roi_height_ratio": float_config("roi_height_ratio"),
                "roi_left_margin_ratio": float_config("roi_left_margin_ratio"),
                "roi_right_margin_ratio": float_config("roi_right_margin_ratio"),
                "base_search_half_width_ratio": float_config("base_search_half_width_ratio"),
                "publish_debug_image": True,
                "debug_publish_every_n": 1,
            },
        ],
    )

    motor_model = Node(
        package="line_follow",
        executable="line_follow_motor_model_node",
        name="line_follow_motor_model_node",
        output="screen",
        parameters=[
            line_follow_params_file,
            {
                "use_sim_time": use_sim_time,
                "angle_topic": "/line_follow/visual_heading_error_deg",
                "offset_norm_topic": "/line_follow/visual_lateral_error_norm",
                "line_detected_topic": "/line_follow/line_detected",
                "speed_topic": "/motor_speed_cmd",
                "base_motor_rpm": float_config("base_motor_rpm"),
                "max_motor_rpm": float_config("max_motor_rpm"),
                "max_delta_motor_rpm": float_config("max_delta_motor_rpm"),
                "max_motor_rpm_step_per_sec": float_config("max_motor_rpm_step_per_sec"),
                "heading_gain": float_config("heading_gain"),
                "offset_gain_deg": float_config("offset_gain_deg"),
                "offset_priority_threshold_norm": float_config("offset_priority_threshold_norm"),
                "offset_priority_full_norm": float_config("offset_priority_full_norm"),
                "offset_priority_min_heading_scale": float_config("offset_priority_min_heading_scale"),
                "angle_lowpass_alpha": float_config("angle_lowpass_alpha"),
                "angle_deadband_deg": float_config("angle_deadband_deg"),
                "slow_speed_turn_boost": float_config("slow_speed_turn_boost"),
                "severe_speed_turn_boost": float_config("severe_speed_turn_boost"),
                "steering_nonlinearity": float_config("steering_nonlinearity"),
                "command_timeout_sec": float_config("motor_command_timeout_sec"),
                "fallback_straight_when_no_angle": False,
                "left_motor_sign": float_config("left_motor_sign"),
                "right_motor_sign": float_config("right_motor_sign"),
            },
        ],
    )

    rpm_bridge = Node(
        package="line_follow",
        executable="sim_motor_speed_to_cmd_vel_node",
        name="sim_motor_speed_to_cmd_vel_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "speed_topic": "/motor_speed_cmd",
                "cmd_vel_topic": "/cmd_vel",
                "track_width_m": float_config("track_width_m"),
                "drive_wheel_diameter_m": float_config("drive_wheel_diameter_m"),
                "gear_ratio": float_config("gear_ratio"),
                "left_motor_sign": float_config("left_motor_sign"),
                "right_motor_sign": float_config("right_motor_sign"),
                "linear_scale": float_config("linear_scale"),
                "angular_scale": float_config("angular_scale"),
                "command_timeout_sec": float_config("bridge_command_timeout_sec"),
            }
        ],
    )

    sim_cmd_vel_controller = Node(
        package="line_follow",
        executable="sim_cmd_vel_controller_node",
        name="sim_cmd_vel_controller_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "entity_name": "line_follow_car",
                "cmd_vel_topic": "/cmd_vel",
                "command_timeout_sec": 0.6,
                "initial_x": x_pose,
                "initial_y": y_pose,
                "initial_z": z_pose,
            }
        ],
    )

    web_debug_dashboard = Node(
        package="line_follow",
        executable="web_debug_dashboard_node",
        name="web_debug_dashboard_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_dashboard")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "port": int_config("dashboard_port"),
                "open_browser": False,
                "source_topic": LaunchConfiguration("sim_image_topic"),
                "source_topic_type": "raw",
                "detect_topic": "/line_follow/debug_detect",
                "binary_topic": "/line_follow/debug_binary",
                "curve_topic": "/line_follow/debug_angle_curve",
                "speed_cmd_topic": "/motor_speed_cmd",
                "speed_status_topic": "/motor_speed_cmd",
                "line_detected_topic": "/line_follow/line_detected",
                "heading_error_topic": "/line_follow/visual_heading_error_deg",
                "lateral_error_topic": "/line_follow/visual_lateral_error_norm",
                "cmd_vel_topic": "/cmd_vel",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock",
            ),
            DeclareLaunchArgument("x", default_value="-1.5", description="Initial robot x"),
            DeclareLaunchArgument("y", default_value="0.0", description="Initial robot y"),
            DeclareLaunchArgument("z", default_value="0.0", description="Initial robot z"),
            DeclareLaunchArgument(
                "world",
                default_value=world_file,
                description="Gazebo track world file",
            ),
            DeclareLaunchArgument(
                "sim_image_topic",
                default_value="/sim/camera/line_follow_camera/image_raw",
                description="Gazebo camera image topic",
            ),
            DeclareLaunchArgument(
                "detection_method",
                default_value="threshold",
                description="Vision binary method used in simulation",
            ),
            DeclareLaunchArgument("thresh", default_value="120", description="Black-line threshold"),
            DeclareLaunchArgument("morph_ksize", default_value="3", description="Morphology kernel"),
            DeclareLaunchArgument("margin", default_value="80", description="Sliding-window half width"),
            DeclareLaunchArgument("minpix", default_value="20", description="Minimum pixels per window"),
            DeclareLaunchArgument("angle_bias_deg", default_value="0.0", description="Sim angle bias"),
            DeclareLaunchArgument("target_offset_px", default_value="0.0", description="Sim target offset"),
            DeclareLaunchArgument(
                "roi_bottom_offset_ratio",
                default_value="0.0",
                description="Detection ROI bottom offset ratio",
            ),
            DeclareLaunchArgument(
                "roi_height_ratio",
                default_value="0.70",
                description="Detection ROI height ratio",
            ),
            DeclareLaunchArgument(
                "roi_left_margin_ratio",
                default_value="0.08",
                description="Detection ROI left margin ratio",
            ),
            DeclareLaunchArgument(
                "roi_right_margin_ratio",
                default_value="0.08",
                description="Detection ROI right margin ratio",
            ),
            DeclareLaunchArgument(
                "base_search_half_width_ratio",
                default_value="0.25",
                description="Initial bottom histogram search half width ratio",
            ),
            DeclareLaunchArgument("base_motor_rpm", default_value="450.0", description="Sim base motor RPM"),
            DeclareLaunchArgument("max_motor_rpm", default_value="900.0", description="Sim motor RPM cap"),
            DeclareLaunchArgument(
                "max_delta_motor_rpm",
                default_value="700.0",
                description="Sim differential RPM cap",
            ),
            DeclareLaunchArgument(
                "max_motor_rpm_step_per_sec",
                default_value="2500.0",
                description="Sim RPM slew-rate limit",
            ),
            DeclareLaunchArgument("heading_gain", default_value="20.0", description="Heading gain"),
            DeclareLaunchArgument("offset_gain_deg", default_value="8.0", description="Lateral offset gain"),
            DeclareLaunchArgument(
                "offset_priority_threshold_norm",
                default_value="0.18",
                description="Offset-priority start threshold",
            ),
            DeclareLaunchArgument(
                "offset_priority_full_norm",
                default_value="0.30",
                description="Offset-priority full threshold",
            ),
            DeclareLaunchArgument(
                "offset_priority_min_heading_scale",
                default_value="0.25",
                description="Minimum heading scale while offset priority is active",
            ),
            DeclareLaunchArgument(
                "angle_lowpass_alpha",
                default_value="0.55",
                description="Angle low-pass filter alpha",
            ),
            DeclareLaunchArgument(
                "angle_deadband_deg",
                default_value="2.5",
                description="Angle deadband in degrees",
            ),
            DeclareLaunchArgument(
                "slow_speed_turn_boost",
                default_value="2.4",
                description="Moderate-error differential steering boost",
            ),
            DeclareLaunchArgument(
                "severe_speed_turn_boost",
                default_value="3.4",
                description="Severe-error differential steering boost",
            ),
            DeclareLaunchArgument(
                "steering_nonlinearity",
                default_value="1.25",
                description="Nonlinear steering exponent",
            ),
            DeclareLaunchArgument(
                "motor_command_timeout_sec",
                default_value="0.4",
                description="Motor model vision command timeout",
            ),
            DeclareLaunchArgument(
                "bridge_command_timeout_sec",
                default_value="0.5",
                description="RPM bridge command timeout",
            ),
            DeclareLaunchArgument("track_width_m", default_value="0.577", description="Track width"),
            DeclareLaunchArgument(
                "drive_wheel_diameter_m",
                default_value="0.18",
                description="Drive wheel diameter",
            ),
            DeclareLaunchArgument("gear_ratio", default_value=str(14.0 / 1.27), description="Gear ratio"),
            DeclareLaunchArgument("left_motor_sign", default_value="1.0", description="Left motor sign"),
            DeclareLaunchArgument("right_motor_sign", default_value="-1.0", description="Right motor sign"),
            DeclareLaunchArgument("linear_scale", default_value="1.0", description="Linear speed scale"),
            DeclareLaunchArgument("angular_scale", default_value="1.0", description="Angular speed scale"),
            DeclareLaunchArgument(
                "start_dashboard",
                default_value="true",
                description="Start the web debug dashboard",
            ),
            DeclareLaunchArgument(
                "dashboard_port",
                default_value="8091",
                description="Web debug dashboard port",
            ),
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            angle_process,
            motor_model,
            rpm_bridge,
            sim_cmd_vel_controller,
            web_debug_dashboard,
        ]
    )
