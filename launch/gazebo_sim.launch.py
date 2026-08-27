#!/usr/bin/env python3
"""Launch the line-follow car model in Gazebo Classic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    world_file = PathJoinSubstitution([pkg_share, "worlds", "line_follow_empty.world"])

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

    sim_cmd_vel_controller = Node(
        package="line_follow",
        executable="sim_cmd_vel_controller_node",
        name="sim_cmd_vel_controller_node",
        output="screen",
        parameters=[
            {
                "entity_name": "line_follow_car",
                "cmd_vel_topic": "/cmd_vel",
                "command_timeout_sec": 2.0,
                "initial_x": x_pose,
                "initial_y": y_pose,
                "initial_z": z_pose,
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
            DeclareLaunchArgument(
                "x",
                default_value="0.0",
                description="Initial robot x position in meters",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="0.0",
                description="Initial robot y position in meters",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.05",
                description="Initial robot z position in meters",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=world_file,
                description="Gazebo world file",
            ),
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            sim_cmd_vel_controller,
        ]
    )
