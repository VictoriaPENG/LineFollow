import os
from glob import glob

from setuptools import setup


package_name = "line_follow"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={package_name: "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "docs"),
            sorted(glob("Markdown/*.md")),
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.json"),
        ),
        (os.path.join("share", package_name, "board_tools"), glob("board_tools/*.sh")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="yunbo",
    maintainer_email="support@example.com",
    description="Vision-based line following package for OriginBot tracked platform.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "line_follow_angle_node = line_follow.angle_node:main",
            "line_follow_motor_model_node = line_follow.motor_model_node:main",
            "motor_driver_control_node = line_follow.motor_driver_control:main",
            "joystick_drive_node = line_follow.joystick_drive_node:main",
            "remote_relay_drive_node = line_follow.remote_relay_drive_node:main",
            "remote_long_press_start_line_follow_node = line_follow.remote_long_press_start_line_follow_node:main",
            "web_debug_dashboard_node = line_follow.web_debug_dashboard_node:main",
            "sim_cmd_vel_controller_node = line_follow.sim_cmd_vel_controller:main",
            "sim_motion_smoke_test_node = line_follow.sim_motion_smoke_test:main",
            "sim_motor_speed_to_cmd_vel_node = line_follow.sim_motor_speed_to_cmd_vel:main",
        ],
    },
)
