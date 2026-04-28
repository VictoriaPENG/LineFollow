# Copyright (c) 2024, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
USB 相机与网页调试链路启动文件。

本 launch 负责：
1. 启动 USB 相机节点，把相机图像发布到 ROS
2. 可选启动 websocket 图像页面
3. 启动 JPEG -> NV12 的解码链路，供板端共享内存消费
4. 启动多画面 Web 调试页
5. 在相机启动后延时设置亮度，避免设备尚未就绪时配置失败
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    """生成 USB 相机、图像解码和网页调试相关节点。"""
    cam_node = None
    camera_device_arg = None

    # 相机输入和网页展示的基础参数。
    usb_cam_device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video0',
        description='usb camera device')
    websocket_image_topic_arg = DeclareLaunchArgument(
        'websocket_image_topic',
        default_value='/image',
        description='image topic shown on the web page')
    websocket_image_type_arg = DeclareLaunchArgument(
        'websocket_image_type',
        default_value='mjpeg',
        description='websocket output image type')
    websocket_only_show_image_arg = DeclareLaunchArgument(
        'websocket_only_show_image',
        default_value='True',
        description='whether the web page only shows the image stream')
    camera_brightness_arg = DeclareLaunchArgument(
        'camera_brightness',
        default_value='15',
        description='brightness value applied to the camera via v4l2-ctl')
    dashboard_source_topic_type_arg = DeclareLaunchArgument(
        'dashboard_source_topic_type',
        default_value='raw',
        description='source topic type for dashboard: compressed or raw')
    dashboard_bind_host_arg = DeclareLaunchArgument(
        'dashboard_bind_host',
        default_value='0.0.0.0',
        description='bind host for multi-view dashboard')
    dashboard_port_arg = DeclareLaunchArgument(
        'dashboard_port',
        default_value='8091',
        description='port for multi-view dashboard')
    dashboard_open_browser_arg = DeclareLaunchArgument(
        'dashboard_open_browser',
        default_value='True',
        description='auto open browser for multi-view dashboard when desktop is available')
    dashboard_source_topic_arg = DeclareLaunchArgument(
        'dashboard_source_topic',
        default_value='/line_follow/debug_undistorted',
        description='source image topic shown in dashboard')
    dashboard_detect_topic_arg = DeclareLaunchArgument(
        'dashboard_detect_topic',
        default_value='/line_follow/debug_detect',
        description='detect debug image topic shown in dashboard')
    dashboard_binary_topic_arg = DeclareLaunchArgument(
        'dashboard_binary_topic',
        default_value='/line_follow/debug_binary',
        description='binary debug image topic shown in dashboard')
    dashboard_curve_topic_arg = DeclareLaunchArgument(
        'dashboard_curve_topic',
        default_value='/line_follow/debug_angle_curve',
        description='curve debug image topic shown in dashboard')
    dashboard_speed_status_topic_arg = DeclareLaunchArgument(
        'dashboard_speed_status_topic',
        default_value='/motor_speed_status',
        description='motor speed status topic shown in dashboard')
    enable_websocket_arg = DeclareLaunchArgument(
        'enable_websocket',
        default_value='true',
        description='whether to start websocket image page')
    enable_dashboard_arg = DeclareLaunchArgument(
        'enable_dashboard',
        default_value='true',
        description='whether to start multi-view debug dashboard')

    usb_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_usb_cam'),
                'launch/hobot_usb_cam.launch.py')),
        launch_arguments={
            'usb_video_device': LaunchConfiguration('device')
        }.items()
    )
    print("using usb cam")
    cam_node = usb_node
    camera_device_arg = usb_cam_device_arg

    camera_control_node = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    '/bin/bash',
                    '/userdata/dev_ws/src/originbot/Line_follow/board_tools/set_camera_brightness.sh',
                    LaunchConfiguration('device'),
                    LaunchConfiguration('camera_brightness'),
                ],
                shell=False,
            )
        ],
    )

    # 老的 websocket 图像页，适合只看单路原图。
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_websocket')),
        launch_arguments={
            'websocket_image_topic': LaunchConfiguration('websocket_image_topic'),
            'websocket_image_type': LaunchConfiguration('websocket_image_type'),
            'websocket_only_show_image': LaunchConfiguration('websocket_only_show_image')
        }.items()
    )

    # 巡线节点默认消费板端共享内存 NV12，因此这里把 JPEG 解码后转发到 `/hbmem_img`。
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_shm'),
                'launch/hobot_shm.launch.py'))
    )
    dashboard_node = Node(
        package='line_follow',
        executable='web_debug_dashboard_node',
        condition=IfCondition(LaunchConfiguration('enable_dashboard')),
        output='screen',
        parameters=[
            {
                'bind_host': LaunchConfiguration('dashboard_bind_host'),
                'port': LaunchConfiguration('dashboard_port'),
                'open_browser': LaunchConfiguration('dashboard_open_browser'),
                'source_topic': LaunchConfiguration('dashboard_source_topic'),
                'source_topic_type': LaunchConfiguration('dashboard_source_topic_type'),
                'detect_topic': LaunchConfiguration('dashboard_detect_topic'),
                'binary_topic': LaunchConfiguration('dashboard_binary_topic'),
                'curve_topic': LaunchConfiguration('dashboard_curve_topic'),
                'speed_status_topic': LaunchConfiguration('dashboard_speed_status_topic'),
            }
        ],
    )

    return LaunchDescription([
        camera_device_arg,
        websocket_image_topic_arg,
        websocket_image_type_arg,
        websocket_only_show_image_arg,
        camera_brightness_arg,
        dashboard_source_topic_type_arg,
        dashboard_bind_host_arg,
        dashboard_port_arg,
        dashboard_open_browser_arg,
        dashboard_source_topic_arg,
        dashboard_detect_topic_arg,
        dashboard_binary_topic_arg,
        dashboard_curve_topic_arg,
        dashboard_speed_status_topic_arg,
        enable_websocket_arg,
        enable_dashboard_arg,
        # 相机图像发布链路。
        cam_node,
        camera_control_node,
        # 单路 websocket 图像页。
        web_node,
        # JPEG -> NV12 解码链路。
        nv12_codec_node, 
        # 启动零拷贝共享内存环境。
        shared_mem_node,
        # 多画面调试页。
        dashboard_node,
    ])
