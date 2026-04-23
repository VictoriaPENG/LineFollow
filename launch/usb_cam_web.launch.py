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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    cam_node = None
    camera_device_arg = None

    # usb cam图片发布pkg
    usb_cam_device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video2',
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
        default_value='/image',
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

    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': LaunchConfiguration('websocket_image_topic'),
            'websocket_image_type': LaunchConfiguration('websocket_image_type'),
            'websocket_only_show_image': LaunchConfiguration('websocket_only_show_image')
        }.items()
    )

    # jpeg->nv12
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
        output='screen',
        parameters=[
            {
                'bind_host': LaunchConfiguration('dashboard_bind_host'),
                'port': LaunchConfiguration('dashboard_port'),
                'open_browser': LaunchConfiguration('dashboard_open_browser'),
                'source_topic': LaunchConfiguration('dashboard_source_topic'),
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
        dashboard_bind_host_arg,
        dashboard_port_arg,
        dashboard_open_browser_arg,
        dashboard_source_topic_arg,
        dashboard_detect_topic_arg,
        dashboard_binary_topic_arg,
        dashboard_curve_topic_arg,
        dashboard_speed_status_topic_arg,
        # 图片发布pkg
        cam_node,
        # web展示pkg
        web_node,
        # 图像编解码
        nv12_codec_node, 
        # 启动零拷贝环境配置节点
        shared_mem_node,
        # 多画面调试页
        dashboard_node,
    ])
