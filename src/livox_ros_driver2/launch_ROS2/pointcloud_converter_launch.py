import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file for Livox pointcloud converter node"""

    converter_node = Node(
        package='livox_ros_driver2',
        executable='pointcloud_converter_node',
        name='livox_pointcloud_converter',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/sensing/lidar/top/pointcloud_raw_ex',
            'output_frame_id': 'velodyne_top_base_link'
        }]
    )

    return LaunchDescription([
        converter_node
    ])
