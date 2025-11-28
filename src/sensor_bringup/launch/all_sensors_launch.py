#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
All Sensors Launch File
同时启动所有传感器：GPS、IMU、Livox 激光雷达和点云转换器
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    生成 launch 描述，同时启动所有传感器节点：
    1. Livox MID360 激光雷达驱动
    2. 点云格式转换器 (Livox -> Extended Format)
    3. IMU/AHRS 驱动
    4. GPS 转换节点
    """

    # ============== 声明启动参数 ==============
    # GPS 串口参数
    gps_serial_port_arg = DeclareLaunchArgument(
        'gps_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for GNSS receiver'
    )

    gps_baudrate_arg = DeclareLaunchArgument(
        'gps_baudrate',
        default_value='115200',
        description='Baudrate for GNSS serial communication'
    )

    # IMU 串口参数
    imu_serial_port_arg = DeclareLaunchArgument(
        'imu_serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for IMU/AHRS'
    )

    imu_baudrate_arg = DeclareLaunchArgument(
        'imu_baudrate',
        default_value='921600',
        description='Baudrate for IMU serial communication'
    )

    # ============== 1. Livox 激光雷达配置 ==============
    livox_config_path = get_package_share_directory('livox_ros_driver2')
    user_config_path = os.path.join(livox_config_path, 'config', 'MID360_config.json')

    livox_ros2_params = [
        {"xfer_format": 0},              # 0-Pointcloud2, 1-自定义点云格式
        {"multi_topic": 0},              # 0-所有雷达共享一个话题
        {"data_src": 0},                 # 0-激光雷达
        {"publish_freq": 10.0},          # 发布频率 10Hz
        {"output_data_type": 0},         # 输出数据类型
        {"frame_id": 'livox_frame'},     # 坐标系ID
        {"lvx_file_path": '/home/livox/livox_test.lvx'},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": 'livox0000000001'}
    ]

    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # ============== 2. 点云转换器配置 ==============
    pointcloud_converter_node = Node(
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

    # ============== 3. IMU/AHRS 驱动配置 ==============
    imu_driver_node = Node(
        package='fdilink_ahrs',
        executable='ahrs_driver_node',
        name='ahrs_driver_node',
        output='screen',
        parameters=[{
            'if_debug_': False,
            'serial_port_': LaunchConfiguration('imu_serial_port'),
            'serial_baud_': LaunchConfiguration('imu_baudrate'),
            'imu_topic': '/imu/data',
            'imu_frame_id_': 'imu_link',
            'mag_pose_2d_topic': '/imu/mag_pose_2d',
            'Magnetic_topic': '/imu/magnetic',
            'Euler_angles_topic': '/imu/euler_angles',
            'gps_topic': '/imu/gps_fix',
            'twist_topic': '/imu/twist',
            'NED_odom_topic': '/imu/ned_odometry',
            'device_type_': 1
        }]
    )

    # ============== 4. GPS 转换器配置 ==============
    gnss_converter_node = Node(
        package='gnss_converter',
        executable='gnss_converter_node',
        name='gnss_converter_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('gps_serial_port'),
            'baudrate': LaunchConfiguration('gps_baudrate'),
            'frame_id': 'gnss_link',
        }]
    )

    # ============== 构建 Launch Description ==============
    return LaunchDescription([
        # 声明参数
        gps_serial_port_arg,
        gps_baudrate_arg,
        imu_serial_port_arg,
        imu_baudrate_arg,

        # 启动节点
        livox_driver_node,
        pointcloud_converter_node,
        imu_driver_node,
        gnss_converter_node,
    ])
