#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
统一启动文件 - 同时启动激光雷达、IMU和GPS转换
Author: yunle_sensors
Date: 2025-11-28
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    生成launch描述，同时启动三个传感器节点：
    1. Livox激光雷达 (MID360)
    2. IMU/AHRS驱动
    3. GPS转换节点
    """

    # ============== 激光雷达配置 ==============
    try:
        livox_config_path = get_package_share_directory('livox_ros_driver2')
        livox_config_path = os.path.join(livox_config_path, 'config')
        user_config_path = os.path.join(livox_config_path, 'MID360_config.json')
    except:
        # 如果找不到包，使用相对路径
        user_config_path = '/home/cplus/Desktop/yunle_sensors/src/livox_ros_driver2/config/MID360_config.json'

    # 激光雷达参数
    livox_ros2_params = [
        {"xfer_format": 1},              # 0-Pointcloud2, 1-自定义点云格式
        {"multi_topic": 0},              # 0-所有雷达共享一个话题, 1-每个雷达一个话题
        {"data_src": 0},                 # 0-激光雷达
        {"publish_freq": 10.0},          # 发布频率 Hz
        {"output_data_type": 0},         # 输出数据类型
        {"frame_id": 'livox_frame'},     # 坐标系ID
        {"lvx_file_path": '/home/livox/livox_test.lvx'},  # LVX文件路径
        {"user_config_path": user_config_path},           # 配置文件路径
        {"cmdline_input_bd_code": 'livox0000000001'}      # 设备代码
    ]

    # ============== IMU/AHRS配置 ==============
    # IMU参数
    ahrs_params = {
        'if_debug_': False,
        'serial_port_': '/dev/ttyUSB0',      # IMU串口
        'serial_baud_': 921600,              # 波特率
        'imu_topic': '/imu',
        'imu_frame_id_': 'gyro_link',
        'mag_pose_2d_topic': '/mag_pose_2d',
        'Magnetic_topic': '/magnetic',
        'Euler_angles_topic': '/euler_angles',
        'gps_topic': '/gps/fix',
        'twist_topic': '/system_speed',
        'NED_odom_topic': '/NED_odometry',
        'device_type_': 1
    }

    # ============== GPS转换配置 ==============
    # GPS参数声明
    gps_serial_port_arg = DeclareLaunchArgument(
        'gps_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for GNSS receiver'
    )

    gps_baudrate_arg = DeclareLaunchArgument(
        'gps_baudrate',
        default_value='115200',
        description='Baudrate for GPS serial communication'
    )

    gps_frame_id_arg = DeclareLaunchArgument(
        'gps_frame_id',
        default_value='gnss_link',
        description='Frame ID for GNSS data'
    )

    # ============== 创建节点 ==============

    # 1. 激光雷达节点
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # 2. IMU/AHRS驱动节点
    ahrs_driver_node = Node(
        package='fdilink_ahrs',
        executable='ahrs_driver_node',
        name='ahrs_driver',
        output='screen',
        parameters=[ahrs_params]
    )

    # 3. GPS转换节点
    gnss_converter_node = Node(
        package='gnss_converter',
        executable='gnss_converter_node',
        name='gnss_converter_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('gps_serial_port'),
            'baudrate': LaunchConfiguration('gps_baudrate'),
            'frame_id': LaunchConfiguration('gps_frame_id'),
        }]
    )

    # ============== 返回Launch描述 ==============
    return LaunchDescription([
        # 声明参数
        gps_serial_port_arg,
        gps_baudrate_arg,
        gps_frame_id_arg,

        # 启动节点
        livox_driver_node,
        ahrs_driver_node,
        gnss_converter_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
