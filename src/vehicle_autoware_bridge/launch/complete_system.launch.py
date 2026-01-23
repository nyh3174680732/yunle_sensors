#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Complete System Launch File
启动传感器、底盘驱动和桥接节点的完整系统
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    完整系统启动：
    1. 传感器 (Livox, IMU, GNSS)
    2. 底盘驱动 (如果需要)
    3. 桥接节点 (vehicle_autoware_bridge)
    """

    # 参数声明
    launch_sensors_arg = DeclareLaunchArgument(
        'launch_sensors',
        default_value='true',
        description='Launch all sensors (Livox, IMU, GNSS)'
    )

    launch_chassis_arg = DeclareLaunchArgument(
        'launch_chassis',
        default_value='false',
        description='Launch chassis driver (set to false if already running)'
    )

    # ============== 1. 传感器启动 ==============
    try:
        sensor_bringup_dir = get_package_share_directory('sensor_bringup')
        sensors_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                sensor_bringup_dir, '/launch/all_sensors_launch.py'
            ])
        )
    except:
        print("Warning: sensor_bringup package not found, sensors will not be launched")
        sensors_launch = None

    # ============== 2. 底盘驱动启动 (可选) ==============
    # 如果底盘驱动已经在单独终端运行，可以设置 launch_chassis:=false
    try:
        chassis_driver_dir = get_package_share_directory('chassis_driver')
        chassis_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                chassis_driver_dir, '/launch/chassis_driver.launch.py'
            ])
        )
    except:
        print("Warning: chassis_driver package not found")
        chassis_launch = None

    # ============== 3. 桥接节点启动 ==============
    bridge_dir = get_package_share_directory('vehicle_autoware_bridge')
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            bridge_dir, '/launch/vehicle_autoware_bridge.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )

    # 构建 launch 描述
    launch_items = [
        launch_sensors_arg,
        launch_chassis_arg,
        bridge_launch,
    ]

    if sensors_launch:
        launch_items.append(sensors_launch)

    # 注意：底盘驱动通常需要单独启动，因为它可能需要特殊权限
    # 如果需要自动启动，取消下面的注释
    if chassis_launch:
        launch_items.append(chassis_launch)

    return LaunchDescription(launch_items)
