#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Vehicle-Autoware Bridge Launch File
启动所有桥接节点，连接真实车辆传感器/底盘与 Autoware 系统
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
    生成 launch 描述，启动所有桥接节点：
    1. Autoware 控制命令 → 底盘 ECU 命令
    2. 底盘状态 → Autoware 车辆状态
    3. IMU 话题重映射
    4. GNSS 坐标转换
    5. 传感器启动 (可选)
    """

    # ============== 参数声明 ==============
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (false for real vehicle)'
    )

    launch_sensors_arg = DeclareLaunchArgument(
        'launch_sensors',
        default_value='true',
        description='Launch sensor nodes (Livox, IMU, GNSS)'
    )

    # ============== 配置文件路径 ==============
    pkg_share_dir = get_package_share_directory('vehicle_autoware_bridge')
    config_file = os.path.join(pkg_share_dir, 'config', 'vehicle_params.yaml')

    # ============== 1. Autoware → 底盘控制转换器 ==============
    autoware_to_chassis_node = Node(
        package='vehicle_autoware_bridge',
        executable='autoware_to_chassis_node',
        name='autoware_to_chassis_converter',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Autoware 标准控制命令话题
            ('/control/command/control_cmd', '/control/command/control_cmd'),
        ]
    )

    # ============== 2. 底盘 → Autoware 状态转换器 ==============
    chassis_to_autoware_node = Node(
        package='vehicle_autoware_bridge',
        executable='chassis_to_autoware_node',
        name='chassis_to_autoware_converter',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Autoware 标准车辆状态话题
            ('/vehicle/status/control_mode', '/vehicle/status/control_mode'),
            ('/vehicle/status/gear_status', '/vehicle/status/gear_status'),
            ('/vehicle/status/hazard_lights_status', '/vehicle/status/hazard_lights_status'),
            ('/vehicle/status/steering_status', '/vehicle/status/steering_status'),
            ('/vehicle/status/turn_indicators_status', '/vehicle/status/turn_indicators_status'),
            ('/vehicle/status/velocity_status', '/vehicle/status/velocity_status'),
        ]
    )

    # # ============== 3. 点云转换器配置 ==============
    # pointcloud_to_autoware_converter_node = Node(
    #     package='vehicle_autoware_bridge',
    #     executable='pointcloud_to_autoware_converter_node',
    #     name='pointcloud_to_autoware',
    #     output='screen',
    #     parameters=[{
    #         'input_topic': '/livox/lidar',
    #         'output_topic': '/sensing/lidar/top/pointcloud_raw_ex',
    #         'output_frame_id': 'velodyne_top_base_link'
    #     }]
    # )

    # ============== 4. IMU to GNSS Orientation 转换器 ==============
    # 将 IMU 姿态转换为 gnss_poser 需要的 GnssInsOrientationStamped 格式
    imu_to_gnss_orientation_node = Node(
        package='vehicle_autoware_bridge',
        executable='imu_to_gnss_orientation_node',
        name='imu_to_gnss_orientation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'input_topic': '/sensing/imu/tamagawa/imu_raw'},
            {'output_topic': '/autoware_orientation'},
            {'default_rmse_rotation': 0.1}
        ]
    )

    # ============== 5. 传感器启动 (可选) ==============
    # 包含 sensor_bringup 的 all_sensors_launch.py
    sensor_bringup_dir = get_package_share_directory('sensor_bringup')
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            sensor_bringup_dir, '/launch/all_sensors_launch.py'
        ]),
        condition=lambda: LaunchConfiguration('launch_sensors').perform(None) == 'true'
    )

    # ============== 构建 Launch Description ==============
    return LaunchDescription([
        # 参数声明
        use_sim_time_arg,
        launch_sensors_arg,

        # 桥接节点
        autoware_to_chassis_node,
        chassis_to_autoware_node,
        #imu_to_gnss_orientation_node,
        #pointcloud_to_autoware_converter_node
        # 传感器启动 (可选)
        # sensors_launch,  # 如果需要自动启动传感器，取消此注释
    ])
